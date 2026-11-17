#!/usr/bin/env python3
"""Offline 45D actor contract with explicit action fixtures, never learned inference.

All times are seconds on one synthetic monotonic timeline. Returned targets are
inspection records, not ROS messages, SDK packets, or hardware-safe commands.
"""
import argparse
import copy
import json
import math
from pathlib import Path

JOINT_NAMES = [f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL")
               for joint in ("hip", "thigh", "calf")]
OBSERVATION_TERMS = ["body_omega", "projected_gravity", "command",
                     "q_minus_default", "dq", "previous_action"]
ACTION_FIXTURE = [2.0, -2.0, 0.5] * 4


def default_metadata():
    """A teaching schema, deliberately unrelated to a trained policy checkpoint."""
    return {"schema": "lesson12-fixture-v1", "observation_dim": 45,
            "observation_terms": OBSERVATION_TERMS[:], "quaternion_order": "xyzw",
            "quaternion_rotation": "body_to_world", "world_up": "z",
            "joint_names": JOINT_NAMES[:], "default_q": [0.0, 0.8, -1.5] * 4,
            "omega_scale": 0.25, "command_scale": [2.0, 2.0, 0.25],
            "q_scale": 1.0, "dq_scale": 0.05, "clip_observation": 10.0,
            "clip_action": 1.0, "action_scale": 0.25,
            "history_steps": 3, "history_order": "oldest_to_newest_zero_padding",
            "pd_dt": 0.002, "decimation": 10, "policy_dt": 0.02,
            "dt_tolerance": 0.0005, "max_age": 0.05}


def finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number (not bool)")
    return float(value)


def vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{label} must have {size} values")
    return [finite(item, f"{label}[{i}]") for i, item in enumerate(value)]


def clip(values, limit):
    # Check arithmetic overflow before clipping so it cannot disguise bad data.
    return [max(-limit, min(limit, finite(v, "computed value"))) for v in values]


class PolicyContract:
    """Count accepted feedback ticks, build actor inputs and inspect fake actions.

    Invalid data latches DEGRADED. Only reset() starts a new sequence. Reset also
    permits an intentional clock restart, clears previous actions and zero pads
    history. This models admission, not a robot's fallback actuator behavior.
    """
    def __init__(self, metadata=None):
        self.metadata = copy.deepcopy(default_metadata() if metadata is None else metadata)
        self._validate_metadata()
        self.reset()

    def _validate_metadata(self):
        m = self.metadata
        if not isinstance(m, dict):
            raise ValueError("metadata must be an object")
        expected = {"schema": "lesson12-fixture-v1", "observation_dim": 45,
                    "observation_terms": OBSERVATION_TERMS, "quaternion_order": "xyzw",
                    "quaternion_rotation": "body_to_world", "world_up": "z",
                    "history_order": "oldest_to_newest_zero_padding"}
        for key, value in expected.items():
            if m.get(key) != value:
                raise ValueError(f"incompatible metadata: {key}")
        names = m.get("joint_names")
        if (not isinstance(names, list) or len(names) != 12
                or not all(isinstance(name, str) for name in names)
                or len(set(names)) != 12 or set(names) != set(JOINT_NAMES)):
            raise ValueError("joint_names must contain the 12 unique named joints")
        m["default_q"] = vector(m.get("default_q"), 12, "default_q")
        m["command_scale"] = vector(m.get("command_scale"), 3, "command_scale")
        for key in ("omega_scale", "q_scale", "dq_scale", "clip_observation", "clip_action",
                    "action_scale", "pd_dt", "policy_dt", "dt_tolerance", "max_age"):
            m[key] = finite(m.get(key), key)
            if m[key] <= 0:
                raise ValueError(f"{key} must be positive")
        if any(scale <= 0 for scale in m["command_scale"]):
            raise ValueError("command_scale must be positive")
        # Teaching workload bounds also catch accidentally pasted huge counts.
        for key, maximum in (("decimation", 1000), ("history_steps", 64)):
            if type(m.get(key)) is not int or not 1 <= m[key] <= maximum:
                raise ValueError(f"{key} must be an integer in [1, {maximum}]")
        if not math.isclose(m["policy_dt"], m["pd_dt"] * m["decimation"], rel_tol=1e-12):
            raise ValueError("policy_dt must equal pd_dt * decimation")
        if m["dt_tolerance"] >= m["pd_dt"]:
            raise ValueError("dt_tolerance must be smaller than pd_dt")

    def reset(self):
        self.state = "WAITING"
        self.last_stamp = None
        self.tick = 0
        self.previous_action = [0.0] * 12
        self.history = [[0.0] * 45 for _ in range(self.metadata["history_steps"])]
        self.targets = []

    def _observation(self, frame):
        m = self.metadata
        omega = vector(frame.get("body_omega"), 3, "body_omega")
        command = vector(frame.get("command"), 3, "command")
        quat = vector(frame.get("orientation_xyzw"), 4, "orientation_xyzw")
        norm = math.hypot(*quat)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError("quaternion norm must be finite and at least 1e-12")
        x, y, z, w = [value / norm for value in quat]
        # R(body->world)^T * [0,0,-1]; direction only, not m/s^2.
        gravity = [2 * (w * y - x * z), -2 * (w * x + y * z), 2 * (x * x + y * y) - 1]
        joints = frame.get("joints")
        if not isinstance(joints, list) or len(joints) != 12:
            raise ValueError("joints must have exactly 12 records")
        named = {}
        for joint in joints:
            if not isinstance(joint, dict) or not isinstance(joint.get("name"), str):
                raise ValueError("joint record needs a name")
            name = joint["name"]
            if name not in m["joint_names"] or name in named:
                raise ValueError(f"unknown or duplicate joint: {name}")
            named[name] = (finite(joint.get("q"), name + ".q"),
                           finite(joint.get("dq"), name + ".dq"))
        q = [named[name][0] for name in m["joint_names"]]
        dq = [named[name][1] for name in m["joint_names"]]
        obs = ([v * m["omega_scale"] for v in omega] + gravity
               + [v * scale for v, scale in zip(command, m["command_scale"])]
               + [(v - default) * m["q_scale"] for v, default in zip(q, m["default_q"])]
               + [v * m["dq_scale"] for v in dq] + self.previous_action[:])
        return clip(obs, m["clip_observation"])

    def step(self, frame, *, now, action_fixture):
        """Process one ideal PD feedback tick; fixtures substitute for model output.

        A single stamp represents an already synchronized complete snapshot. A
        real adapter must independently check each source's freshness and skew.
        All inputs are checked on every tick, even while holding the last target.
        """
        if self.state == "DEGRADED":
            return {"accepted": False, "state": self.state, "reason": "explicit reset required", "targets": []}
        try:
            if not isinstance(frame, dict):
                raise ValueError("frame must be an object")
            now = finite(now, "now")
            stamp = finite(frame.get("stamp"), "stamp")
            age = now - stamp
            if not math.isfinite(age) or age < 0 or age > self.metadata["max_age"]:
                raise ValueError("snapshot is stale or in the future")
            if self.last_stamp is not None:
                delta = stamp - self.last_stamp
                if delta <= 0:
                    raise ValueError("duplicate or backwards snapshot stamp")
                if abs(delta - self.metadata["pd_dt"]) > self.metadata["dt_tolerance"]:
                    raise ValueError("feedback interval violates pd_dt tolerance")
            observation = self._observation(frame)
            fixture = vector(action_fixture, 12, "action_fixture")
            due = self.tick % self.metadata["decimation"] == 0
            if due:
                bounded = clip(fixture, self.metadata["clip_action"])
                q_des = [finite(default + self.metadata["action_scale"] * action, "q_des")
                         for default, action in zip(self.metadata["default_q"], bounded)]
                targets = [{"index": index, "name": name, "q_des": q_des[index]}
                           for index, name in enumerate(self.metadata["joint_names"])]
                # Commit all state only after the full frame and output validate.
                self.history = self.history[1:] + [observation]
                self.previous_action = bounded
                self.targets = targets
            self.last_stamp = stamp
            self.tick += 1
            self.state = "ACTIVE"
            return {"accepted": True, "state": self.state, "policy_due": due,
                    "source": "explicit_action_fixture_no_model", "observation": observation,
                    "history": [v for obs in self.history for v in obs],
                    "action_used": self.previous_action[:], "targets": copy.deepcopy(self.targets)}
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.state = "DEGRADED"
            return {"accepted": False, "state": self.state, "reason": str(error), "targets": []}


def demo_frame(stamp=9.99):
    return {"stamp": stamp, "body_omega": [2.0, -4.0, 1.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0], "command": [0.5, -0.25, 2.0],
            "joints": [{"name": name, "q": [0.0, 0.8, -1.5][index % 3], "dq": 0.0}
                       for index, name in enumerate(JOINT_NAMES)]}


CASES = ("valid", "shuffled", "stale", "future", "missing", "duplicate_joint", "nan",
         "zero_quaternion", "duplicate_stamp", "backwards_stamp", "bad_dt", "bad_action")


def run_case(name, metadata):
    contract = PolicyContract(metadata)
    payload = demo_frame()
    action = ACTION_FIXTURE[:]
    if name == "shuffled":
        payload["joints"].reverse()
    elif name == "stale":
        payload["stamp"] = 9.0
    elif name == "future":
        payload["stamp"] = 10.001
    elif name == "missing":
        payload["joints"].pop()
    elif name == "duplicate_joint":
        payload["joints"][-1]["name"] = JOINT_NAMES[0]
    elif name == "nan":
        payload["body_omega"][0] = float("nan")
    elif name == "zero_quaternion":
        payload["orientation_xyzw"] = [0.0] * 4
    elif name == "bad_action":
        action[0] = float("inf")
    elif name in ("duplicate_stamp", "backwards_stamp", "bad_dt"):
        contract.step(payload, now=10.0, action_fixture=action)
        payload["stamp"] = {"duplicate_stamp": 9.99, "backwards_stamp": 9.988, "bad_dt": 9.998}[name]
    return contract.step(payload, now=10.0, action_fixture=action)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--case", choices=CASES, default="valid")
    source.add_argument("--demo", action="store_true", help="pass only if valid cases pass and bad cases reject")
    source.add_argument("--input", type=Path, help="JSON object: frame and action_fixture; one fresh sequence")
    parser.add_argument("--metadata", type=Path, help="explicit teaching schema JSON")
    parser.add_argument("--now", type=float, default=10.0,
                        help="only with --input: synthetic now in the same clock as frame.stamp")
    parser.add_argument("--output", type=Path, help="save result JSON, not a reusable frame")
    args = parser.parse_args()
    try:
        metadata = json.loads(args.metadata.read_text()) if args.metadata else default_metadata()
        if args.demo:
            cases = {name: run_case(name, metadata) for name in CASES}
            sequence = PolicyContract(metadata)
            policy_ticks = []
            for tick in range(21):
                stamp = 9.95 + tick * metadata["pd_dt"]
                decision = sequence.step(demo_frame(stamp), now=stamp + 0.001, action_fixture=ACTION_FIXTURE)
                if not decision["accepted"]:
                    raise ValueError(decision["reason"])
                if decision["policy_due"]:
                    policy_ticks.append(tick)
            sequence.reset()
            after_reset = sequence.step(demo_frame(1.0), now=1.001, action_fixture=[0.0] * 12)
            passed = (all(result["accepted"] == (name in ("valid", "shuffled"))
                          for name, result in cases.items()) and after_reset["accepted"])
            result = {"demo_passed": passed, "source": "explicit_action_fixture_no_model",
                      "observation_dim": metadata["observation_dim"],
                      "history_dim": metadata["observation_dim"] * metadata["history_steps"],
                      "example_pd_hz": 1 / metadata["pd_dt"],
                      "example_policy_hz": 1 / metadata["policy_dt"], "policy_ticks": policy_ticks,
                      "reset_previous_action": after_reset["observation"][33:], "cases": cases}
            status = 0 if passed else 1
        elif args.input:
            payload = json.loads(args.input.read_text())
            if not isinstance(payload, dict):
                raise ValueError("input must be an object containing frame and action_fixture")
            result = PolicyContract(metadata).step(payload.get("frame"), now=args.now,
                                                   action_fixture=payload.get("action_fixture"))
            status = 0 if result["accepted"] else 1
        else:
            result = run_case(args.case, metadata)
            status = 0 if result["accepted"] else 1
        rendered = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered + "\n")
        print(rendered)
        return status
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
