#!/usr/bin/env python3
"""Named-target admission exercise. It never opens an SDK or network connection.

The output is a list of twelve checked teaching records, not a LowCmd packet.
Bounds here are deliberately illustrative, not a calibrated robot envelope.
"""
import argparse
import copy
import json
import math
from pathlib import Path

JOINT_NAMES = tuple(f"{leg}_{part}_joint" for leg in ("FR", "FL", "RR", "RL")
                    for part in ("hip", "thigh", "calf"))
POSITION_BOUNDS = {"hip": (-0.5, 0.5), "thigh": (0.2, 1.8), "calf": (-2.6, -0.8)}
# These small numbers are for the offline lesson, not robot operating settings.
FIELD_BOUNDS = {"dq": (-1.0, 1.0), "tau": (-2.0, 2.0), "kp": (0.0, 20.0), "kd": (0.0, 2.0)}


def number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label}: expected a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{label}: expected a finite number")
    return value


class OfflineBridge:
    """Only an admission decision; contains no sender or SDK packet builder."""

    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True

    def check(self, frame, *, now, previous=None):
        if self.closed:
            return {"accepted": False, "reason": "bridge is closed", "sdk_targets": []}
        try:
            now = number(now, "now")
            if not isinstance(frame, dict):
                raise ValueError("frame must be an object")
            for key in ("command_stamp", "feedback_stamp"):
                age = now - number(frame[key], key)
                if age < -0.005:
                    raise ValueError(f"{key}: future timestamp or mismatched clocks")
                if age > 0.1:
                    raise ValueError(f"{key}: stale (more than 0.1 s old)")
            if not isinstance(frame["targets"], list):
                raise ValueError("targets must be a list")
            named = {}
            for target in frame["targets"]:
                if not isinstance(target, dict):
                    raise ValueError("every target must be an object")
                name = target["name"]
                if name not in JOINT_NAMES:
                    raise ValueError(f"unknown joint: {name}")
                if name in named:
                    raise ValueError(f"duplicate joint: {name}")
                named[name] = target
            if set(named) != set(JOINT_NAMES):
                raise ValueError("missing joint: expected all twelve named joints")
            # A fresh timestamp alone is not feedback. The actual named angle
            # values are needed to check every target's change from feedback.
            if previous is None:
                previous = frame.get("feedback_positions")
            if not isinstance(previous, dict):
                raise ValueError("feedback_positions: twelve named feedback angles are required")

            checked = []
            for index, name in enumerate(JOINT_NAMES):
                source = named[name]
                values = {field: number(source[field], f"{name}.{field}")
                          for field in ("q", "dq", "tau", "kp", "kd")}
                bounds = {"q": POSITION_BOUNDS[name.split("_")[1]], **FIELD_BOUNDS}
                for field, (lower, upper) in bounds.items():
                    if not lower <= values[field] <= upper:
                        raise ValueError(f"{name}.{field}: outside teaching bounds [{lower}, {upper}]")
                if name not in previous:
                    raise ValueError(f"missing previous feedback for {name}")
                old = number(previous[name], f"previous.{name}")
                if abs(values["q"] - old) > 0.05:
                    raise ValueError(f"{name}.q: step exceeds teaching limit 0.05 rad")
                checked.append({"index": index, "name": name, "q": values["q"],
                                "dq": values["dq"], "tau": values["tau"],
                                "Kp": values["kp"], "Kd": values["kd"]})
            return {"accepted": True, "reason": "offline checks passed; no packet sent", "sdk_targets": checked}
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            return {"accepted": False, "reason": str(error), "sdk_targets": []}


def demo_frame():
    return {"command_stamp": 9.98, "feedback_stamp": 9.99,
            "feedback_positions": {name: (0.0, 0.67, -1.3)[i % 3] for i, name in enumerate(JOINT_NAMES)},
            "targets": [{"name": name, "q": (0.0, 0.67, -1.3)[i % 3],
                         "dq": 0.0, "tau": 0.0, "kp": 10.0, "kd": 1.0}
                        for i, name in enumerate(JOINT_NAMES)]}


def demo_case(case, now=10.0):
    payload = copy.deepcopy(demo_frame())
    previous = {r["name"]: r["q"] for r in payload["targets"]}
    guard = OfflineBridge()
    if case == "stale":
        payload["command_stamp"] = 9.0
    elif case == "feedback_stale":
        payload["feedback_stamp"] = 9.0
    elif case == "missing":
        payload["targets"].pop()
    elif case == "duplicate":
        payload["targets"][-1] = copy.deepcopy(payload["targets"][0])
    elif case == "nan":
        payload["targets"][0]["q"] = float("nan")
    elif case == "effort":
        payload["targets"][0]["tau"] = 3.0
    elif case == "step":
        payload["targets"][0]["q"] = 0.2
    elif case == "closed":
        guard.close()
    return guard.check(payload, now=now, previous=previous)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--demo", action="store_true", help="run valid and rejected examples (default)")
    cases = ("valid", "stale", "feedback_stale", "missing", "duplicate", "nan", "effort", "step", "closed")
    mode.add_argument("--case", choices=cases, help="one case: accepted=exit0, rejected=exit1")
    mode.add_argument("--input", type=Path, help="check a JSON frame with the illustrative bounds")
    parser.add_argument("--now", type=float, default=10.0, help="time in the same local clock as frame stamps")
    parser.add_argument("--output", type=Path, help="also save the decision JSON")
    args = parser.parse_args(argv)
    try:
        number(args.now, "--now")
        if args.input:
            with args.input.open(encoding="utf-8") as stream:
                payload = json.load(stream)
            result = OfflineBridge().check(payload, now=args.now)
            exit_code = 0 if result["accepted"] else 1
        elif args.case:
            result = demo_case(args.case, args.now)
            exit_code = 0 if result["accepted"] else 1
        else:
            result = {case: demo_case(case, args.now) for case in cases}
            exit_code = 0 if all(d["accepted"] == (case == "valid") for case, d in result.items()) else 1
        text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        print(text)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text + "\n", encoding="utf-8")
        return exit_code
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
