#!/usr/bin/env python3
"""Read-only AlienGo URDF FK. Lengths: m; angles: rad; result: trunk frame.

No ROS, publishers, SDK, or external Python packages are used. This is a
kinematic calculation, not a motor command generator.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


LEGS = ("FR", "FL", "RR", "RL")
PARTS = ("hip", "thigh", "calf")
DEFAULT_URDF = Path(__file__).resolve().parents[2] / ".ros1/aliengo.urdf"


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    """Compose two 4x4 transforms acting on column vectors."""
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def rotation(axis, angle):
    """Rodrigues rotation around a joint-local axis, right-hand rule."""
    norm = math.sqrt(sum(v*v for v in axis))
    if norm == 0 or not math.isfinite(norm) or not math.isfinite(angle):
        raise ValueError("Rotation axis must be nonzero and angle finite")
    x, y, z = (v / norm for v in axis)
    c, s, d = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    result = identity()
    result[:3] = [[c+x*x*d, x*y*d-z*s, x*z*d+y*s, 0],
                  [y*x*d+z*s, c+y*y*d, y*z*d-x*s, 0],
                  [z*x*d-y*s, z*y*d+x*s, c+z*z*d, 0]]
    return result


def origin_transform(xyz, rpy):
    # URDF RPY uses Rz(yaw) Ry(pitch) Rx(roll), with fixed parent axes.
    roll, pitch, yaw = rpy
    result = multiply(multiply(rotation((0, 0, 1), yaw),
                               rotation((0, 1, 0), pitch)),
                      rotation((1, 0, 0), roll))
    for i in range(3):
        result[i][3] = xyz[i]
    return result


def vector(element, attribute, default):
    values = tuple(float(v) for v in element.get(attribute, default).split())
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"Invalid URDF vector: {attribute}")
    return values


class UrdfModel:
    """Minimal tree FK for fixed/revolute/continuous joints in this lesson.

    `forward` returns T_root_tip (4x4 lists). Every moving joint on the chain
    requires an explicitly named position; omitted joints never silently use 0.
    """

    def __init__(self, path):
        path = Path(path)
        if not path.is_file():
            raise ValueError(f"Missing URDF: {path}. Generate it with "
                             "./scripts/ros1_model.sh")
        robot = ET.parse(path).getroot()
        self.links = {link.attrib["name"] for link in robot.findall("link")}
        self.by_child = {}
        for element in robot.findall("joint"):
            origin = element.find("origin")
            axis = element.find("axis")
            limit = element.find("limit")
            xyz = (0., 0., 0.) if origin is None else vector(origin, "xyz", "0 0 0")
            rpy = (0., 0., 0.) if origin is None else vector(origin, "rpy", "0 0 0")
            joint = {"name": element.attrib["name"], "type": element.attrib["type"],
                     "parent": element.find("parent").attrib["link"],
                     "child": element.find("child").attrib["link"],
                     "xyz": xyz, "rpy": rpy, "origin": origin_transform(xyz, rpy),
                     "axis": (1., 0., 0.) if axis is None else vector(axis, "xyz", "1 0 0"),
                     "lower": None if limit is None or "lower" not in limit.attrib else float(limit.attrib["lower"]),
                     "upper": None if limit is None or "upper" not in limit.attrib else float(limit.attrib["upper"])}
            if joint["child"] in self.by_child:
                raise ValueError(f"URDF has multiple parents: {joint['child']}")
            self.by_child[joint["child"]] = joint

    def chain(self, root, tip):
        if root not in self.links or tip not in self.links:
            raise ValueError(f"Unknown root/tip link: {root}, {tip}")
        reverse, seen = [], set()
        while tip != root:
            if tip in seen or tip not in self.by_child:
                raise ValueError(f"No tree chain from {root} to {tip}")
            seen.add(tip)
            joint = self.by_child[tip]
            reverse.append(joint)
            tip = joint["parent"]
        return list(reversed(reverse))

    def forward(self, root, tip, positions, check_limits=True):
        transform = identity()
        for joint in self.chain(root, tip):
            transform = multiply(transform, joint["origin"])
            kind, name = joint["type"], joint["name"]
            if kind == "fixed":
                continue
            if kind not in ("revolute", "continuous"):
                raise ValueError(f"Unsupported joint type for this lesson: {kind}")
            if name not in positions or not math.isfinite(positions[name]):
                raise ValueError(f"Missing or nonfinite angle: {name}")
            angle = positions[name]
            if check_limits and kind == "revolute":
                low, high = joint["lower"], joint["upper"]
                if low is None or high is None or not low <= angle <= high:
                    raise ValueError(f"{name}: {angle} rad outside URDF limits [{low}, {high}]")
            transform = multiply(transform, rotation(joint["axis"], angle))
        return transform


def foot_position(model, leg, angles, check_limits=True):
    if leg not in LEGS or len(angles) != 3:
        raise ValueError("Use FR/FL/RR/RL and exactly three angles: hip thigh calf")
    positions = {f"{leg}_{part}_joint": angle for part, angle in zip(PARTS, angles)}
    transform = model.forward("trunk", f"{leg}_foot", positions, check_limits)
    return tuple(transform[i][3] for i in range(3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--leg", choices=LEGS, default="FR")
    parser.add_argument("--angles", type=float, nargs=3, default=[0, .8, -1.6],
                        metavar=("HIP_RAD", "THIGH_RAD", "CALF_RAD"))
    parser.add_argument("--all-legs", action="store_true")
    parser.add_argument("--allow-outside-limits", action="store_true",
                        help="Mathematical reference only; does not make a pose physically admissible")
    parser.add_argument("--describe", action="store_true", help="Print chain origins, axes, and limits")
    args = parser.parse_args()
    try:
        model = UrdfModel(args.urdf)
        if args.describe:
            for joint in model.chain("trunk", f"{args.leg}_foot"):
                print(json.dumps({key: value for key, value in joint.items() if key != "origin"}))
        for leg in LEGS if args.all_legs else (args.leg,):
            xyz = foot_position(model, leg, args.angles, not args.allow_outside_limits)
            print(json.dumps({"leg": leg, "frame": "trunk", "point": f"{leg}_foot origin",
                              "angles_rad": args.angles, "xyz_m": xyz,
                              "limits_checked": not args.allow_outside_limits}))
    except (ValueError, ET.ParseError, OSError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
