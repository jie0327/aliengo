#!/usr/bin/env python3
"""Teaching 2R IK: origin at joint 1, +x right, +y up, CCW angles in rad.

This plane is independent of AlienGo's URDF joint conventions. Results are
offline mathematics and must not be sent to a robot.
"""

import argparse
import json
import math


def validate_lengths(l1, l2):
    if not all(math.isfinite(v) and v > 0 for v in (l1, l2)):
        raise ValueError("Link lengths must be finite and positive (m)")


def planar_fk(q1, q2, l1=.25, l2=.25):
    validate_lengths(l1, l2)
    if not all(math.isfinite(v) for v in (q1, q2)):
        raise ValueError("Joint angles must be finite (rad)")
    return (l1*math.cos(q1) + l2*math.cos(q1+q2),
            l1*math.sin(q1) + l2*math.sin(q1+q2))


def planar_ik(x, y, l1=.25, l2=.25, limits=None):
    """Return principal elbow branches; reject unreachable/underdetermined goals.

    Optional limits are ((q1_min,q1_max), (q2_min,q2_max)) in radians.
    Angles use the principal representation; no +/-2pi limit search is made.
    """
    validate_lengths(l1, l2)
    if not all(math.isfinite(v) for v in (x, y)):
        raise ValueError("Target coordinates must be finite (m)")
    radius = math.hypot(x, y)
    tolerance = 1e-12 * max(l1, l2)
    if radius > l1+l2+tolerance or radius < abs(l1-l2)-tolerance:
        raise ValueError("unreachable target: outside the two-link annulus")
    if radius <= tolerance and abs(l1-l2) <= tolerance:
        raise ValueError("singular target at origin: infinitely many folded joint-1 angles")
    if limits is not None:
        if len(limits) != 2 or any(len(pair) != 2 or
                                  not all(math.isfinite(v) for v in pair) or pair[0] > pair[1]
                                  for pair in limits):
            raise ValueError("limits must be two finite ordered angle ranges")
    cosine = (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
    cosine = max(-1., min(1., cosine))  # roundoff only, after reachability check
    elbow = math.acos(cosine)
    solutions = []
    # At straight/folded boundaries the two signs describe the same geometry.
    elbows = (elbow,) if abs(math.sin(elbow)) < 1e-12 else (elbow, -elbow)
    for q2 in elbows:
        q1 = math.atan2(y, x) - math.atan2(l2*math.sin(q2), l1+l2*math.cos(q2))
        q1 = math.atan2(math.sin(q1), math.cos(q1))
        if limits is None or all(low <= q <= high for q, (low, high) in zip((q1, q2), limits)):
            solutions.append((q1, q2))
    if not solutions:
        raise ValueError("reachable geometry, but no principal solution satisfies the joint limits")
    return solutions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group()
    choice.add_argument("--angles", type=float, nargs=2, metavar=("Q1_RAD", "Q2_RAD"))
    choice.add_argument("--target", type=float, nargs=2, metavar=("X_M", "Y_M"))
    parser.add_argument("--lengths", type=float, nargs=2, default=[.25, .25])
    parser.add_argument("--limits", type=float, nargs=4,
                        metavar=("Q1_MIN", "Q1_MAX", "Q2_MIN", "Q2_MAX"))
    args = parser.parse_args()
    try:
        lengths = args.lengths
        target = tuple(args.target) if args.target is not None else planar_fk(*(args.angles or [.6, -1.2]), *lengths)
        limits = None if args.limits is None else (args.limits[:2], args.limits[2:])
        solutions = planar_ik(*target, *lengths, limits=limits)
        print(json.dumps({"plane": "joint1 origin; +x right, +y up; CCW radians",
                          "target_m": target, "lengths_m": lengths}))
        for q in solutions:
            recovered = planar_fk(*q, *lengths)
            print(json.dumps({"angles_rad": q, "recovered_m": recovered,
                              "error_m": math.hypot(*(a-b for a, b in zip(recovered, target))),
                              "jacobian_determinant_m2": lengths[0]*lengths[1]*math.sin(q[1])}))
    except ValueError as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
