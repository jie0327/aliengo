#!/usr/bin/env python3
"""Offline diagonal-trot phase/cycloid CSV, with no robot connection.

Leg order is FR FL RR RL; phase biases are 0 .5 .5 0. `contact` means the
scheduled support flag, not a measured contact. Local swing offsets use +x
forward and +z up (m). This omits WaveGenerator transition synchronization,
state estimation, landing point feedback, IK, and force control.
"""

import argparse
import csv
import math
import sys


LEGS = ("FR", "FL", "RR", "RL")
BIASES = (0., .5, .5, 0.)


def swing(phase, duration, stride, height):
    """Cycloid x,z offsets and dx/dt,dz/dt within one swing segment."""
    if not all(math.isfinite(v) for v in (phase, duration, stride, height)):
        raise ValueError("Swing inputs must be finite")
    if not 0 <= phase <= 1 or duration <= 0 or height < 0:
        raise ValueError("Use phase in [0,1], positive duration, nonnegative height")
    angle = 2*math.pi*phase
    return (stride*(angle-math.sin(angle))/(2*math.pi),
            height*(1-math.cos(angle))/2,
            stride*(1-math.cos(angle))/duration,
            height*math.pi*math.sin(angle)/duration)


def sample(time, period=.45, stance_ratio=.5, stride=.06, height=.04):
    if not all(math.isfinite(v) for v in (time, period, stance_ratio, stride, height)):
        raise ValueError("Timing and geometry must be finite")
    if time < 0 or period <= 0 or not 0 < stance_ratio < 1 or height < 0:
        raise ValueError("Use time>=0, period>0, 0<stance_ratio<1, height>=0")
    states = []
    for leg, bias in zip(LEGS, BIASES):
        cycle = (time/period - bias) % 1.
        # Stabilize exact cycle boundaries represented with floating roundoff.
        if math.isclose(cycle, 1., abs_tol=1e-12, rel_tol=0):
            cycle = 0.
        if math.isclose(cycle, stance_ratio, abs_tol=1e-12, rel_tol=0):
            cycle = stance_ratio
        contact = int(cycle < stance_ratio)
        phase = cycle/stance_ratio if contact else (cycle-stance_ratio)/(1-stance_ratio)
        offsets = (0., 0., 0., 0.) if contact else swing(phase, period*(1-stance_ratio), stride, height)
        states.append(dict(zip(("leg", "cycle", "contact", "phase", "swing_x_m", "swing_z_m",
                                "swing_vx_m_s", "swing_vz_m_s"),
                               (leg, cycle, contact, phase, *offsets))))
    return states


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--period", type=float, default=.45, help="Full gait period in seconds")
    parser.add_argument("--stance-ratio", type=float, default=.5)
    parser.add_argument("--duration", type=float, default=.9, help="Output time span in seconds")
    parser.add_argument("--dt", type=float, default=.05625, help="Sampling interval in seconds")
    parser.add_argument("--stride", type=float, default=.06, help="Local swing travel in metres")
    parser.add_argument("--height", type=float, default=.04, help="Local swing peak height in metres")
    args = parser.parse_args()
    try:
        if not math.isfinite(args.duration) or not math.isfinite(args.dt) or args.duration < 0 or args.dt <= 0:
            raise ValueError("Use finite duration>=0 and dt>0")
        sample_intervals = args.duration/args.dt
        if not math.isfinite(sample_intervals) or sample_intervals >= 100000:
            raise ValueError("Output sample count must be at most 100000; increase --dt or reduce --duration")
        sample(0, args.period, args.stance_ratio, args.stride, args.height)
        fields = ["time_s", "leg", "cycle", "contact", "phase", "swing_x_m", "swing_z_m",
                  "swing_vx_m_s", "swing_vz_m_s"]
        writer = csv.DictWriter(sys.stdout, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for i in range(math.floor(sample_intervals + 1e-10)+1):
            time = i*args.dt
            for state in sample(time, args.period, args.stance_ratio, args.stride, args.height):
                writer.writerow({"time_s": time, **state})
    except ValueError as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
