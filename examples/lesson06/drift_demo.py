#!/usr/bin/env python3
"""Pure-Python IMU conventions and constant acceleration bias experiment."""
import argparse
import csv
import math
from pathlib import Path


def finite_values(values, size=None):
    result = tuple(float(value) for value in values)
    if (size is not None and len(result) != size) or not all(math.isfinite(v) for v in result):
        raise ValueError('expected finite values with the required dimensions')
    return result


def wxyz_to_xyzw(quaternion):
    w, x, y, z = finite_values(quaternion, 4)
    return x, y, z, w


def rotate_xyzw(quaternion, vector):
    """Rotate a sensor/body vector into world; q denotes body-to-world."""
    x, y, z, w = finite_values(quaternion, 4)
    vx, vy, vz = finite_values(vector, 3)
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('quaternion norm must be finite and nonzero')
    x, y, z, w = (component / norm for component in (x, y, z, w))
    return (
        (1 - 2 * (y*y + z*z)) * vx + 2 * (x*y - w*z) * vy + 2 * (x*z + w*y) * vz,
        2 * (x*y + w*z) * vx + (1 - 2 * (x*x + z*z)) * vy + 2 * (y*z - w*x) * vz,
        2 * (x*z - w*y) * vx + 2 * (y*z + w*x) * vy + (1 - 2 * (x*x + y*y)) * vz,
    )


def world_acceleration(quaternion_xyzw, specific_force_sensor):
    """a_world = R_world_sensor f_sensor + g_world (z up, SI units)."""
    x, y, z = rotate_xyzw(quaternion_xyzw, specific_force_sensor)
    return x, y, z - 9.81


def checked_dt(previous, current, max_dt=0.1):
    previous, current, max_dt = finite_values((previous, current, max_dt))
    dt = current - previous
    if max_dt <= 0 or dt <= 0 or dt > max_dt:
        raise ValueError('timestamp must advance with 0 < dt <= max_dt; reset integration on clock reset')
    return dt


def integrate_bias(bias=0.02, duration=10.0, dt=0.01):
    """Integrate constant *world linear acceleration error*, not raw IMU."""
    bias, duration, dt = finite_values((bias, duration, dt))
    if dt <= 0 or duration < 0:
        raise ValueError('dt must be positive and duration nonnegative')
    if duration / dt > 1_000_000:
        raise ValueError('experiment limited to one million integration steps')
    t = velocity = position = 0.0
    rows = [(t, velocity, position)]
    while t < duration:
        next_t = min((len(rows)) * dt, duration)
        step = checked_dt(t, next_t, max_dt=dt * (1 + 1e-9))
        position += velocity * step + 0.5 * bias * step * step
        velocity += bias * step
        if not all(math.isfinite(value) for value in (position, velocity)):
            raise ValueError('integration produced non-finite state; check bias, duration and dt')
        t = next_t
        rows.append((t, velocity, position))
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bias', type=float, default=0.02, help='constant world acceleration error, m/s^2')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--dt', type=float, default=0.01)
    parser.add_argument('--csv', type=Path, help='optional output path')
    args = parser.parse_args(argv)
    try:
        rows = integrate_bias(args.bias, args.duration, args.dt)
        if args.csv:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            with args.csv.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(('time_s', 'velocity_error_m_s', 'position_error_m'))
                writer.writerows(rows)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    t, velocity, position = rows[-1]
    print(f't={t:.6f} s, velocity_error={velocity:.6f} m/s, position_error={position:.6f} m')
    print('world z-up; stationary truth; residual constant bias after gravity compensation')
    if args.csv:
        print(f'CSV: {args.csv}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
