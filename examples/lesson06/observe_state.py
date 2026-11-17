#!/usr/bin/env python3
"""Bounded read-only ROS 1 IMU/joint observer; maths remain importable without ROS."""
import argparse
import json
import math
import time


def stamp_seconds(sec, nanosec):
    if not isinstance(sec, int) or not isinstance(nanosec, int) or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('ROS stamp requires integer seconds and nanoseconds in [0, 1e9)')
    return sec + nanosec * 1e-9


def finite_vector(values, length=None):
    values = tuple(float(value) for value in values)
    if (length is not None and len(values) != length) or not all(math.isfinite(v) for v in values):
        raise ValueError('state contains non-finite values or inconsistent dimensions')
    return values


def imu_summary(quaternion, gyro, acceleration, orientation_available=True,
                gyro_available=True, acceleration_available=True):
    result = {'quaternion_norm': None, 'gyro_norm_rad_s': None, 'specific_force_norm_m_s2': None}
    if orientation_available:
        norm = math.hypot(*finite_vector(quaternion, 4))
        if abs(norm - 1.0) > 0.1:
            raise ValueError('available IMU orientation is not approximately a unit quaternion')
        result['quaternion_norm'] = norm
        result['quaternion_xyzw'] = tuple(quaternion)
    if gyro_available:
        result['gyro_rad_s'] = finite_vector(gyro, 3)
        result['gyro_norm_rad_s'] = math.hypot(*result['gyro_rad_s'])
    if acceleration_available:
        result['specific_force_m_s2'] = finite_vector(acceleration, 3)
        result['specific_force_norm_m_s2'] = math.hypot(*result['specific_force_m_s2'])
    return result


def joint_summary(names, positions):
    if not names or len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('joint names must be unique and have one position per name')
    positions = finite_vector(positions)
    return {'joint_count': len(names), 'position_rad': dict(zip(names, positions))}


class SampleTimes:
    def __init__(self):
        self.previous = {}

    def accept(self, stream, stamp):
        """Return None for first stamp, 0 for duplicate, positive dt for a new sample."""
        stamp, = finite_vector((stamp,))
        previous = self.previous.get(stream)
        if previous is not None and stamp < previous:
            raise ValueError(f'{stream} stamp moved backwards; restart observation after clock reset')
        if previous is not None and stamp == previous:
            return 0.0
        self.previous[stream] = stamp
        return None if previous is None else stamp - previous


class ObserverState:
    """Wait for each stream to become valid; keep one fixed wall deadline."""

    def __init__(self, samples, timeout, start_time):
        self.samples = samples
        self.deadline = start_time + timeout
        self.counts = {'imu': 0, 'joints': 0}
        self.summaries = {}
        self.errors = []
        self.duplicate_stamps = {stream: 0 for stream in self.counts}
        self.startup_invalid = {stream: {'count': 0, 'last_reason': None} for stream in self.counts}
        self.timing = SampleTimes()

    def remaining(self, now):
        return max(0.0, self.deadline - now)

    def finished(self, now):
        return bool(self.errors) or min(self.counts.values()) >= self.samples or self.remaining(now) <= 0

    def consume(self, stream, read_sample, now):
        if self.finished(now):
            return
        try:
            summary = dict(read_sample())
            delta = self.timing.accept(stream, summary['stamp_s'])
            if delta == 0.0:
                self.duplicate_stamps[stream] += 1
                return
            summary['last_dt_s'] = delta
            self.summaries[stream] = summary
            self.counts[stream] = min(self.samples, self.counts[stream] + 1)
        except ValueError as error:
            if self.counts[stream] == 0:
                self.startup_invalid[stream]['count'] += 1
                self.startup_invalid[stream]['last_reason'] = str(error)
            else:
                self.errors.append(f'{stream}: {error}')

    def report(self, now):
        complete = min(self.counts.values()) >= self.samples and not self.errors
        return {'complete': complete, 'timed_out': not complete and self.remaining(now) <= 0,
                'counts': dict(self.counts), 'last_samples': dict(self.summaries),
                'duplicate_stamps': dict(self.duplicate_stamps),
                'startup_invalid': {stream: dict(value) for stream, value in self.startup_invalid.items()},
                'errors': list(self.errors)}


def main(argv=None):
    # ROS 1 subscriber shares the pure validation and wall-time deadline above.
    from ros1_observe_state import main as ros1_main
    return ros1_main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
