#!/usr/bin/env python3
"""ROS-independent measured-floor mapping for the static, flat Depot trial.

All points passed to ObservedTerrain are real surface returns in lio_map.
The stance-height prior selects a measured height cluster; it never adds cells.
A stored cell says that surface was observed, not that it is currently visible.
"""
from collections import deque
import math

import numpy as np

SOURCES = ('lidar', 'camera')


def _points(points):
    try:
        points = np.asarray(points, dtype=float)
    except (TypeError, ValueError):
        return np.empty((0, 3))
    if points.ndim != 2 or points.shape[1] != 3:
        return np.empty((0, 3))
    return points[np.all(np.isfinite(points), axis=1)]


def transform_points(points, transform):
    """Apply T_target_source without modifying the native points."""
    points = _points(points)
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError('invalid rigid transform')
    return points @ transform[:3, :3].T + transform[:3, 3]


def filter_native_points(points, base_to_sensor, min_range=.15, max_range=20., voxel=.04):
    """Range/voxel/body filter; output coordinates remain in the sensor frame."""
    points = _points(points)
    ranges = np.linalg.norm(points, axis=1)
    points = points[(ranges >= min_range) & (ranges <= max_range)]
    base = transform_points(points, base_to_sensor)
    # Conservative trunk envelope only: do not remove the floor as robot body.
    body = ((np.abs(base[:, 0]) <= .43) & (np.abs(base[:, 1]) <= .26)
            & (base[:, 2] >= -.24) & (base[:, 2] <= .22))
    points = points[~body]
    if voxel > 0. and len(points):
        _, first = np.unique(np.floor(points / voxel).astype(np.int64),
                             axis=0, return_index=True)
        points = points[np.sort(first)]
    return points


def imu_to_base_pose(map_to_imu, base_to_imu):
    """T_map_base = T_map_IMU inverse(T_base_IMU)."""
    return np.asarray(map_to_imu) @ np.linalg.inv(np.asarray(base_to_imu))


def imu_to_base_twist(linear, angular, base_to_imu):
    """Twist at IMU origin in IMU axes -> twist at base origin in base axes."""
    mount = np.asarray(base_to_imu)
    angular = mount[:3, :3] @ np.asarray(angular)
    linear = mount[:3, :3] @ np.asarray(linear) - np.cross(angular, mount[:3, 3])
    return linear, angular


class GyroBuffer:
    """Bounded IMU-frame gyro samples matched by acquisition time, never receipt."""
    def __init__(self, frame, max_age=.025, capacity=400):
        self.frame = frame
        self.max_age = float(max_age)
        self.samples = deque(maxlen=max(1, int(capacity)))

    def add(self, stamp, frame, angular, covariance):
        angular = np.asarray(angular, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        if (frame.lstrip('/') != self.frame or not math.isfinite(stamp) or stamp <= 0
                or angular.shape != (3,) or covariance.shape != (3, 3)
                or not np.all(np.isfinite(angular)) or not np.all(np.isfinite(covariance))
                or np.any(np.diag(covariance) < 0)):
            return False
        self.samples.append((stamp, angular.copy(), covariance.copy()))
        return True

    def nearest(self, stamp):
        if not self.samples or not math.isfinite(stamp):
            return None
        sample = min(self.samples, key=lambda item: abs(item[0]-stamp))
        if abs(sample[0]-stamp) > self.max_age:
            return None
        return sample[1].copy(), sample[2].copy()


class TransformQueue:
    """Bounded acquisition-time queues, independent for each physical source.

    lookup(source, original_stamp) returns a transform bundle or None. A missing
    transform holds that source's head while other sources continue. max_age is
    monotonic wall time, so a paused ROS clock cannot grow the queues forever.
    """
    def __init__(self, max_per_source=8, max_age=1.):
        self.limit = max(1, int(max_per_source))
        self.max_age = float(max_age)
        self.queues = {source: deque() for source in SOURCES}
        self.dropped = {source: 0 for source in SOURCES}

    def enqueue(self, source, stamp, payload, wall):
        if source not in self.queues or not math.isfinite(stamp) or stamp <= 0:
            return False
        queue = self.queues[source]
        if any(item[0] == stamp for item in queue):
            return False
        queue.append((stamp, payload, wall))
        # Sort only within a physical source. Never compare cameras to lidars.
        self.queues[source] = deque(sorted(queue, key=lambda item: item[0]))
        if len(self.queues[source]) > self.limit:
            self.queues[source].popleft()
            self.dropped[source] += 1
        return True

    def pending(self, source):
        return len(self.queues[source])

    def ready(self, lookup, wall):
        ready = []
        for source in SOURCES:
            queue = self.queues[source]
            while queue:
                stamp, payload, arrived = queue[0]
                if wall - arrived > self.max_age:
                    queue.popleft()
                    self.dropped[source] += 1
                    continue
                transform = lookup(source, stamp)
                if transform is None:
                    break
                queue.popleft()
                ready.append((source, stamp, payload, transform))
        return ready


class ObservedTerrain:
    """Fixed world grid with measured support and persistent hazard evidence.

    pose arguments are (x,y,z,yaw) of base in lio_map. A footprint checks every
    cell touched by the rotated body's AABB (a conservative superset). Stance
    z must agree with the measured flat floor; this is not a stair gait model.
    """
    def __init__(self, resolution=.1, length=16., center=(0., 0.),
                 nominal_base_height=.43, ground_tolerance=.035,
                 preinitialization_limit=100000):
        if resolution <= 0 or length < resolution:
            raise ValueError('invalid grid dimensions')
        self.resolution = float(resolution)
        self.size = int(round(length / resolution))
        # Align the fixed world cells to resolution, including with odd sizes.
        minimum = np.floor((np.asarray(center) - self.size * resolution / 2) / resolution)
        self.minimum = minimum.astype(np.int64)
        self.center = (self.minimum + self.size / 2.) * resolution
        self.nominal_base_height = float(nominal_base_height)
        self.tolerance = float(ground_tolerance)
        self.floor_z = None
        self.values = np.full((self.size, self.size), np.nan, dtype=np.float32)
        self.blocked = np.zeros_like(self.values, dtype=bool)
        self.sources = {source: {'stamp': 0., 'count': 0} for source in SOURCES}
        self.preinitialization_limit = max(0, int(preinitialization_limit))
        self.unclassified = []
        self.unclassified_count = 0

    def _indices(self, xy):
        # Matrix index zero is at the positive-x, positive-y corner.
        absolute = np.floor(np.asarray(xy) / self.resolution + 1.e-10).astype(np.int64)
        return self.size - 1 - (absolute - self.minimum)

    def _inside(self, index):
        return bool(np.all(index >= 0) and np.all(index < self.size))

    def _find_floor(self, points, base_z):
        prior = base_z - self.nominal_base_height
        candidates = points[np.abs(points[:, 2] - prior) < .04]
        if len(candidates) < 12:
            return None
        # A supported narrow height cluster, spread over actual XY hit cells.
        bins = np.floor((candidates[:, 2] - prior) / .02).astype(int)
        labels, counts = np.unique(bins, return_counts=True)
        best = labels[np.argmax(counts)]
        peak = float(np.median(candidates[bins == best, 2]))
        inliers = candidates[np.abs(candidates[:, 2] - peak) <= .025]
        if len(inliers) < 12 or len(np.unique(self._indices(inliers[:, :2]), axis=0)) < 3:
            return None
        # Verify the selected cluster is a horizontal plane, not a vertical edge.
        design = np.column_stack((inliers[:, :2], np.ones(len(inliers))))
        plane, _, rank, _ = np.linalg.lstsq(design, inliers[:, 2], rcond=None)
        if rank < 3 or np.linalg.norm(plane[:2]) > .06:
            return None
        if np.quantile(np.abs(design @ plane - inliers[:, 2]), .9) > .02:
            return None
        return float(np.median(inliers[:, 2]))

    def update(self, source, stamp, map_points, base_pose):
        points = _points(map_points)
        try:
            pose = np.asarray(base_pose, dtype=float)
            valid = (pose.shape == (4,) and np.all(np.isfinite(pose))
                     and math.isfinite(stamp) and stamp > 0)
        except (TypeError, ValueError):
            return False
        if source not in self.sources or not valid or not len(points):
            return False
        if stamp <= self.sources[source]['stamp']:
            return False
        self.sources[source] = {'stamp': float(stamp), 'count': self.sources[source]['count'] + 1}
        if self.floor_z is None:
            self.floor_z = self._find_floor(points, pose[2])
        if self.floor_z is None:
            indices = self._indices(points[:, :2])
            inside = np.all((indices >= 0) & (indices < self.size), axis=1)
            points, indices = points[inside], indices[inside]
            keep = min(len(points), self.preinitialization_limit-self.unclassified_count)
            if keep:
                self.unclassified.append(points[:keep].copy())
                self.unclassified_count += keep
            # Losing unclassified evidence must fail closed, even after later
            # ground hits. This bounds memory without silently forgetting hazards.
            overflow = indices[keep:]
            self.blocked[overflow[:, 0], overflow[:, 1]] = True
            return True
        if self.unclassified:
            points = np.concatenate((*self.unclassified, points))
            self.unclassified.clear()
            self.unclassified_count = 0
        indices = self._indices(points[:, :2])
        inside = np.all((indices >= 0) & (indices < self.size), axis=1)
        indices, points = indices[inside], points[inside]
        if not len(points):
            return True
        delta = points[:, 2] - self.floor_z
        ground = np.abs(delta) <= self.tolerance
        support = indices[ground]
        heights = points[ground, 2]
        unblocked = ~self.blocked[support[:, 0], support[:, 1]]
        support = support[unblocked]
        self.values[support[:, 0], support[:, 1]] = heights[unblocked]
        # Any return below the floor or in the low/body obstacle band is unsafe.
        hazard = ((delta < -self.tolerance) | ((delta > self.tolerance) & (delta < 1.)))
        unsafe = indices[hazard]
        self.blocked[unsafe[:, 0], unsafe[:, 1]] = True
        self.values[unsafe[:, 0], unsafe[:, 1]] = np.nan
        return True

    def cell_elevation(self, x, y):
        index = self._indices((x, y))
        return float(self.values[tuple(index)]) if self._inside(index) else float('nan')

    def grid(self):
        return self.values.copy(), tuple(self.center)

    def footprint_known(self, pose):
        try:
            pose = np.asarray(pose, dtype=float)
        except (TypeError, ValueError):
            return False, 'invalid base pose'
        if pose.shape != (4,) or not np.all(np.isfinite(pose)):
            return False, 'invalid base pose'
        if self.floor_z is None:
            return False, 'floor not observed'
        if abs(pose[2] - self.floor_z - self.nominal_base_height) > .12:
            return False, 'base height disagrees with flat floor'
        x, y, _, yaw = pose
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        dx, dy = .375*c+.225*s, .375*s+.225*c
        lower = np.floor((np.array([x-dx, y-dy]) - 1.e-10) / self.resolution).astype(int)
        upper = np.floor((np.array([x+dx, y+dy]) + 1.e-10) / self.resolution).astype(int)
        for ax in range(lower[0], upper[0]+1):
            for ay in range(lower[1], upper[1]+1):
                index = self.size - 1 - (np.array([ax, ay]) - self.minimum)
                if not self._inside(index):
                    return False, 'footprint outside observed grid'
                if self.blocked[tuple(index)]:
                    return False, 'obstacle or hole in footprint'
                if not np.isfinite(self.values[tuple(index)]):
                    return False, 'unknown footprint cell'
        return True, 'observed flat support'

    def validate_path(self, poses, spacing=.05):
        try:
            poses = np.asarray(poses, dtype=float)
        except (TypeError, ValueError):
            return False, 'invalid path', 0
        if poses.ndim != 2 or poses.shape[1] != 4 or not 0 < len(poses) <= 1000 or not np.all(np.isfinite(poses)):
            return False, 'invalid path', 0
        if not .01 <= spacing <= .1:
            return False, 'invalid interpolation spacing', 0
        for i, pose in enumerate(poses):
            previous = poses[max(i-1, 0)]
            distance = np.linalg.norm(pose[:3] - previous[:3])
            yaw_delta = math.atan2(math.sin(pose[3]-previous[3]), math.cos(pose[3]-previous[3]))
            steps = max(1, int(math.ceil(max(distance, abs(yaw_delta)*.45) / spacing)))
            if steps > 10000:
                return False, 'path segment too long', i
            for ratio in np.linspace(0., 1., steps+1):
                sample = previous + ratio*(pose-previous)
                sample[3] = previous[3] + ratio*yaw_delta
                valid, reason = self.footprint_known(sample)
                if not valid:
                    return False, reason, i
        return True, 'observed flat support', -1

    def status(self, stamp, base_pose):
        known, reason = self.footprint_known(base_pose)
        return {'stamp': float(stamp), 'frame_id': 'lio_map', 'floor_z': self.floor_z,
                'cells': int(np.count_nonzero(np.isfinite(self.values))),
                'footprint_known': known, 'reason': reason,
                'sources': {name: dict(state) for name, state in self.sources.items()}}

    def ground_points(self):
        indices = np.argwhere(np.isfinite(self.values))
        xy = (self.minimum + self.size - indices - .5) * self.resolution
        return np.column_stack((xy, self.values[indices[:, 0], indices[:, 1]]))


def unpack_xyz(data, width, height, point_step, row_step, fields, big_endian=False):
    """Decode PointCloud2 XYZ, including organized row padding, without ROS.

    fields are (name,byte_offset,PointField datatype,count). Only scalar floating
    XYZ are accepted. Invalid layouts raise rather than inventing observations.
    """
    if (width <= 0 or height <= 0 or width*height > 4000000 or point_step <= 0
            or row_step < width*point_step or len(data) < height*row_step):
        raise ValueError('invalid PointCloud2 buffer dimensions')
    description = {name: (offset, datatype, count) for name, offset, datatype, count in fields}
    columns = []
    for name in ('x', 'y', 'z'):
        if name not in description:
            raise ValueError('PointCloud2 missing XYZ')
        offset, datatype, count = description[name]
        if datatype not in (7, 8) or count != 1:
            raise ValueError('PointCloud2 XYZ must be scalar floats')
        dtype = np.dtype(('>' if big_endian else '<') + ('f4' if datatype == 7 else 'f8'))
        if offset < 0 or offset + dtype.itemsize > point_step:
            raise ValueError('PointCloud2 field outside point')
        column = np.ndarray((height, width), dtype=dtype, buffer=data,
                            offset=offset, strides=(row_step, point_step))
        columns.append(column.reshape(-1))
    return _points(np.column_stack(columns))
