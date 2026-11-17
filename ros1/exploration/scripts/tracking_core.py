# Reused pure geometry from the user's prior aliengo_navigation controller; ROS wrapper is new.
"""Planar pure-pursuit geometry. All values are SI; no ROS or robot SDK import."""
from bisect import bisect_right
from dataclasses import dataclass
import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x, y, z, w):
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Quaternion contains a nonfinite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-6 or abs(norm - 1.0) > 0.05:
        raise ValueError("Quaternion must be nonzero and normalized")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class Config:
    sample_spacing: float = 0.10
    lookahead: float = 0.35
    max_linear_speed: float = 0.15
    max_yaw_speed: float = 0.30
    max_linear_accel: float = 0.15
    max_yaw_accel: float = 0.45
    position_tolerance: float = 0.12
    yaw_tolerance: float = 0.15
    rotate_threshold: float = 0.60
    heading_gain: float = 0.8
    slowdown_distance: float = 0.60
    max_cross_track_error: float = 0.40
    projection_window: float = 0.60
    max_path_length: float = 100.0
    max_points: int = 20000

    def __post_init__(self):
        for key, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Invalid positive controller parameter: " + key)
        if self.max_linear_speed > 0.20 or self.max_yaw_speed > 0.30:
            raise ValueError("Follower limits may not exceed 0.20 m/s and 0.30 rad/s")
        if self.position_tolerance >= self.lookahead:
            raise ValueError("position_tolerance must be smaller than lookahead")


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float


class PathGeometry:
    def __init__(self, points, config):
        if not points or len(points) > config.max_points:
            raise ValueError("Path is empty or exceeds max_points")
        if any(not all(math.isfinite(v) for v in (p.x, p.y, p.z, p.yaw)) for p in points):
            raise ValueError("Path contains a nonfinite pose")
        self.goal = points[-1]
        self.points = [points[0]]
        length = 0.0
        for start, end in zip(points, points[1:]):
            segment = math.hypot(end.x - start.x, end.y - start.y)
            length += segment
            if length > config.max_path_length:
                raise ValueError("Path exceeds max_path_length")
            # Preserve vertices so a sharp corner never disappears during sampling.
            if segment < 1e-9:
                continue
            count = max(1, int(math.ceil(segment / config.sample_spacing)))
            if len(self.points) + count > config.max_points:
                raise ValueError("Resampled path exceeds max_points")
            for index in range(1, count + 1):
                alpha = index / count
                self.points.append(Waypoint(
                    start.x + alpha * (end.x - start.x),
                    start.y + alpha * (end.y - start.y),
                    start.z + alpha * (end.z - start.z),
                    wrap_angle(start.yaw + alpha * wrap_angle(end.yaw - start.yaw))))
        self.distances = [0.0]
        for start, end in zip(self.points, self.points[1:]):
            self.distances.append(self.distances[-1] + math.hypot(end.x - start.x, end.y - start.y))
        self.length = self.distances[-1]

    def at(self, distance):
        if len(self.points) == 1 or distance >= self.length:
            return self.goal
        distance = max(0.0, distance)
        index = min(bisect_right(self.distances, distance) - 1, len(self.points) - 2)
        start, end = self.points[index:index + 2]
        alpha = (distance - self.distances[index]) / (self.distances[index + 1] - self.distances[index])
        return Waypoint(start.x + alpha * (end.x - start.x),
                        start.y + alpha * (end.y - start.y),
                        start.z + alpha * (end.z - start.z),
                        wrap_angle(start.yaw + alpha * wrap_angle(end.yaw - start.yaw)))

    def project(self, x, y, lower, upper):
        """Find the closest point in a bounded forward interval; ties prefer less progress."""
        lower, upper = max(0.0, lower), min(self.length, upper)
        if len(self.points) == 1:
            return 0.0, math.hypot(x - self.goal.x, y - self.goal.y)
        if lower >= self.length:
            # There is no segment after the final vertex. Keep the real
            # endpoint distance so yaw alignment can finish without a false
            # infinite cross-track fault on the next control update.
            return self.length, math.hypot(x - self.goal.x, y - self.goal.y)
        first = max(0, bisect_right(self.distances, lower) - 1)
        best_distance, best_s = float("inf"), lower
        for index in range(first, len(self.points) - 1):
            segment_start = self.distances[index]
            if segment_start > upper + 1e-9:
                break
            start, end = self.points[index:index + 2]
            length = self.distances[index + 1] - segment_start
            alpha = ((x - start.x) * (end.x - start.x) + (y - start.y) * (end.y - start.y)) / (length * length)
            alpha = max(0.0, (lower - segment_start) / length,
                        min(alpha, 1.0, (upper - segment_start) / length))
            s = segment_start + alpha * length
            distance = math.hypot(x - start.x - alpha * (end.x - start.x),
                                  y - start.y - alpha * (end.y - start.y))
            if distance < best_distance - 1e-9 or (abs(distance - best_distance) <= 1e-9 and s < best_s):
                best_distance, best_s = distance, s
        return best_s, best_distance


class Controller:
    def __init__(self, config=None):
        self.config = config or Config()
        self.path = None
        self.progress = 0.0
        self.reached = False
        self.previous_velocity = (0.0, 0.0)

    def set_path(self, points):
        geometry = PathGeometry(points, self.config)
        self.path = geometry
        self.progress = 0.0
        self.reached = False
        self.stop()

    def cancel(self):
        self.path = None
        self.progress = 0.0
        self.reached = False
        self.stop()

    def stop(self):
        # Stops bypass acceleration limiting, including odometry/safety timeouts.
        self.previous_velocity = (0.0, 0.0)
        return (0.0, 0.0)

    def update(self, x, y, yaw, dt):
        if not all(math.isfinite(value) for value in (x, y, yaw, dt)) or dt <= 0:
            self.stop()
            raise ValueError("Nonfinite pose or invalid control timestep")
        if self.path is None:
            return *self.stop(), "waiting"
        if self.reached:
            return *self.stop(), "goal_reached"
        cfg, path = self.config, self.path
        candidate, error = path.project(x, y, self.progress, self.progress + cfg.projection_window)
        if error > cfg.max_cross_track_error:
            return *self.stop(), "fault"
        self.progress = max(self.progress, candidate)
        goal_distance = math.hypot(path.goal.x - x, path.goal.y - y)
        # Arc progress also matters on a closed path whose final pose is near the start.
        near_end = path.length - self.progress <= max(cfg.lookahead, cfg.position_tolerance)
        if goal_distance <= cfg.position_tolerance and near_end:
            heading = wrap_angle(path.goal.yaw - yaw)
            if abs(heading) <= cfg.yaw_tolerance:
                self.reached = True
                return *self.stop(), "goal_reached"
            linear, angular = 0.0, max(-cfg.max_yaw_speed, min(cfg.max_yaw_speed, cfg.heading_gain * heading))
        else:
            target = path.at(self.progress + cfg.lookahead)
            heading = wrap_angle(math.atan2(target.y - y, target.x - x) - yaw)
            angular = max(-cfg.max_yaw_speed, min(cfg.max_yaw_speed, cfg.heading_gain * heading))
            if abs(heading) > cfg.rotate_threshold:
                linear = 0.0
            else:
                remaining = max(goal_distance, path.length - self.progress)
                linear = cfg.max_linear_speed * max(0.0, math.cos(heading)) * min(1.0, remaining / cfg.slowdown_distance)
                curvature = 2.0 * math.sin(heading) / max(cfg.lookahead, math.hypot(target.x - x, target.y - y))
                angular = max(-cfg.max_yaw_speed, min(cfg.max_yaw_speed, linear * curvature))
        # A long scheduler gap cannot permit a large acceleration step.
        dt = min(dt, 0.1)
        previous_linear, previous_yaw = self.previous_velocity
        # Entering rotate-in-place must immediately remove translation.
        if linear > 0:
            linear = max(previous_linear - cfg.max_linear_accel * dt,
                         min(linear, previous_linear + cfg.max_linear_accel * dt))
        angular = max(previous_yaw - cfg.max_yaw_accel * dt,
                      min(angular, previous_yaw + cfg.max_yaw_accel * dt))
        self.previous_velocity = (linear, angular)
        return linear, angular, "tracking"
