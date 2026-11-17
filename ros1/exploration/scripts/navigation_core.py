"""ROS-independent exploration lifecycle; stale state never resumes cancelled motion."""
import math

from tracking_core import Config, Controller, Waypoint, wrap_angle, quaternion_yaw


class Freshness:
    def __init__(self):
        self.stamp = None
        self.wall = None

    def observe(self, stamp, wall):
        if not all(math.isfinite(v) for v in (stamp, wall)) or stamp <= 0:
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.stamp, self.wall = stamp, wall
        return True

    def fresh(self, now, wall, ros_age=1.5, wall_gap=1.5):
        return (self.stamp is not None and math.isfinite(now) and math.isfinite(wall)
                and -.05 <= now - self.stamp <= ros_age
                and 0 <= wall - self.wall <= wall_gap)


def trim_path(points, horizon):
    """Retain vertices and interpolate the final pose at a short execution horizon."""
    if not points or len(points) > 2000:
        raise ValueError('Path is empty or oversized')
    if any(not all(math.isfinite(v) for v in (p.x, p.y, p.z, p.yaw)) for p in points):
        raise ValueError('Nonfinite path')
    result, length = [points[0]], 0.
    for first, last in zip(points, points[1:]):
        distance = math.hypot(last.x-first.x, last.y-first.y)
        if length + distance > horizon:
            ratio = (horizon-length)/distance
            # A prefix endpoint is not the planner goal: finish along the
            # segment instead of inventing a terminal turn to its stored yaw.
            result.append(Waypoint(first.x+ratio*(last.x-first.x),
                                   first.y+ratio*(last.y-first.y),
                                   first.z+ratio*(last.z-first.z),
                                   math.atan2(last.y-first.y, last.x-first.x)))
            break
        result.append(last)
        length += distance
    extent = sum(math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(result, result[1:]))
    if extent < .08 and abs(wrap_angle(result[-1].yaw-result[0].yaw)) < .2:
        raise ValueError('Planner path makes no progress')
    return result


class Session:
    def __init__(self, horizon=.6, config=None):
        self.controller = Controller(config or Config(
            sample_spacing=.05, lookahead=.25, max_linear_speed=.10,
            position_tolerance=.07, yaw_tolerance=.15,
            rotate_threshold=.45, max_cross_track_error=.30,
            slowdown_distance=.20))
        self.horizon = horizon
        self.enabled = False
        self.state = 'disabled'
        self.reason = ''
        self.generation = 0
        self.completed = 0
        self.path_ros = self.path_wall = self.last_ros = None
        self.last_tick_wall = None

    def enable(self):
        self.generation += 1
        self.controller.cancel()
        self.enabled, self.state, self.reason = True, 'ready', ''
        self.path_ros = self.path_wall = self.last_ros = None

    def cancel(self, reason='operator stop'):
        self.generation += 1
        self.controller.cancel()
        self.enabled, self.state, self.reason = False, 'stopped', reason
        self.path_ros = self.path_wall = self.last_ros = None

    def begin_plan(self):
        if not self.enabled or self.state not in ('ready', 'goal_reached'):
            return None
        self.generation += 1
        self.state = 'planning'
        return self.generation

    def accept_path(self, token, points, now, wall, frame='lio_map'):
        if not self.enabled or self.state != 'planning' or token != self.generation:
            return False
        try:
            if frame != 'lio_map':
                raise ValueError('Wrong path frame')
            if not all(math.isfinite(v) for v in (now, wall)):
                raise ValueError('Invalid path time')
            self.controller.set_path(trim_path(points, self.horizon))
        except ValueError:
            self.cancel('invalid planner path')
            raise
        self.path_ros, self.path_wall, self.last_ros = now, wall, now
        self.state, self.reason = 'tracking', ''
        return True

    def tick(self, pose, now, wall, healthy=True, reason='input stopped'):
        previous_wall = self.last_tick_wall
        self.last_tick_wall = wall
        if not self.enabled:
            return 0., 0., self.state
        # Command lifecycle only: a stalled publisher must not resume its old
        # generation, even when perception has caught up before this tick.
        if (not math.isfinite(wall) or (previous_wall is not None
                and (wall < previous_wall or wall-previous_wall > .4))):
            self.cancel('command publisher wall gap')
            return 0., 0., self.state
        if not healthy:
            self.cancel(reason)
            return 0., 0., self.state
        if self.state != 'tracking':
            return 0., 0., self.state
        if (not all(math.isfinite(v) for v in (*pose, now, wall))
                or now < self.last_ros or now-self.path_ros > 60
                or wall < self.path_wall or wall-self.path_wall > 120):
            self.cancel('active path time invalid or expired')
            return 0., 0., self.state
        dt = now-self.last_ros
        if dt <= 0:
            return *self.controller.previous_velocity, self.state
        self.last_ros = now
        linear, angular, status = self.controller.update(*pose, dt)
        if status == 'fault':
            self.cancel('cross-track error')
            return 0., 0., self.state
        if status == 'goal_reached':
            self.completed += 1
            self.state = 'goal_reached'
        return linear, angular, self.state
