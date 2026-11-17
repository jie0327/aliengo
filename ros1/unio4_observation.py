"""Pure observation adapter for the fixed UniO4 AlienGo checkpoint.

The output is suitable only for the associated policy, not a generic RL layout.
No ROS messages or actuator commands are produced here.
"""
import math

JOINTS = tuple(f'{leg}_{joint}_joint' for leg in ('FL', 'FR', 'RL', 'RR')
               for joint in ('hip', 'thigh', 'calf'))
DEFAULT_Q = (0.1, 0.8, -1.5, -0.1, 0.8, -1.5,
             0.1, 1.0, -1.5, -0.1, 1.0, -1.5)
COMMAND_SCALE = (2.0, 2.0, 0.25, 2.0, 1.0, 1.0, 1.0, 1.0,
                 1.0, 0.15, 0.3, 0.3, 1.0, 1.0, 1.0)
DT = 0.02


def finite_sequence(values, length):
    values = [float(x) for x in values]
    if len(values) != length or not all(math.isfinite(x) for x in values):
        raise ValueError('数据维度错误或包含非有限数')
    return values


class ObservationHistory:
    def __init__(self):
        self.reset()

    def reset(self):
        self.history = [0.0] * 290
        self.previous_action = [0.0] * 12
        self.gait = 0.0
        self.clock = [0.0] * 4

    def append(self, quaternion_xyzw, joints, command):
        q = finite_sequence(quaternion_xyzw, 4)
        norm = math.sqrt(sum(x*x for x in q))
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError('无效姿态四元数')
        x, y, z, w = [v / norm for v in q]
        gravity = [-2*(x*z-y*w), -2*(y*z+x*w), -(1-2*(x*x+y*y))]
        speed = finite_sequence(command, 3)
        cmd = speed + [0, 3, .5, 0, 0, .5, .08, 0, 0, .33, .4, 0]
        cmd = [a*b for a, b in zip(cmd, COMMAND_SCALE)]
        try:
            positions = finite_sequence([joints[n]['q'] for n in JOINTS], 12)
            velocities = finite_sequence([joints[n]['dq'] for n in JOINTS], 12)
        except (KeyError, TypeError) as error:
            raise ValueError('关节反馈不完整') from error
        frame = (gravity + cmd + [a-b for a, b in zip(positions, DEFAULT_Q)] +
                 [v*.05 for v in velocities] + self.previous_action + self.clock)
        frame = [max(-100.0, min(100.0, v)) for v in frame]
        self.history = self.history[58:] + frame
        self.gait = (self.gait + DT * 3.0) % 1.0
        self.clock = [math.sin(2*math.pi*(self.gait+offset))
                      for offset in (.5, 0.0, 0.0, .5)]
        return list(self.history)

    def accept_action(self, action):
        self.previous_action = [max(-10.0, min(10.0, a))
                                for a in finite_sequence(action, 12)]
