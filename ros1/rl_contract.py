"""Pure, configurable 48-value locomotion contract; no ROS or motor commands.

Input orientation is body-to-world xyzw; linear velocity is world-frame and
angular velocity is body-frame. Caller must enforce message age, synchronization,
control ownership, and target safety before applying the returned records.
"""
import math


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a finite number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return value


def _vector(values, size, label):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f'{label} must contain {size} values')
    return [_number(value, f'{label}[{i}]') for i, value in enumerate(values)]


def _positive(value, label):
    result = _number(value, label)
    if result <= 0:
        raise ValueError(f'{label} must be positive')
    return result


def _nonnegative_vector(values, size, label):
    result = _vector(values, size, label)
    if any(value < 0 for value in result):
        raise ValueError(f'{label} must be nonnegative')
    return result


def _cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])


def _inverse_rotate(vector, quaternion):
    # Rotate world vector using inverse of normalized body-to-world quaternion.
    x, y, z, w = quaternion
    axis = (-x, -y, -z)
    first = _cross(axis, vector)
    second = _cross(axis, first)
    return [_number(vector[i] + 2.0 * (w*first[i] + second[i]), 'rotated vector')
            for i in range(3)]


class PolicyContract:
    """Build observations and targets using explicit model-specific parameters.

    Calling targets commits the clipped action as previous_action. The caller
    should reset on control restart or rejected application of a target.
    """
    def __init__(self, config):
        if not isinstance(config, dict):
            raise ValueError('config must be a dict')
        names = config.get('joint_names')
        if (not isinstance(names, (list, tuple)) or len(names) != 12 or
                any(not isinstance(n, str) or not n for n in names) or len(set(names)) != 12):
            raise ValueError('joint_names must contain 12 unique names')
        self.names = tuple(names)
        self.default_q = _vector(config.get('default_q'), 12, 'default_q')
        self.kp = _nonnegative_vector(config.get('kp'), 12, 'kp')
        self.kd = _nonnegative_vector(config.get('kd'), 12, 'kd')
        self.multipliers = _nonnegative_vector(config.get('action_multipliers'), 12, 'action_multipliers')
        self.previous_multipliers = _nonnegative_vector(
            config.get('previous_action_multipliers'), 12, 'previous_action_multipliers')
        self.action_scale = _positive(config.get('action_scale'), 'action_scale')
        self.action_clip = _positive(config.get('action_clip'), 'action_clip')
        self.obs_clip = _positive(config.get('observation_clip'), 'observation_clip')
        scales = config.get('observation_scales')
        if not isinstance(scales, dict):
            raise ValueError('observation_scales must be a dict')
        self.scales = {key: _number(scales.get(key), 'observation_scales.' + key)
                       for key in ('lin_vel', 'ang_vel', 'gravity', 'dof_pos', 'dof_vel', 'previous_action')}
        self.scales['commands'] = _vector(scales.get('commands'), 3, 'observation_scales.commands')
        self.reset()

    def reset(self):
        self.previous_action = [0.0] * 12

    def observation(self, frame):
        if not isinstance(frame, dict):
            raise ValueError('frame must be a dict')
        quat = _vector(frame.get('quaternion_xyzw'), 4, 'quaternion_xyzw')
        norm = math.hypot(*quat)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError('quaternion norm must be finite and nonzero')
        quat = [v / norm for v in quat]
        lin = _inverse_rotate(_vector(frame.get('linear_velocity_world'), 3, 'linear_velocity_world'), quat)
        gravity = _inverse_rotate([0., 0., -1.], quat)
        angular = _vector(frame.get('angular_velocity_body'), 3, 'angular_velocity_body')
        command = _vector(frame.get('command'), 3, 'command')
        joints = frame.get('joints')
        if not isinstance(joints, dict):
            raise ValueError('joints must be a mapping')
        positions, velocities = [], []
        for i, name in enumerate(self.names):
            joint = joints.get(name)
            if not isinstance(joint, dict):
                raise ValueError(f'missing joint {name}')
            positions.append(_number(joint.get('q'), name + '.q') - self.default_q[i])
            velocities.append(_number(joint.get('dq'), name + '.dq'))
        s = self.scales
        obs = ([v*s['lin_vel'] for v in lin] + [v*s['ang_vel'] for v in angular] +
               [v*s['gravity'] for v in gravity] + [v*k for v, k in zip(command, s['commands'])] +
               [v*s['dof_pos'] for v in positions] + [v*s['dof_vel'] for v in velocities] +
               [v*k*s['previous_action'] for v, k in zip(self.previous_action, self.previous_multipliers)])
        # Reject nonfinite derived arithmetic before clipping.
        return [max(-self.obs_clip, min(self.obs_clip, _number(v, 'observation'))) for v in obs]

    def targets(self, actions):
        values = _vector(actions, 12, 'actions')
        used = [max(-self.action_clip, min(self.action_clip, v)) for v in values]
        records = []
        for i, name in enumerate(self.names):
            q = _number(self.default_q[i] + used[i] * self.action_scale * self.multipliers[i],
                        name + '.target')
            records.append({'name': name, 'q': q, 'dq': 0., 'kp': self.kp[i], 'kd': self.kd[i], 'tau': 0.})
        self.previous_action = used
        return records
