"""Unit/frame conversion for the pinned Livox driver; no timestamp rewriting."""
from copy import deepcopy
import math
import numpy as np


def imu_si(message, scale):
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('invalid IMU acceleration scale')
    output = deepcopy(message)
    output.header.frame_id = 'mid360_imu_link'
    for axis in ('x', 'y', 'z'):
        value = getattr(message.linear_acceleration, axis)
        if not math.isfinite(value):
            raise ValueError('nonfinite IMU acceleration')
        setattr(output.linear_acceleration, axis, value*scale)
    if output.linear_acceleration_covariance[0] >= 0:
        output.linear_acceleration_covariance = [v*scale*scale for v in
                                                 message.linear_acceleration_covariance]
    return output


def xyz_returns(message):
    """Diagnostic raw cloud only. Navigation uses the deskewed LIO cloud."""
    if message.point_num != len(message.points):
        raise ValueError('CustomMsg point_num differs from points length')
    return [(p.x, p.y, p.z) for p in message.points
            if (p.tag & 0x30) in (0, 0x10)
            and all(math.isfinite(v) for v in (p.x, p.y, p.z))]


def near_returns(message, registration_blind):
    """Keep the original mapper's >=.2m near field outside the ICP input."""
    result = []
    for p in message.points:
        if (p.tag & 0x30) not in (0, 0x10):
            continue
        distance2 = p.x*p.x+p.y*p.y+p.z*p.z
        if .2**2 <= distance2 <= registration_blind**2:
            result.append((p.x, p.y, p.z, p.offset_time))
    return result


def deskew_near(points, start_ns, map_to_reference, pose_at_ns):
    """Use measured LIO TF at original point times, grouped at most 1ms apart.

    This supplements full LIO deskewed output, without changing its ICP blind
    distance. It neither invents points nor extrapolates an unavailable pose.
    """
    if not points:
        return np.empty((0, 3))
    values = np.asarray(points, dtype=np.float64)
    result = np.empty((len(values), 3))
    buckets = values[:, 3].astype(np.int64)//1000000
    for bucket in np.unique(buckets):
        selected = buckets == bucket
        stamp = start_ns+int(np.min(values[selected, 3]))
        transform = map_to_reference @ pose_at_ns(stamp)
        result[selected] = values[selected, :3] @ transform[:3, :3].T + transform[:3, 3]
    return result
