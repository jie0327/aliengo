#!/usr/bin/env python3
"""Validate measured hardware inputs and derive only the planner sensor pose."""
import argparse
import math
from pathlib import Path
import yaml


def load_config(path):
    value = yaml.safe_load(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError('robot.yaml must be a mapping')
    for mount in ('base_to_mid360', 'base_to_d435'):
        for name in ('xyz', 'rpy'):
            vector = value.get(mount, {}).get(name)
            if (not isinstance(vector, list) or len(vector) != 3 or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                        not math.isfinite(v) for v in vector)):
                raise ValueError(mount+'.'+name+' 需要填写实测的三个有限数值')
    scale = value.get('imu_acceleration_scale')
    if isinstance(scale, bool) or not isinstance(scale, (int, float)) or not math.isfinite(scale) or scale <= 0:
        raise ValueError('imu_acceleration_scale must be positive')
    return value


def planner_sensor(value):
    mount = value['base_to_mid360']
    roll, pitch, yaw = mount['rpy']
    sensor = {'center_offset': mount['xyz'], 'rotations': [yaw, pitch, roll]}
    return {'SensorParams': {'MID360': sensor}, 'CameraAnnotationParams': {'MID360': sensor}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config')
    parser.add_argument('--sensor-output')
    parser.add_argument('--runtime-output')
    args = parser.parse_args()
    try:
        config = load_config(args.config)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.exit(1, str(error)+'\n')
    if args.sensor_output:
        Path(args.sensor_output).write_text(yaml.safe_dump(planner_sensor(config), sort_keys=False))
    if args.runtime_output:
        # rosparam/XMLRPC cannot upload None. Unknown metadata stays in robot.yaml.
        Path(args.runtime_output).write_text(yaml.safe_dump(
            {k: v for k, v in config.items() if v is not None}, sort_keys=False))
    print('外参格式有效；数值正确性仍需用实物与点云确认。')
