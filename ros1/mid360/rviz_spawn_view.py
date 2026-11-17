#!/usr/bin/env python3
"""Launch RViz with a fixed world view centered once at the Gazebo spawn."""
import copy
import os
import sys
import tempfile

import yaml


def render_config(template, spawn):
    # 深拷贝模板；焦点与参考网格固定在出生点，不随机器人移动。
    config = copy.deepcopy(template)
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = 'world'
    current = manager['Views']['Current']
    current['Class'] = 'rviz/Orbit'
    current['Target Frame'] = '<Fixed Frame>'
    current['Focal Point'] = dict(zip('XYZ', spawn))
    x, y, z = spawn
    for display in manager['Displays']:
        if display['Class'] == 'rviz/Grid':
            display['Reference Frame'] = '<Fixed Frame>'
            display['Offset'] = {'X': x, 'Y': y, 'Z': z - .6}
    return config


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    spawn = tuple(map(float, args[:3]))
    command = args[3:]
    config_index = command.index('-d') + 1
    with open(command[config_index], encoding='utf-8') as source:
        config = render_config(yaml.safe_load(source), spawn)
    # exec 后 RViz 才读取配置；临时文件随仿真容器退出清理。
    with tempfile.NamedTemporaryFile(mode='w', prefix='aliengo-rviz-', suffix='.rviz',
                                     encoding='utf-8', delete=False) as generated:
        yaml.safe_dump(config, generated, allow_unicode=True, sort_keys=False)
    command[config_index] = generated.name
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
