#!/usr/bin/env python3
"""固定官方 ROS1 版本，首次复制并应用本地仿真兼容补丁。"""
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = json.loads((ROOT / 'ros1/super_lio/source.json').read_text())
    reference = ROOT / source['reference_path']
    if not reference.exists():
        reference.mkdir(parents=True)
        subprocess.run(['git', '-C', str(reference), 'init'], check=True)
        subprocess.run(['git', '-C', str(reference), 'remote', 'add', 'origin',
                        source['repository']], check=True)
        subprocess.run(['git', '-C', str(reference), 'fetch', '--depth=1', 'origin',
                        source['commit']], check=True)
        subprocess.run(['git', '-C', str(reference), 'checkout', '--detach',
                        source['commit']], check=True)
    head = subprocess.check_output(['git', '-C', str(reference), 'rev-parse', 'HEAD'],
                                   text=True).strip()
    if head != source['commit']:
        raise RuntimeError('Super-LIO 参考版本与 source.json 不一致: ' + head)
    workspace = ROOT / source['workspace']
    if not (workspace / 'src').exists():
        shutil.copytree(reference / 'src', workspace / 'src')
        for name in [source['patch'], *source['additional_patches']]:
            with (ROOT / name).open('rb') as patch:
                subprocess.run(['patch', '--batch', '-p0', '-d', str(workspace / 'src')],
                               stdin=patch, check=True)
    (workspace / 'maps').mkdir(exist_ok=True)
    print('Super-LIO 工作区:', workspace)


if __name__ == '__main__':
    main()
