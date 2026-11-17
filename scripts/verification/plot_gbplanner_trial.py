#!/usr/bin/env python3
"""Plot evaluation-only Gazebo samples; never publish or feed navigation."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    args = parser.parse_args()
    summary = json.loads((args.trial/'summary.json').read_text())
    samples = json.loads((args.trial/'samples.json').read_text())
    rows = [r for r in samples['truth'] if r['phase'] not in ('startup', 'stand')]
    if not rows:
        parser.error('trial has no post-stand evaluation samples')
    origin = np.array(rows[0]['position'])
    fig, (route, height) = plt.subplots(1, 2, figsize=(10, 3.7), constrained_layout=True)
    groups = [(['warmup_trot', 'warmup_walk', 'warmup_stop', 'warmup_stand'],
               'Manual initialization', '#6b7280'),
              (['autonomy'], 'Autonomous execution', '#16a34a'),
              (['command_interruption', 'paused_clock', 'resumed_clock', 'operator_stop'],
               'Interruption / stop checks', '#2563eb')]
    for phases, label, color in groups:
        selected = [r for r in rows if r['phase'] in phases]
        if not selected:
            continue
        xyz = np.array([r['position'] for r in selected])
        route.plot(xyz[:, 0]-origin[0], xyz[:, 1]-origin[1], color=color, lw=2, label=label)
        height.plot([r['stamp'] for r in selected], xyz[:, 2], color=color, lw=1)
    route.scatter(0, 0, c='black', s=22, zorder=3)
    route.set(xlabel='World X from warmup start (m)', ylabel='World Y from warmup start (m)',
              title='Measured robot motion', aspect='equal', ylim=(-.6, .6))
    route.legend(fontsize=8, loc='upper left')
    height.set(xlabel='Simulation time (s)', ylabel='Base height above world ground (m)',
               title='Posture during execution', ylim=(.40, .45))
    for axis in (route, height):
        axis.grid(alpha=.25)
    manual = summary.get('warmup', {}).get('distance_m', 0.)
    autonomous = summary.get('autonomy', {}).get('displacement_m', 0.)
    fig.suptitle('%s | manual %.2f m | autonomous %.2f m\nGazebo poses used for evaluation only'
                 % (summary['result'], manual, autonomous), fontsize=11)
    for extension in ('png', 'svg'):
        fig.savefig(args.trial/('evaluation-trajectory.'+extension), dpi=160)
    print(args.trial/'evaluation-trajectory.png')


if __name__ == '__main__':
    main()
