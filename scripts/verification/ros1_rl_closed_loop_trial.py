#!/usr/bin/env python3
"""Isolated, bounded ROS 1 RL experiment. Never connects to a real robot."""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import time

import rosgraph
import rospy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from unitree_legged_msgs.msg import MotorCmd
import torch

from unio4_observation import DEFAULT_Q, JOINTS, ObservationHistory, finite_sequence
from unio4_preview import ADAPT_SHA, BODY_SHA, checked_file


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--calibrate', action='store_true')
    mode.add_argument('--baseline', action='store_true')
    args = parser.parse_args()
    out = Path('/result')
    report = {'result': 'FAIL', 'phase': 'startup', 'calibrate': args.calibrate, 'samples': []}
    state = {'clock': None, 'imu': None, 'joints': None, 'pose': None}
    controller = None
    terminal = None
    launch = None
    launch_log = (out / 'launch.log').open('w')
    controller_log = (out / 'controller.log').open('wb')
    try:
        torch.set_num_threads(1)
        adapt = torch.jit.load(checked_file('/course/.ros1_rl/cache/adapt.jit', ADAPT_SHA), map_location='cpu')
        body = torch.jit.load(checked_file('/course/.ros1_rl/cache/body.jit', BODY_SHA), map_location='cpu')
        adapt.eval()
        body.eval()
        launch = subprocess.Popen(['roslaunch', 'unitree_guide', 'gazeboSim.launch',
                                   'rname:=aliengo', 'gui:=false', 'paused:=false'],
                                  stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 90
        while not rosgraph.is_master_online():
            if launch.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('ROS master failed to start')
            time.sleep(.1)
        rospy.init_node('rl_calibration_trial', disable_signals=True)

        def clock(msg):
            state['clock'] = msg.clock.to_sec()

        def model(msg):
            if 'aliengo_gazebo' not in msg.name:
                return
            p = msg.pose[msg.name.index('aliengo_gazebo')]
            q = p.orientation
            state['pose'] = {'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                             'roll': math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
                             'pitch': math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))}

        rospy.Subscriber('/clock', Clock, clock, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, model, queue_size=1)
        rospy.Subscriber('/trunk_imu', Imu, lambda x: state.update(imu=x), queue_size=1)
        rospy.Subscriber('/aliengo_gazebo/joint_states', JointState,
                         lambda x: state.update(joints=x), queue_size=1)

        def wait_until(predicate, timeout, label):
            end = time.monotonic() + timeout
            while not predicate():
                if launch.poll() is not None or time.monotonic() >= end:
                    raise RuntimeError(label + ' timed out or simulation exited')
                time.sleep(.005)

        wait_until(lambda: all(state[k] is not None for k in state) and state['clock'] > 0,
                   90, 'initial feedback')
        terminal, slave = pty.openpty()
        controller = subprocess.Popen(['/work/devel/lib/unitree_guide/junior_ctrl'],
                                      stdin=slave, stdout=controller_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
        os.close(slave)

        def sim_wait(seconds, label):
            start = state['clock']
            wait_until(lambda: state['clock'] - start >= seconds, max(30, seconds * 15), label)

        sim_wait(1, 'controller initialization')
        os.write(terminal, b'2')
        sim_wait(8, 'Guide stand')
        if controller.poll() is not None:
            raise RuntimeError('Guide exited before standing')
        report['stand_pose'] = state['pose']
        os.killpg(controller.pid, signal.SIGINT)
        controller.wait(timeout=10)
        master = rosgraph.Master('/rl_calibration_trial')
        topics = ['/aliengo_gazebo/' + n.replace('_joint', '_controller') + '/command' for n in JOINTS]

        def no_motor_publishers():
            return not any(name in topics and nodes for name, nodes in master.getSystemState()[0])

        wait_until(no_motor_publishers, 5, 'Guide publisher shutdown')
        pubs = [rospy.Publisher(topic, MotorCmd, queue_size=1) for topic in topics]
        wait_until(lambda: all(p.get_num_connections() > 0 for p in pubs), 5, 'motor subscribers')

        def snapshot(max_age=.2):
            now, imu, js, pose = state['clock'], state['imu'], state['joints'], state['pose']
            if any(now - m.header.stamp.to_sec() > max_age or now - m.header.stamp.to_sec() < -.05
                   for m in (imu, js)):
                raise RuntimeError('stale/future sensor feedback')
            if not pose or not all(math.isfinite(v) for v in pose.values()):
                raise RuntimeError('non-finite pose')
            if pose['z'] < .25 or abs(pose['roll']) > .35 or abs(pose['pitch']) > .35:
                raise RuntimeError('pose/height safety threshold exceeded: ' + str(pose))
            names = list(js.name)
            if len(set(names)) != len(names) or len(js.position) != len(names) or len(js.velocity) != len(names):
                raise RuntimeError('invalid joint feedback')
            values = {n: {'q': js.position[i], 'dq': js.velocity[i]} for i, n in enumerate(names)}
            if any(n not in values for n in JOINTS):
                raise RuntimeError('missing joint')
            for n in JOINTS:
                finite_sequence([values[n]['q'], values[n]['dq']], 2)
            return now, imu, values, pose.copy()

        def publish(target):
            for i, pub in enumerate(pubs):
                msg = MotorCmd()
                msg.mode, msg.q, msg.dq, msg.Kp, msg.Kd, msg.tau = 10, target[i], 0, 20, .5, 0
                pub.publish(msg)

        if args.calibrate:
            report['phase'] = 'calibration'
            _, _, values, _ = snapshot()
            initial = [values[n]['q'] for n in JOINTS]
            report['calibration_initial_q'] = initial
            # Reference deployment adjusts each target by at most .05 rad per step.
            steps = max(1, math.ceil(max(abs(a-b) for a, b in zip(initial, DEFAULT_Q)) / .05))
            for step in range(1, steps + 1):
                snapshot()
                target = [a + min(1, step/steps)*(b-a) for a, b in zip(initial, DEFAULT_Q)]
                publish(target)
                sim_wait(.07, 'calibration step')
            report['calibrated_pose'] = snapshot()[3]
            report['calibrated_q'] = [snapshot()[2][n]['q'] for n in JOINTS]

        report['phase'] = 'policy_zero'
        obs = ObservationHistory()
        start = state['clock']
        previous_step = None
        deadline = time.monotonic() + 180
        while state['clock'] - start < 10:
            if time.monotonic() >= deadline or launch.poll() is not None:
                raise RuntimeError('policy phase timeout or simulator exited')
            now = state['clock']
            if previous_step is not None and now - previous_step < .02:
                time.sleep(.001)
                continue
            if previous_step is not None and (now <= previous_step or now - previous_step > .06):
                raise RuntimeError('policy cycle missed: %.6f s' % (now - previous_step))
            now, imu, values, pose = snapshot()
            q = imu.orientation
            history = obs.append([q.x, q.y, q.z, q.w], values, [0, 0, 0])
            with torch.no_grad():
                tensor = torch.tensor([history], dtype=torch.float32)
                latent = adapt(tensor)
                action = finite_sequence(body(torch.cat((tensor, latent), dim=-1)).reshape(-1).tolist(), 12)
            obs.accept_action(action)
            target = [q0 + .25*(.5 if i % 3 == 0 else 1)*a
                      for i, (q0, a) in enumerate(zip(DEFAULT_Q, obs.previous_action))]
            publish(target)
            report['samples'].append({'sim': now, 'dt': None if previous_step is None else now-previous_step,
                                      'pose': pose, 'q': [values[n]['q'] for n in JOINTS],
                                      'target': target, 'action': obs.previous_action[:]})
            previous_step = now
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error)
    finally:
        (out / 'summary.json').write_text(json.dumps(report, indent=2))
        for process in (controller, launch):
            if process and process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10)
                except (OSError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except OSError:
                        pass
        if terminal is not None:
            os.close(terminal)
        launch_log.close()
        controller_log.close()
    print('RL trial:', report['result'], report.get('error', ''), flush=True)
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
