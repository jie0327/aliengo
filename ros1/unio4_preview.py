#!/usr/bin/env python3
"""Run a UniO4 policy on ROS 1 sensor feedback without controlling actuators."""
import argparse
import hashlib
import math
from pathlib import Path
import sys
import threading
import time

from unio4_observation import ObservationHistory, JOINTS, finite_sequence

ADAPT_SHA = 'ad8f32345fe04f34210a7c0a762b46c8231565408bee881f665057cf4a60c76b'
BODY_SHA = '31884a6295076bcd846ea38af6c864cccb5c4cec8dda2b5ccf5da318ae32961d'


def checked_file(path, expected):
    path = Path(path)
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        raise ValueError('模型 SHA256 不匹配: ' + str(path))
    return str(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vx', type=float, default=0.0, help='推理用速度意图；不控制机器人')
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--samples', type=int, default=0, help='0 持续运行；正数为最多输出条数')
    parser.add_argument('--timeout', type=float, default=30, help='等待首次有效反馈的墙钟秒数')
    parser.add_argument('--model-dir', default='/course/.ros1_rl/cache')
    args = parser.parse_args()
    command = finite_sequence([args.vx, args.vy, args.yaw], 3)
    if args.samples < 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('samples 不可为负；timeout 必须大于零')
    if abs(args.vx) > 1 or abs(args.vy) > 1 or abs(args.yaw) > 1:
        parser.error('预览意图范围限于 [-1, 1]')

    import rospy
    import torch
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float32MultiArray

    torch.set_num_threads(1)
    adapt = torch.jit.load(checked_file(Path(args.model_dir) / 'adapt.jit', ADAPT_SHA), map_location='cpu')
    body = torch.jit.load(checked_file(Path(args.model_dir) / 'body.jit', BODY_SHA), map_location='cpu')
    adapt.eval()
    body.eval()
    state = {'imu': None, 'joints': None}
    lock = threading.Lock()

    def update(name, msg):
        with lock:
            state[name] = msg

    rospy.init_node('aliengo_rl_preview', anonymous=False)
    rospy.Subscriber('/trunk_imu', Imu, lambda msg: update('imu', msg), queue_size=1)
    rospy.Subscriber('/aliengo_gazebo/joint_states', JointState,
                     lambda msg: update('joints', msg), queue_size=1)
    pub = rospy.Publisher('/rl_policy/action_preview', Float32MultiArray, queue_size=1)
    obs = ObservationHistory()
    count = 0
    deadline = time.monotonic() + args.timeout
    last_sim_time = None
    last_step = None
    last_progress = time.monotonic()
    print('只读策略推理：不发送关节控制命令；意图为', command, flush=True)
    while not rospy.is_shutdown() and (not args.samples or count < args.samples):
        with lock:
            imu, joint = state['imu'], state['joints']
        now = rospy.Time.now().to_sec()
        if last_sim_time is not None and now < last_sim_time:
            raise RuntimeError('仿真时间回退，请重启推理')
        if last_sim_time is None or now > last_sim_time:
            last_progress = time.monotonic()
        elif time.monotonic() - last_progress > args.timeout:
            raise RuntimeError('仿真时钟未更新')
        last_sim_time = now
        if last_step is not None and now - last_step < .02:
            time.sleep(.001)
            continue
        if imu is None or joint is None:
            if time.monotonic() > deadline:
                raise RuntimeError('等待 IMU/关节反馈超时')
            time.sleep(.01)
            continue
        ages = (now - imu.header.stamp.to_sec(), now - joint.header.stamp.to_sec())
        if any(age < -0.05 or age > .2 for age in ages):
            obs.reset()
            rospy.logwarn_throttle(5, '传感器反馈过期，策略历史已重置，等待新反馈')
            if time.monotonic() > deadline:
                raise RuntimeError('持续未收到有效反馈')
            time.sleep(.01)
            continue
        deadline = time.monotonic() + args.timeout
        if last_step is not None and now - last_step > .1:
            obs.reset()
            rospy.logwarn_throttle(5, '推理周期错过超过 0.1 秒，策略历史已重置')
        last_step = now
        names = list(joint.name)
        if len(set(names)) != len(names) or len(joint.position) != len(names) or len(joint.velocity) != len(names):
            raise ValueError('关节消息不完整或名称重复')
        values = {name: {'q': joint.position[i], 'dq': joint.velocity[i]}
                  for i, name in enumerate(names)}
        if not all(name in values for name in JOINTS):
            raise ValueError('缺少 AlienGo 关节')
        q = imu.orientation
        history = obs.append([q.x, q.y, q.z, q.w], values, command)
        with torch.no_grad():
            tensor = torch.tensor([history], dtype=torch.float32)
            latent = adapt(tensor)
            action = body(torch.cat((tensor, latent), dim=-1)).reshape(-1).tolist()
        action = finite_sequence(action, 12)
        obs.accept_action(action)
        pub.publish(Float32MultiArray(data=obs.previous_action))
        count += 1
        if count == 1 or count % 50 == 0 or (args.samples and count == args.samples):
            print('推理 #%d: %s' % (count, ' '.join('%.4f' % a for a in obs.previous_action)), flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print('推理停止:', error, file=sys.stderr)
        raise SystemExit(1)
