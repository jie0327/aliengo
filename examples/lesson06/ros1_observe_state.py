#!/usr/bin/env python3
"""Read ROS 1 IMU and joints without publishing commands."""
import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from observe_state import ObserverState, finite_vector, imu_summary, joint_summary, stamp_seconds


def read_sample(stream, message):
    stamp = stamp_seconds(message.header.stamp.secs, message.header.stamp.nsecs)
    if stream == 'imu':
        q, w, a = message.orientation, message.angular_velocity, message.linear_acceleration
        result = imu_summary((q.x, q.y, q.z, q.w), (w.x, w.y, w.z), (a.x, a.y, a.z),
                             message.orientation_covariance[0] != -1,
                             message.angular_velocity_covariance[0] != -1,
                             message.linear_acceleration_covariance[0] != -1)
        result['covariance_note'] = 'all-zero covariance means unknown, not perfect accuracy'
    elif stream == 'joints':
        result = joint_summary(message.name, message.position)
        for field in ('velocity', 'effort'):
            values = getattr(message, field)
            if values and len(values) != len(message.name):
                raise ValueError(f'joint {field} length differs from names')
            finite_vector(values)
    else:
        raise ValueError(f'unknown stream {stream}')
    result.update(stamp_s=stamp, frame_id=message.header.frame_id)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--imu-topic', default='/trunk_imu')
    parser.add_argument('--joint-topic', default='/aliengo_gazebo/joint_states')
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--timeout', type=float, default=8.0, help='wall-clock seconds')
    args = parser.parse_args(argv)
    if not 1 <= args.samples <= 10000 or not math.isfinite(args.timeout) or not 0 < args.timeout <= 60:
        parser.error('samples must be in [1,10000], timeout in (0,60] seconds')
    try:
        import rospy
        from sensor_msgs.msg import Imu, JointState
    except ImportError:
        parser.error('ROS 1 Python packages unavailable; use scripts/ros1_course.sh sim')
    rospy.init_node('lesson06_readonly_observer', anonymous=True, disable_signals=True)
    state = ObserverState(args.samples, args.timeout, time.monotonic())
    lock = threading.Lock()

    def record(message, stream):
        with lock:
            state.consume(stream, lambda: read_sample(stream, message), time.monotonic())

    subscribers = [rospy.Subscriber(args.imu_topic, Imu, record, 'imu', queue_size=10),
                   rospy.Subscriber(args.joint_topic, JointState, record, 'joints', queue_size=10)]
    interrupted = False
    try:
        while not rospy.is_shutdown():
            with lock:
                if state.finished(time.monotonic()):
                    break
            time.sleep(0.02)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            report = state.report(time.monotonic())
        rospy.signal_shutdown('observation complete')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('消息时间戳来自发布者；墙钟超时与仿真时钟独立。两路样本未同步。')
    return 130 if interrupted else (0 if report['complete'] else 2)


if __name__ == '__main__':
    raise SystemExit(main())
