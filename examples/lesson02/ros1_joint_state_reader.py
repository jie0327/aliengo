#!/usr/bin/env python3
"""Read the right-front leg from the ROS 1 simulator without publishing."""
import argparse
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joint_state_reader import ReaderState, positive_float, positive_int


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/aliengo_gazebo/joint_states')
    parser.add_argument('--samples', type=positive_int, default=5)
    parser.add_argument('--timeout', type=positive_float, default=10.0)
    parser.add_argument('--rate', type=positive_float, default=2.0)
    args = parser.parse_args(argv)
    try:
        import rospy
        from sensor_msgs.msg import JointState
    except ImportError:
        parser.error('ROS 1 Python packages unavailable; use scripts/ros1_course.sh sim')
    rospy.init_node('lesson02_joint_state_reader', anonymous=True, disable_signals=True)
    state = ReaderState(args.rate)
    lock = threading.Lock()

    def callback(msg):
        with lock:
            line = state.consume(msg.name, msg.position, time.monotonic())
        if line is not None:
            print(line, flush=True)

    subscriber = rospy.Subscriber(args.topic, JointState, callback, queue_size=10)
    deadline = time.monotonic() + args.timeout
    interrupted = False
    try:
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with lock:
                if state.samples >= args.samples:
                    break
            time.sleep(0.02)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        subscriber.unregister()
        rospy.signal_shutdown('observation complete')
    if interrupted:
        return 130
    with lock:
        if state.samples >= args.samples:
            print(f'完成：received={state.messages}, complete_printed={state.samples}')
            return 0
        print(f'超时：received={state.messages}, complete_printed={state.samples}; {state.last_issue}',
              file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
