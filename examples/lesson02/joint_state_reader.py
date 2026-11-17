#!/usr/bin/env python3
"""Lesson 02: read JointState only; importing this module needs no ROS installation."""
import argparse
import math
import sys
import time


FR_JOINTS = ("FR_hip_joint", "FR_thigh_joint", "FR_calf_joint")


def extract_fr_positions(names, positions):
    """Return name -> angle/None and problems; never guess missing values as zero."""
    values = {}                ##字典可以通过名称查找数值
    issues = []                ##列表专门用来保存错误信息
    if len(positions) != len(names):
        issues.append(f"name长度{len(names)}与position长度{len(positions)}不同")        ##检查两个数组长度是否一致
    for joint in FR_JOINTS:
        indices = [index for index, name in enumerate(names) if name == joint]    ##enumerate() 会在遍历元素时同时提供索引
        values[joint] = None
        if not indices:
            issues.append(f"缺少关节名 {joint}")
        elif len(indices) > 1:
            issues.append(f"关节名重复 {joint}")
        elif indices[0] >= len(positions):
            issues.append(f"缺少位置 {joint}")
        else:
            angle = float(positions[indices[0]])
            if math.isfinite(angle):
                values[joint] = angle
            else:
                issues.append(f"位置不是有限数 {joint}")
    return values, issues


class ReaderState:
    """Keep display rate separate from incoming message rate."""

    def __init__(self, rate):
        self.interval = 1.0 / rate
        self.last_print = None
        self.last_receive = None
        self.messages = 0
        self.samples = 0
        self.last_issue = "尚未收到消息"

    def consume(self, names, positions, now):
        self.messages += 1
        self.last_receive = now
        values, issues = extract_fr_positions(names, positions)
        self.last_issue = "; ".join(issues) if issues else "最近一条消息完整"
        if self.last_print is not None and now - self.last_print < self.interval:
            return None
        self.last_print = now
        fields = []
        for name in FR_JOINTS:
            angle = values[name]
            fields.append(f"{name}={angle:.4f}" if angle is not None else f"{name}=N/A")
        if not issues:
            self.samples += 1
        suffix = f" | {self.last_issue}" if issues else ""
        return f"sample={self.samples} received={self.messages} rad | " + " ".join(fields) + suffix


def positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("需要大于0的有限数")
    return value


def positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("需要大于0的整数")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description="只读右前腿角度，不发布任何控制命令")
    parser.add_argument("--topic", default="/aliengo_gazebo/joint_states", help="JointState话题名")
    parser.add_argument("--samples", type=positive_int, default=5,
                        help="打印多少条完整样本后成功退出，默认5")
    parser.add_argument("--timeout", type=positive_float, default=10.0,
                        help="总墙钟时间上限（秒），默认10；超时返回1")
    parser.add_argument("--rate", type=positive_float, default=2.0,
                        help="最大打印频率（Hz），默认2；不改变发布频率")
    parser.add_argument("--demo", action="store_true", help="离线演示数组重排与缺字段，不加载ROS")
    args = parser.parse_args(argv)
    if args.demo:
        state = ReaderState(args.rate)
        cases = [
            ("原顺序", list(FR_JOINTS), [0.1, 0.7, -1.4]),
            ("重排后", [FR_JOINTS[2], FR_JOINTS[0], FR_JOINTS[1]], [-1.4, 0.1, 0.7]),
            ("缺数据", [FR_JOINTS[0]], [0.1]),
        ]
        for index, (label, names, positions) in enumerate(cases):
            print(label + ": " + state.consume(names, positions, now=index / args.rate))
        return 0
    # ROS 1 subscription is kept in a separate adapter; shared validation stays here.
    from ros1_joint_state_reader import main as ros1_main
    return ros1_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
