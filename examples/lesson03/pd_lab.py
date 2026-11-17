#!/usr/bin/env python3
"""Lesson 03: offline, one-joint mathematical toy; never connects to ROS or a robot."""
import argparse
import csv
from dataclasses import asdict, dataclass, replace
import math
from pathlib import Path
import sys


@dataclass(frozen=True)
class Config:
    # All defaults are invented teaching values, not AlienGo parameters.
    target: float = 0.5       # rad; constant target, so target velocity is zero
    q0: float = 0.0           # rad
    dq0: float = 0.0          # rad/s
    kp: float = 4.0           # N*m/rad
    kd: float = 0.8           # N*m*s/rad
    inertia: float = 0.05     # kg*m^2
    damping: float = 0.1      # N*m*s/rad; passive viscous damping
    load: float = 0.2         # N*m; positive load resists positive rotation
    feedforward: float = 0.0  # N*m
    torque_limit: float = 3.0 # N*m; symmetric actuator limit
    dt: float = 0.002         # s
    duration: float = 4.0     # s


def pd_torque(q, dq, *, target, kp, kd, feedforward, torque_limit):
    raw = kp * (target - q) + kd * (0.0 - dq) + feedforward
    applied = max(-torque_limit, min(torque_limit, raw))
    return raw, applied


def simulate(config):
    for name, value in asdict(config).items():
        if not math.isfinite(value):
            raise ValueError(f"{name}需要有限数")
    for name in ("inertia", "torque_limit", "dt", "duration"):
        if getattr(config, name) <= 0:
            raise ValueError(f"{name}需要大于0")
    for name in ("kp", "kd", "damping"):
        if getattr(config, name) < 0:
            raise ValueError(f"{name}不能为负数")
    step_count = config.duration / config.dt
    if not math.isfinite(step_count) or step_count > 200000:
        raise ValueError("最多200000步；增大dt或缩短duration")
    steps = math.ceil(step_count)
    rows = []
    q, dq = config.q0, config.dq0
    for step in range(steps + 1):
        t = min(step * config.dt, config.duration)
        raw, torque = pd_torque(q, dq, target=config.target, kp=config.kp,
                                kd=config.kd, feedforward=config.feedforward,
                                torque_limit=config.torque_limit)
        acceleration = (torque - config.damping * dq - config.load) / config.inertia
        if not all(math.isfinite(value) for value in (q, dq, raw, acceleration)):
            raise ValueError("数值发散；尝试减小dt并检查增益、惯量")
        rows.append({"t_s": t, "q_rad": q, "dq_rad_s": dq,
                     "target_rad": config.target, "error_rad": config.target - q,
                     "tau_raw_Nm": raw, "tau_applied_Nm": torque,
                     "ddq_rad_s2": acceleration,
                     "saturated": int(abs(raw) > config.torque_limit)})
        if step < steps:
            h = min(config.dt, config.duration - t)
            dq = dq + acceleration * h
            q = q + dq * h  # semi-implicit Euler: use the NEW velocity
    return rows


def save_csv(rows, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def print_table(rows):
    print("    t(s)      q(rad)    dq(rad/s)   error(rad)    tau(Nm)")
    indices = sorted({round(i * (len(rows) - 1) / 8) for i in range(9)})
    for index in indices:
        row = rows[index]
        print(f"{row['t_s']:8.3f} {row['q_rad']:11.5f} {row['dq_rad_s']:12.5f} "
              f"{row['error_rad']:12.5f} {row['tau_applied_Nm']:10.5f}")


def save_plot(traces, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise ValueError("当前Python没有matplotlib；省略--plot仍可得到表格与CSV，"
                         "或在已有matplotlib的主机Python运行") from None
    figure, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for name, rows in traces.items():
        times = [row["t_s"] for row in rows]
        axes[0].plot(times, [row["q_rad"] for row in rows], label=name)
        axes[1].plot(times, [row["tau_applied_Nm"] for row in rows], label=name)
    first = next(iter(traces.values()))
    axes[0].axhline(first[0]["target_rad"], color="black", linestyle="--", label="target")
    axes[0].set_ylabel("q (rad)")
    axes[1].set_ylabel("applied torque (N m)")
    axes[1].set_xlabel("time (s)")
    for axis in axes:
        axis.grid(True)
        axis.legend(fontsize=8)
    figure.suptitle("Offline teaching toy: not AlienGo parameters")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description="离线单关节PD数学实验；所有默认值均非真机参数")
    defaults = Config()
    descriptions = {
        "target": "固定目标角度(rad)", "q0": "初始角度(rad)",
        "dq0": "初始角速度(rad/s)", "kp": "位置增益(N*m/rad)",
        "kd": "速度增益(N*m*s/rad)", "inertia": "转动惯量(kg*m²)",
        "damping": "被动阻尼(N*m*s/rad)", "load": "抵抗正转的固定负载力矩(N*m)",
        "feedforward": "前馈力矩(N*m)", "torque_limit": "对称力矩上限(N*m)",
        "dt": "积分步长(s)", "duration": "实验时长(s)",
    }
    for name, description in descriptions.items():
        parser.add_argument("--" + name.replace("_", "-"), type=float,
                            default=getattr(defaults, name),
                            help=f"{description}，默认{getattr(defaults, name):g}")
    parser.add_argument("--csv", type=Path, default=Path("logs/course/lesson03/pd.csv"),
                        help="CSV路径；suite会在文件名后加实验名")
    parser.add_argument("--suite", action="store_true", help="分别对照Kp、Kd、前馈、限幅和步长")
    parser.add_argument("--plot", type=Path, help="可选PNG路径，需要已有matplotlib")
    args = parser.parse_args(argv)
    config = Config(**{name: getattr(args, name) for name in descriptions})
    cases = {"base": config}
    if args.suite:
        cases.update({"kp_low": replace(config, kp=config.kp / 2),
                      "kp_high": replace(config, kp=config.kp * 2),
                      "kd_low": replace(config, kd=config.kd / 16),
                      "kd_high": replace(config, kd=config.kd * 2),
                      "load_ff": replace(config, feedforward=config.load),
                      "limited": replace(config, torque_limit=config.torque_limit / 30),
                      "coarse_dt": replace(config, dt=0.2)})
    traces = {}
    try:
        print("离线数学toy：参数未经AlienGo辨识，程序不连接机器人。")
        for name, case in cases.items():
            rows = simulate(case)
            traces[name] = rows
            path = args.csv.with_name(f"{args.csv.stem}_{name}{args.csv.suffix}") if args.suite else args.csv
            save_csv(rows, path)
            if not args.suite:
                print_table(rows)
            print(f"CSV: {path}")
        if args.suite:
            print("case          final q(rad)   final error   peak q(rad)   max|tau|  saturated%")
            for name, rows in traces.items():
                # Final row is a state snapshot, not an interval to integrate.
                saturated = sum(row["saturated"] for row in rows[:-1]) / (len(rows) - 1)
                print(f"{name:12} {rows[-1]['q_rad']:13.5f} {rows[-1]['error_rad']:13.5f} "
                      f"{max(row['q_rad'] for row in rows):13.5f} "
                      f"{max(abs(row['tau_applied_Nm']) for row in rows):10.4f} {100*saturated:10.2f}")
        if args.plot:
            save_plot(traces, args.plot)
            print(f"PNG: {args.plot}")
    except (ValueError, OSError) as error:
        print(f"实验失败：{error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
