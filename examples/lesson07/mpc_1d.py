#!/usr/bin/env python3
"""Exhaustive finite-horizon 1D teaching MPC; no ROS, OCS2 or quadruped model."""
import argparse
import csv
import itertools
import math
from pathlib import Path


def finite_values(values):
    values = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('model inputs, predicted states and costs must be finite')
    return values


def predict_step(position, velocity, acceleration, dt):
    position, velocity, acceleration, dt = finite_values((position, velocity, acceleration, dt))
    if dt <= 0:
        raise ValueError('dt must be positive')
    return finite_values((position + dt * velocity + 0.5 * dt * dt * acceleration,
                          velocity + dt * acceleration))


def choose_action(position, velocity, horizon=5, action_cost=0.01, dt=0.2,
                  target=1.0, max_speed=2.0):
    """Minimize sum((x-target)^2 + r*a^2), a in {-1,0,+1} m/s^2.

    Check predicted speed at every step. Apply only the first action externally.
    Brute force is exact for this tiny *discrete action set*, not continuous MPC.
    """
    position, velocity, action_cost, dt, target, max_speed = finite_values(
        (position, velocity, action_cost, dt, target, max_speed))
    if not isinstance(horizon, int) or isinstance(horizon, bool) or not 1 <= horizon <= 8:
        raise ValueError('horizon must be an integer in [1, 8] (exhaustive search limit)')
    if action_cost < 0 or dt <= 0 or max_speed <= 0:
        raise ValueError('nonnegative action cost, positive dt and max_speed required')
    if abs(velocity) > max_speed + 1e-12:
        raise ValueError('initial velocity already violates speed constraint')
    best = None
    # Zero first gives reproducible low-effort tie breaking without adding a cost.
    for sequence in itertools.product((0.0, -1.0, 1.0), repeat=horizon):
        x, v = position, velocity
        cost = 0.0
        states = []
        for acceleration in sequence:
            x, v = predict_step(x, v, acceleration, dt)
            if abs(v) > max_speed + 1e-12:
                break
            states.append((x, v))
            error = x - target
            cost += error * error + action_cost * acceleration * acceleration
            finite_values((cost,))
        else:
            if best is None or cost < best['cost'] - 1e-12:
                best = {'action': sequence[0], 'sequence': sequence, 'states': states, 'cost': cost}
    if best is None:
        raise ValueError('no feasible candidate sequence')
    return best


def simulate(horizon=5, action_cost=0.01, steps=40, dt=0.2):
    if not isinstance(steps, int) or isinstance(steps, bool) or not 1 <= steps <= 1000:
        raise ValueError('steps must be an integer in [1, 1000]')
    position = velocity = 0.0
    rows = [{'time_s': 0.0, 'position_m': 0.0, 'velocity_m_s': 0.0, 'applied_acceleration_m_s2': 0.0}]
    energy = integrated_error = 0.0
    for step in range(steps):
        plan = choose_action(position, velocity, horizon=horizon, action_cost=action_cost, dt=dt)
        action = plan['action']
        position, velocity = predict_step(position, velocity, action, dt)
        energy += action * action * dt
        integrated_error += (position - 1.0)**2 * dt
        rows.append({'time_s': (step + 1) * dt, 'position_m': position, 'velocity_m_s': velocity,
                     'applied_acceleration_m_s2': action})
    metrics = {'final_abs_error_m': abs(position - 1.0),
               'overshoot_m': max(0.0, max(row['position_m'] for row in rows) - 1.0),
               'max_speed_m_s': max(abs(row['velocity_m_s']) for row in rows),
               'integrated_squared_error': integrated_error, 'action_energy': energy}
    return rows, metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--horizon', type=int, default=5)
    parser.add_argument('--action-cost', type=float, default=0.01)
    parser.add_argument('--steps', type=int, default=40)
    parser.add_argument('--dt', type=float, default=0.2)
    parser.add_argument('--compare', action='store_true', help='N=1,r=.01; N=5,r=.01; N=5,r=1')
    parser.add_argument('--csv', type=Path)
    args = parser.parse_args(argv)
    configurations = [(1, 0.01), (5, 0.01), (5, 1.0)] if args.compare else [(args.horizon, args.action_cost)]
    combined = []
    try:
        for horizon, weight in configurations:
            rows, metrics = simulate(horizon, weight, args.steps, args.dt)
            print(f'horizon={horizon}, action_cost={weight:g}: ' +
                  ', '.join(f'{key}={value:.6f}' for key, value in metrics.items()))
            combined.extend(dict(horizon=horizon, action_cost=weight, **row) for row in rows)
        if args.csv:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            with args.csv.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(combined[0]))
                writer.writeheader()
                writer.writerows(combined)
            print(f'CSV: {args.csv}')
    except (ValueError, OSError) as error:
        parser.error(str(error))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
