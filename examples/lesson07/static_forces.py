#!/usr/bin/env python3
"""Contact forces and actuator sign in a fixed-base, massless-leg toy."""
import argparse
import math


def finite_vector(values, size=None):
    values = tuple(float(v) for v in values)
    if (size is not None and len(values) != size) or not all(math.isfinite(v) for v in values):
        raise ValueError('expected finite vector of the required dimension')
    return values


def equal_support_forces(mass_kg, contacts, gravity=9.81):
    mass_kg, gravity = finite_vector((mass_kg, gravity))
    if mass_kg <= 0 or gravity <= 0 or not isinstance(contacts, int) or isinstance(contacts, bool) or contacts <= 0:
        raise ValueError('positive mass/gravity and positive integer contact count required')
    # Valid only for symmetric support with CoM centered and no acceleration.
    return [(0.0, 0.0, mass_kg * gravity / contacts) for _ in range(contacts)]


def in_friction_cone(ground_force, mu):
    fx, fy, fz = finite_vector(ground_force, 3)
    mu, = finite_vector((mu,))
    if mu < 0:
        raise ValueError('friction coefficient must be nonnegative')
    return fz >= 0 and math.hypot(fx, fy) <= mu * fz + 1e-12


def compensation_torque(jacobian, ground_force):
    """tau_actuator = -J^T F_ground, ignoring leg gravity/inertia, fixed base.

    J: three Cartesian rows (x,y,z), n joint columns, m/rad.
    F_ground: ground-on-robot force, N, in the same Cartesian frame as J.
    """
    force = finite_vector(ground_force, 3)
    rows = tuple(finite_vector(row) for row in jacobian)
    if len(rows) != 3 or not rows[0] or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError('Jacobian must be a finite rectangular 3 x n matrix')
    return tuple(-sum(rows[axis][joint] * force[axis] for axis in range(3))
                 for joint in range(len(rows[0])))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mass', type=float, default=20.0, help='teaching mass, kg; not AlienGo calibration')
    parser.add_argument('--contacts', type=int, default=4)
    parser.add_argument('--mu', type=float, default=0.5)
    parser.add_argument('--ax', type=float, default=1.0, help='horizontal acceleration to test, m/s^2')
    args = parser.parse_args(argv)
    try:
        support = equal_support_forces(args.mass, args.contacts)
        ax, = finite_vector((args.ax,))
        force = (args.mass * ax / args.contacts, 0.0, support[0][2])
        feasible = in_friction_cone(force, args.mu)
        # A local example Jacobian, NOT a calibrated AlienGo leg.
        jacobian = ((0.2, 0.0), (0.0, 0.0), (0.1, -0.2))
        tau = compensation_torque(jacobian, force)
    except ValueError as error:
        parser.error(str(error))
    print(f'teaching model: mass={args.mass:g} kg, symmetric contacts={args.contacts}, world z-up')
    print(f'weight={args.mass * 9.81:.3f} N, vertical support/foot={support[0][2]:.3f} N')
    print(f'F_ground/foot={force} N, circular_friction_cone={feasible}')
    print(f'|ax| bound under level, constant-height symmetric support: mu*g={args.mu * 9.81:.3f} m/s^2')
    print(f'robot-on-ground force={tuple(-v for v in force)} N')
    print(f'fixed-base massless-leg compensation: tau=-J^T F_ground={tau} N m')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
