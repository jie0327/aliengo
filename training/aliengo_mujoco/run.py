#!/usr/bin/env python3
"""CPU-only experimental RL training; --check does not import ML code."""
import argparse
import json
import os
from pathlib import Path
import resource
from config import ALGORITHMS, validate_steps


def positive_int(value):
    number = int(value)
    if not 1 <= number <= 4096:
        raise argparse.ArgumentTypeError('memory must be between 1 and 4096 MiB')
    return number


def limit_resources(memory_mib):
    for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                 'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
        os.environ[name] = '1'
    os.environ['CUDA_VISIBLE_DEVICES'] = ''
    cap = memory_mib * 1024 * 1024
    _, inherited_hard = resource.getrlimit(resource.RLIMIT_AS)
    if inherited_hard != resource.RLIM_INFINITY:
        cap = min(cap, inherited_hard)
    resource.setrlimit(resource.RLIMIT_AS, (cap, cap))
    return {'device': 'cpu', 'address_space_limit_bytes': cap, 'threads': 1,
            'max_processes': 1}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='check limits only; no simulation')
    parser.add_argument('--urdf', type=Path, help='expanded official AlienGo URDF')
    parser.add_argument('--output', type=Path, help='new output directory, must not exist')
    parser.add_argument('--memory-mib', type=positive_int, default=4096)
    parser.add_argument('--steps', type=int, default=4096)
    parser.add_argument('--seed', type=int, default=19)
    parser.add_argument('--algorithm', choices=ALGORITHMS, default='ppo')
    args = parser.parse_args()
    limits = limit_resources(args.memory_mib)
    if args.check:
        print(json.dumps({**limits, 'algorithm': args.algorithm})); return
    if args.urdf is None or args.output is None:
        parser.error('--urdf and --output are required for training')
    if not args.urdf.is_file():
        parser.error('URDF file does not exist')
    if not validate_steps(args.steps):
        parser.error('steps must be a positive multiple of 256')
    if args.output.exists():
        parser.error('output already exists; choose a new directory')
    # Imports intentionally occur only after CPU and virtual address-space limits.
    import train
    train.run(args.urdf, args.output, args.steps, args.seed, limits, args.algorithm)


if __name__ == '__main__':
    main()
