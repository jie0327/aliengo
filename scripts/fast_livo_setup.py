#!/usr/bin/env python3
"""Prepare a pinned FAST-LIVO2 workspace without changing reference repositories."""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True).strip()


def reference(root, source):
    """Fetch once, then require the pinned commit and an untouched checkout."""
    commit = source['commit']
    if not re.fullmatch(r'[0-9a-f]{40}', commit):
        raise RuntimeError('Reference commit must be a full SHA: ' + commit)
    target = root / source['reference_path']
    if not target.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=target.name + '-', dir=target.parent) as temporary:
            checkout = Path(temporary) / 'checkout'
            subprocess.run(['git', 'init', '-q', str(checkout)], check=True)
            subprocess.run(['git', '-C', str(checkout), 'remote', 'add', 'origin',
                            source['repository']], check=True)
            subprocess.run(['git', '-C', str(checkout), 'fetch', '--depth=1', 'origin',
                            commit], check=True)
            subprocess.run(['git', '-C', str(checkout), 'checkout', '--detach', commit], check=True)
            checkout.rename(target)
    if git(target, 'rev-parse', 'HEAD') != commit:
        raise RuntimeError('Pinned reference commit mismatch: ' + str(target))
    if git(target, 'status', '--porcelain'):
        raise RuntimeError('Modified reference checkout; preserving it: ' + str(target))
    return target


def replace_once(path, original, replacement):
    content = path.read_text()
    if content.count(original) != 1:
        raise RuntimeError('Unexpected pinned dependency content: ' + str(path))
    path.write_text(content.replace(original, replacement, 1))


def prepare(root, source):
    root = Path(root)
    patch = root / source['patch']
    if not patch.is_file():
        raise RuntimeError('Required simulator patch is missing: ' + str(patch))
    state = {'format': 1, 'source': source,
             'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest()}
    workspace = root / source['workspace']
    destination = workspace / 'src'
    stamp = destination / '.fast_livo-source.json'
    if destination.exists():
        if not stamp.is_file() or json.loads(stamp.read_text()) != state:
            raise RuntimeError('Existing workspace has different or missing source provenance; '
                               'preserving it: ' + str(destination) + '. Review src/.fast_livo-source.json; '
                               'move the workspace aside and rerun setup to prepare the new pinned source.')
    main_reference = reference(root, source)
    dependencies = [(item, reference(root, item)) for item in source.get('dependencies', [])]
    if not destination.exists():
        workspace.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.src-', dir=workspace) as temporary:
            staged = Path(temporary) / 'src'
            staged.mkdir()
            shutil.copytree(main_reference, staged / 'fast_livo', ignore=shutil.ignore_patterns('.git'))
            for item, original in dependencies:
                copied = staged / item['workspace_path']
                shutil.copytree(original, copied, ignore=shutil.ignore_patterns('.git'))
                if item['kind'] == 'sophus':
                    # This old API predates C++11 std::complex setter semantics.
                    replace_once(copied / 'sophus/so2.cpp',
                                 'unit_complex_.real() = 1.;\n  unit_complex_.imag() = 0.;',
                                 'unit_complex_ = std::complex<double>(1., 0.);')
                    replace_once(copied / 'CMakeLists.txt', '"-O3 -DNDEBUG "', '"-O2 -DNDEBUG "')
                    (copied / 'CATKIN_IGNORE').touch()
                elif item['kind'] == 'vikit':
                    replace_once(copied / 'vikit_common/CMakeLists.txt', ' -O3 ', ' -O2 ')
                else:
                    raise RuntimeError('Unsupported source dependency kind: ' + item['kind'])
            with patch.open('rb') as stream:
                subprocess.run(['patch', '--batch', '--forward', '-p1', '-d', str(staged / 'fast_livo')],
                               stdin=stream, check=True)
            (staged / '.fast_livo-source.json').write_text(json.dumps(state, indent=2) + '\n')
            staged.rename(destination)
    for name in ('maps', 'logs'):
        (workspace / name).mkdir(exist_ok=True)
    return workspace


def main():
    source = json.loads((ROOT / 'ros1/fast_livo/source.json').read_text())
    workspace = prepare(ROOT, source)
    print('FAST-LIVO2 workspace:', workspace)


if __name__ == '__main__':
    main()
