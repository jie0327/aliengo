"""Exercise source preparation on real local repositories and patch failures."""
import importlib.util
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load_setup():
    assert (ROOT / 'scripts/fast_livo_setup.py').exists(), 'FAST-LIVO2 source preparation is not implemented'
    spec = importlib.util.spec_from_file_location('fast_livo_setup', ROOT / 'scripts/fast_livo_setup.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True).strip()


@pytest.fixture
def project(tmp_path):
    root = tmp_path / 'project'
    root.mkdir()
    upstream = tmp_path / 'upstream'
    upstream.mkdir()
    (upstream / 'sample.txt').write_text('upstream\n')
    subprocess.run(['git', 'init', '-q', str(upstream)], check=True)
    git(upstream, 'add', '.')
    git(upstream, '-c', 'user.name=Test', '-c', 'user.email=test@example.com', 'commit', '-qm', 'fixture')
    patch = root / 'sim.patch'
    patch.write_text('--- a/sample.txt\n+++ b/sample.txt\n@@ -1 +1 @@\n-upstream\n+simulator\n')
    config = {
        'repository': str(upstream),
        'commit': git(upstream, 'rev-parse', 'HEAD'),
        'reference_path': 'references/FAST-LIVO2',
        'workspace': '.fast_livo',
        'patch': 'sim.patch',
        'dependencies': [],
    }
    return root, config


def test_applies_patch_only_to_workspace_and_preserves_local_edits(project):
    root, config = project
    setup = load_setup()
    workspace = setup.prepare(root, config)
    assert (workspace / 'src/fast_livo/sample.txt').read_text() == 'simulator\n'
    assert (root / 'references/FAST-LIVO2/sample.txt').read_text() == 'upstream\n'
    local_file = workspace / 'src/fast_livo/sample.txt'
    local_file.write_text('local work\n')
    assert setup.prepare(root, config) == workspace
    assert local_file.read_text() == 'local work\n'
    assert git(root / 'references/FAST-LIVO2', 'status', '--porcelain') == ''


def test_failed_patch_never_publishes_incomplete_workspace(project):
    root, config = project
    (root / 'sim.patch').write_text('--- a/missing.txt\n+++ b/missing.txt\n@@ -1 +1 @@\n-old\n+new\n')
    with pytest.raises((RuntimeError, subprocess.CalledProcessError)):
        load_setup().prepare(root, config)
    assert not (root / '.fast_livo/src').exists()
    (root / 'sim.patch').write_text('--- a/sample.txt\n+++ b/sample.txt\n@@ -1 +1 @@\n-upstream\n+recovered\n')
    workspace = load_setup().prepare(root, config)
    assert (workspace / 'src/fast_livo/sample.txt').read_text() == 'recovered\n'


def test_rejects_dirty_reference_without_overwriting_it(project):
    root, config = project
    setup = load_setup()
    setup.prepare(root, config)
    reference_file = root / 'references/FAST-LIVO2/sample.txt'
    reference_file.write_text('unexpected edit\n')
    with pytest.raises(RuntimeError, match='reference|参考'):
        setup.prepare(root, config)
    assert reference_file.read_text() == 'unexpected edit\n'


def test_refuses_unmanaged_existing_workspace(project):
    root, config = project
    saved = root / '.fast_livo/src/valuable.txt'
    saved.parent.mkdir(parents=True)
    saved.write_text('keep me\n')
    with pytest.raises(RuntimeError, match='workspace|工作区'):
        load_setup().prepare(root, config)
    assert saved.read_text() == 'keep me\n'


def test_missing_patch_fails_before_downloading_or_creating_workspace(project):
    root, config = project
    (root / 'sim.patch').unlink()
    with pytest.raises((RuntimeError, FileNotFoundError), match='patch|补丁'):
        load_setup().prepare(root, config)
    assert not (root / 'references').exists()
    assert not (root / '.fast_livo').exists()


def test_changed_patch_does_not_silently_leave_old_workspace(project):
    root, config = project
    setup = load_setup()
    workspace = setup.prepare(root, config)
    (root / 'sim.patch').write_text((root / 'sim.patch').read_text().replace('+simulator', '+different'))
    with pytest.raises(RuntimeError, match='workspace|工作区'):
        setup.prepare(root, config)
    assert (workspace / 'src/fast_livo/sample.txt').read_text() == 'simulator\n'
