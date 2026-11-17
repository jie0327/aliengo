#!/usr/bin/env python3
"""Fetch the pinned CitySim assets and generate the static AlienGo world."""
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "ros1/worlds/city_sources.json"


def git(path, *args, capture=False):
    return subprocess.run(
        ["git", "-C", str(path), *args], check=True,
        text=True, capture_output=capture,
    )


def prepare_source(source):
    path = ROOT / source["path"]
    commit = source["commit"]
    if (path / ".git").exists():
        head = git(path, "rev-parse", "HEAD", capture=True).stdout.strip()
        if head == commit:
            print("复用固定来源:", path, flush=True)
            return
    else:
        if path.exists():
            raise RuntimeError("已有非 Git 目录，请先检查: " + str(path))
        path.mkdir(parents=True)
        git(path, "init")
        git(path, "remote", "add", "origin", source["repository"])

    print("准备固定来源:", path, flush=True)
    # Cone mode also retains root files such as LICENSE and README.md.
    git(path, "sparse-checkout", "init", "--cone")
    git(path, "sparse-checkout", "set", *source["sparse_directories"])
    git(path, "fetch", "--depth=1", "--filter=blob:none", "origin", commit)
    git(path, "checkout", "--detach", commit)


def main():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for source in manifest["sources"]:
        prepare_source(source)
    subprocess.run(
        [sys.executable, str(ROOT / manifest["world"]["generator"])], check=True,
    )


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print("城市资源准备失败:", error, file=sys.stderr)
        sys.exit(1)
