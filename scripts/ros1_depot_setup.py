#!/usr/bin/env python3
"""下载固定版本 Depot 资源、校验归档并生成 ROS1 仓库场景。"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = json.loads((ROOT / "ros1/worlds/depot_source.json").read_text())
    archive = ROOT / source["archive"]
    archive.parent.mkdir(parents=True, exist_ok=True)
    if not archive.exists():
        print("下载 Depot v6（约 78 MiB）:", source["archive_url"], flush=True)
        temporary = archive.with_suffix(".zip.part")
        urllib.request.urlretrieve(source["archive_url"], temporary)
        temporary.rename(archive)
    with archive.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    if digest != source["sha256"]:
        raise RuntimeError("Depot 归档 SHA256 不匹配: " + str(archive))
    destination = ROOT / source["path"]
    with zipfile.ZipFile(archive) as bundle:
        for member in bundle.namelist():
            if Path(member).is_absolute() or ".." in Path(member).parts:
                raise RuntimeError("归档路径不合法: " + member)
        bundle.extractall(destination)
    subprocess.run([sys.executable, str(ROOT / "scripts/prepare_depot_world.py")],
                   check=True)


if __name__ == "__main__":
    main()
