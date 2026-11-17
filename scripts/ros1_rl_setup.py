#!/usr/bin/env python3
"""Download the fixed UniO4 checkpoint and a CPU PyTorch runtime locally."""
import hashlib
import os
from pathlib import Path
import platform
import shutil
import sys
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / '.ros1_rl'
REV = '00fcfc088811f7629fea3561f16cbfd3294214c7'
PREFIX = ('https://raw.githubusercontent.com/anubhav1772/UniO4-Aliengo/' + REV +
          '/pretraining/runs/gait-conditioned-agility/recent/225743.070104/checkpoints/')
FILES = [
    ('adapt.jit', PREFIX + 'adaptation_module_latest.jit', 'ad8f32345fe04f34210a7c0a762b46c8231565408bee881f665057cf4a60c76b'),
    ('body.jit', PREFIX + 'body_latest.jit', '31884a6295076bcd846ea38af6c864cccb5c4cec8dda2b5ccf5da318ae32961d'),
    ('torch.whl', 'https://download-r2.pytorch.org/whl/cpu/torch-2.4.1%2Bcpu-cp38-cp38-linux_x86_64.whl', '0c0a7cc4f7c74ff024d5a5e21230a01289b65346b27a626f6c815d94b4b8c955'),
    ('typing_extensions.whl', 'https://files.pythonhosted.org/packages/26/9f/ad63fc0248c5379346306f8668cda6e2e2e9c95e01216d2b8ffd9ff037d0/typing_extensions-4.12.2-py3-none-any.whl', '04e5ca0351e0f3f85c6853954072df659d0d13fac324d0072316b67d7794700d'),
    ('filelock.whl', 'https://files.pythonhosted.org/packages/b9/f8/feced7779d755758a52d1f6635d990b8d98dc0a29fa568bbe0625f18fdf3/filelock-3.16.1-py3-none-any.whl', '2082e5703d51fbf98ea75855d9d5527e33d8ff23099bec374a134febee6946b0'),
    ('networkx.whl', 'https://files.pythonhosted.org/packages/a8/05/9d4f9b78ead6b2661d6e8ea772e111fc4a9fbd866ad0c81906c11206b55e/networkx-3.1-py3-none-any.whl', '4f33f68cb2afcf86f28a45f43efc27a9386b535d567d2127f8f61d51dec58d36'),
    ('sympy.whl', 'https://files.pythonhosted.org/packages/b2/fe/81695a1aa331a842b582453b605175f419fe8540355886031328089d840a/sympy-1.13.1-py3-none-any.whl', 'db36cdc64bf61b9b24578b6f7bab1ecdd2452cf008f34faa33776680c26d66f8'),
    ('mpmath.whl', 'https://files.pythonhosted.org/packages/43/e3/7d92a15f894aa0c9c4b49b8ee9ac9850d6e63b03c9c32c0367a13ae62209/mpmath-1.3.0-py3-none-any.whl', 'a0b2b9fe80bbcd81a6647ff13108738cfb482d481d826cc0e02f5b35e5c88d2c'),
    ('jinja2.whl', 'https://files.pythonhosted.org/packages/31/80/3a54838c3fb461f6fec263ebf3a3a41771bd05190238de3486aae8540c36/jinja2-3.1.4-py3-none-any.whl', 'bc5dd2abb727a5319567b7a813e6a2e7318c39f4f487cfe6c89c6f9c7d25197d'),
    ('fsspec.whl', 'https://files.pythonhosted.org/packages/c6/b2/454d6e7f0158951d8a78c2e1eb4f69ae81beb8dca5fee9809c6c99e9d0d0/fsspec-2024.10.0-py3-none-any.whl', '03b9a6785766a4de40368b88906366755e2819e758b83705c88cd7cb5fe81871'),
]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    if platform.machine() not in ('x86_64', 'amd64'):
        raise RuntimeError('目前提供的 CPU PyTorch 运行时只支持 x86_64')
    cache = DEST / 'cache'
    cache.mkdir(parents=True, exist_ok=True)
    for name, url, sha in FILES:
        path = cache / name
        if not path.exists() or digest(path) != sha:
            print('下载', name, flush=True)
            temp = path.with_suffix(path.suffix + '.part')
            with urllib.request.urlopen(url, timeout=90) as response, temp.open('wb') as output:
                shutil.copyfileobj(response, output)
            if digest(temp) != sha:
                raise RuntimeError('SHA256 校验失败: ' + name)
            os.replace(temp, path)
    site = DEST / 'site'
    stamp = site / '.verified-runtime'
    if not stamp.exists() or stamp.read_text().strip() != FILES[2][2]:
        if site.exists():
            raise RuntimeError('已有运行时目录无法验证；请检查 ' + str(site))
        staging = DEST / 'site-staging'
        if staging.exists():
            raise RuntimeError('已有未完成的安装目录；请检查 ' + str(staging))
        staging.mkdir()
        for name, _, _ in FILES[2:]:
            with zipfile.ZipFile(cache / name) as wheel:
                for item in wheel.infolist():
                    target = (staging / item.filename).resolve()
                    if staging.resolve() not in target.parents:
                        raise RuntimeError('wheel 路径不安全: ' + item.filename)
                wheel.extractall(staging)
        (staging / '.verified-runtime').write_text(FILES[2][2] + '\n')
        os.replace(staging, site)
    print('RL 只读推理依赖已准备:', DEST)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('准备失败:', error, file=sys.stderr)
        sys.exit(1)
