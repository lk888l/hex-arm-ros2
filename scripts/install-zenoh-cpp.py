#!/usr/bin/env python3
"""Install the pinned official Zenoh C/C++ SDK under an explicit prefix.

Requires dpkg-deb; no package-manager or global library configuration changes.
Example: python3 scripts/install-zenoh-cpp.py --prefix /opt/hex-arm-zenoh
Then configure with -DCMAKE_PREFIX_PATH=/opt/hex-arm-zenoh.
"""
import argparse
import hashlib
import io
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile
import urllib.request
import zipfile

VERSION = "1.9.0"
ARCHIVES = {
    "x86_64": ("x86_64-unknown-linux-gnu", "afa6bfd2c6867ee0301d8d578be4a6a9f34dab3c132cbefc1c4cb4140ff92f1b"),
    "aarch64": ("aarch64-unknown-linux-gnu", "c3487db3ea3a70287fb09656cff97264d451c99f5167f4eb7d8dd763bae5075a"),
}
CPP_SHA256 = "ded9caedee884ad32799937ca30d3fca16285389b7769834208456a00d29c9a6"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", required=True, type=Path)
    args = parser.parse_args()
    if platform.machine() not in ARCHIVES:
        parser.error("official pinned GNU/Linux packages supported for x86_64 and aarch64")
    target, sha256 = ARCHIVES[platform.machine()]
    with tempfile.TemporaryDirectory(prefix="hex-zenoh-") as temporary:
        root = Path(temporary)
        for repo, filename, digest in (
            ("zenoh-c", f"libzenohc-{VERSION}-{target}-debian.zip", sha256),
            ("zenoh-cpp", f"zenohcpp-{VERSION}-debian.zip", CPP_SHA256),
        ):
            url = f"https://github.com/eclipse-zenoh/{repo}/releases/download/{VERSION}/{filename}"
            data = urllib.request.urlopen(url, timeout=120).read()
            if hashlib.sha256(data).hexdigest() != digest:
                raise RuntimeError(f"SHA256 mismatch: {url}")
            with zipfile.ZipFile(io.BytesIO(data)) as archive:
                for name in archive.namelist():
                    if name.endswith(".deb"):
                        package = root / Path(name).name
                        package.write_bytes(archive.read(name))
                        subprocess.run(["dpkg-deb", "-x", str(package), str(root / "sdk")], check=True)
            legal = root / "sdk/usr/share/hex_arm_zenoh" / repo
            legal.mkdir(parents=True, exist_ok=True)
            for name in ("LICENSE", "NOTICE.md"):
                source = f"https://raw.githubusercontent.com/eclipse-zenoh/{repo}/{VERSION}/{name}"
                (legal / name).write_bytes(urllib.request.urlopen(source, timeout=30).read())
        shutil.copytree(root / "sdk/usr", args.prefix, dirs_exist_ok=True)
    print(f"Zenoh C/C++ {VERSION}: {args.prefix.resolve()}")


if __name__ == "__main__":
    main()
