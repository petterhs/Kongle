#!/usr/bin/env python3
"""Validate a release tag and stage the OTA application package."""

import argparse
import hashlib
import re
import shutil
import subprocess
import tomllib
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tag")
    args = parser.parse_args()
    version = tomllib.loads(Path("Cargo.toml").read_text())["package"]["version"]
    match = re.fullmatch(rf"v{re.escape(version)}(?:-(rc\.[1-9][0-9]*|dev\.([0-9a-f]{{7,40}})))?", args.tag)
    if not match:
        parser.error(f"tag must be v{version}, v{version}-rc.N or v{version}-dev.COMMIT")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    if match.group(2) and not commit.startswith(match.group(2)):
        parser.error("development tag SHA does not match the checked-out commit")
    if not match.group(2):
        ancestry = subprocess.run(["git", "merge-base", "--is-ancestor", "HEAD", "origin/master"], check=False)
        if ancestry.returncode != 0:
            parser.error("stable and RC tags must point to commits already on master")

    source = Path(f"target/mcuboot/thumbv7em-none-eabihf/release/mcuboot/kongle-mcuboot-app-dfu-{version}.zip")
    if not source.is_file():
        parser.error(f"missing DFU package: {source}")
    output = Path("dist")
    output.mkdir(exist_ok=True)
    package = output / f"kongle-{args.tag}-dfu.zip"
    shutil.copyfile(source, package)
    digest = hashlib.sha256(package.read_bytes()).hexdigest()
    (output / "SHA256SUMS").write_text(f"{digest}  {package.name}\n")
    print(f"Release package: {package} sha256:{digest}")


if __name__ == "__main__":
    main()
