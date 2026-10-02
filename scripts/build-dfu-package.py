#!/usr/bin/env python3
"""Package the MCUBoot image for Furu's Nordic legacy application DFU flow."""

import argparse
import binascii
import json
import struct
import tomllib
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="MCUBoot image produced by build-mcuboot-image.sh")
    parser.add_argument("output", type=Path, help="Output DFU ZIP")
    arguments = parser.parse_args()

    image = arguments.image.read_bytes()
    if not 0x200 <= len(image) <= 0x74000 - 16:
        parser.error("image does not fit the PineTime secondary slot")
    if image[:4] != bytes.fromhex("3db8f396") or image[8:10] != bytes.fromhex("0002"):
        parser.error("input is not a PineTime MCUBoot image with a 0x200-byte header")

    version = tomllib.loads(Path("Cargo.toml").read_text())["package"]["version"]
    major, minor, patch = (int(part) for part in version.split("."))
    application_version = major * 1_000_000 + minor * 1_000 + patch
    crc = binascii.crc_hqx(image, 0xFFFF)
    init = struct.pack("<HHIHHH", 0x0052, 0xFFFF, application_version, 1, 0xFFFF, crc)
    manifest = {
        "manifest": {
            "application": {
                "bin_file": "application.bin",
                "dat_file": "application.dat",
                "init_packet_data": {
                    "device_type": 0x0052,
                    "application_version": application_version,
                    "application_size": len(image),
                },
            },
            "dfu_version": 0.5,
        }
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(arguments.output, "w", compression=ZIP_DEFLATED) as package:
        package.writestr("manifest.json", json.dumps(manifest, indent=2) + "\n")
        package.writestr("application.bin", image)
        package.writestr("application.dat", init)
    print(f"DFU package: {arguments.output} ({len(image)} bytes, CRC16 0x{crc:04x})")


if __name__ == "__main__":
    main()
