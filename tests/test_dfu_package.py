"""Regression tests for the InfiniTime legacy DFU buffer boundary."""

import binascii
import json
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from zipfile import ZipFile


ROOT = Path(__file__).resolve().parents[1]


class DfuPackageTests(unittest.TestCase):
    def package(self, image_size: int) -> tuple[bytes, bytes, dict]:
        image = bytearray(b"\xff" * image_size)
        image[:4] = bytes.fromhex("3db8f396")
        image[8:10] = bytes.fromhex("0002")
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "image.bin"
            output = Path(directory) / "dfu.zip"
            source.write_bytes(image)
            subprocess.run(
                [sys.executable, "scripts/build-dfu-package.py", str(source), str(output)],
                cwd=ROOT,
                check=True,
                capture_output=True,
            )
            with ZipFile(output) as package:
                return (
                    package.read("application.bin"),
                    package.read("application.dat"),
                    json.loads(package.read("manifest.json")),
                )

    def test_exact_buffer_boundary_gets_erased_padding_and_matching_metadata(self) -> None:
        image, init, manifest = self.package(600)
        self.assertEqual(len(image), 604)
        self.assertEqual(image[-4:], b"\xff" * 4)
        self.assertEqual(len(image) % 200, 4)
        self.assertEqual(struct.unpack_from("<H", init, len(init) - 2)[0], binascii.crc_hqx(image, 0xFFFF))
        self.assertEqual(manifest["manifest"]["application"]["init_packet_data"]["application_size"], len(image))

    def test_non_boundary_image_keeps_its_length(self) -> None:
        image, _, _ = self.package(604)
        self.assertEqual(len(image), 604)


if __name__ == "__main__":
    unittest.main()
