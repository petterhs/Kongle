"""Exercise exported-log decoding, session boundaries and real missing samples."""
import importlib.util
import json
import struct
import unittest
from pathlib import Path


SPEC = importlib.util.spec_from_file_location("hr_diagnostics", Path(__file__).resolve().parents[1] / "scripts/hr-diagnostics-to-csv.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def frame(count, elapsed, *, gain=1, version=3, phase=1, separator="-"):
    payload = bytes([version, phase, 0, 0]) + struct.pack("<HIIHHH", count | (gain << 12), 861, 6, 860, 864, elapsed)
    return "Value: (0x) " + separator.join(f"{byte:02X}" for byte in payload)


class HrDiagnosticsTests(unittest.TestCase):
    def test_furu_json_uses_only_diagnostic_frames_and_exports_no_identity(self):
        raw = list(bytes.fromhex(frame(1, 100).split("(0x) ")[1].replace("-", " ")))
        capture = {"version": 1, "address": "private", "startedAt": "private", "samples": [
            {"source": "quality", "bytes": raw},
            {"source": "diagnostics", "bytes": raw},
            {"source": "diagnostics", "bytes": raw},
            {"source": "diagnostics", "bytes": [999] * 20},
            {"source": "diagnostics", "bytes": "invalid"},
        ]}
        rows = MODULE.extract(json.dumps(capture))
        self.assertEqual(rows, MODULE.extract(frame(1, 100)))
        self.assertNotIn("private", repr(rows))

    def test_unknown_furu_capture_version_is_not_guessed(self):
        with self.assertRaises(ValueError):
            MODULE.extract(json.dumps({"version": 2, "samples": []}))

    def test_chronological_frames_preserve_timestamps_and_gaps_without_duplicate_samples(self):
        text = "\n".join([frame(1, 100), frame(1, 100), frame(3, 300, separator=" "), frame(4, 400)])
        rows = MODULE.extract(text)
        self.assertEqual([r["sample"] for r in rows], [1, 3, 4])
        self.assertEqual([r["elapsed_ms"] for r in rows], [100, 300, 400])
        self.assertEqual([r["missing_samples"] for r in rows], [0, 1, 0])
        self.assertEqual(rows[0]["ppg"], 861)
        self.assertEqual(rows[0]["gain"], 1)

    def test_reset_marker_and_counter_rollback_start_new_sessions(self):
        rows = MODULE.extract("\n".join([frame(1, 100), frame(0, 65535), frame(1, 100), frame(4, 400), frame(1, 100)]))
        self.assertEqual([r["session"] for r in rows], [1, 2, 2, 3])
        self.assertEqual(rows[-1]["missing_samples"], 0)

    def test_old_versions_idle_and_invalid_frames_are_not_interpreted_as_current_timing(self):
        for text in [frame(1, 100, version=1), frame(1, 100, version=2), frame(1, 100, phase=0),
                     frame(1, 65535), frame(1, 100, gain=3), "HR: notification [00 48] -> 72 BPM"]:
            self.assertEqual(MODULE.extract(text), [])
        self.assertIsNone(MODULE.decode(bytes(19)))

    def test_trailing_annotations_are_allowed_but_longer_packets_are_not_truncated(self):
        self.assertEqual(len(MODULE.extract(frame(1, 100) + "   (received)")), 1)
        self.assertEqual(MODULE.extract(frame(1, 100) + "-00"), [])
        self.assertEqual(MODULE.extract("FF-" + frame(1, 100).split("(0x) ")[1]), [])


if __name__ == "__main__":
    unittest.main()
