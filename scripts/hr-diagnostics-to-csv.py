#!/usr/bin/env python3
"""Extract Kongle version-3 optical diagnostics from nRF text or Furu JSON.

Accepts nRF Connect's hyphenated or space-separated 20-byte payloads. Old
diagnostic versions have different timestamp semantics and are not guessed.
Only measuring frames are exported. Lost samples remain gaps, never fillers.
"""

import argparse
import csv
import json
import re
import struct
import sys
from pathlib import Path


FRAME = re.compile(r"(?<![0-9a-fA-F])(?<![0-9a-fA-F]{2}[- ])([0-9a-fA-F]{2}(?:[- ][0-9a-fA-F]{2}){19})(?![0-9a-fA-F]|[- ][0-9a-fA-F]{2}\b)")
FIELDS = ["session", "sample", "elapsed_ms", "gain", "ppg", "ambient", "bpm", "error", "minimum", "maximum", "missing_samples"]


def decode(payload: bytes) -> dict | None:
    if len(payload) != 20 or payload[0] != 3 or payload[1] != 1:
        return None
    packed, ppg, ambient, minimum, maximum, elapsed = struct.unpack_from("<HIIHHH", payload, 4)
    gain = packed >> 12
    count = packed & 0x0FFF
    if gain not in (1, 2, 4, 8) or ppg > 0x3FFFF or ambient > 0x3FFFF:
        return None
    if count and elapsed == 0xFFFF:
        return None
    return dict(sample=count, elapsed_ms=elapsed, gain=gain, ppg=ppg,
                ambient=ambient, bpm=payload[2], error=payload[3],
                minimum=minimum, maximum=maximum)


def extract(text: str) -> list[dict]:
    if text.lstrip().startswith("{"):
        capture = json.loads(text)
        if capture.get("version") != 1 or not isinstance(capture.get("samples"), list):
            raise ValueError("unsupported Furu capture format")
        frames = []
        for sample in capture["samples"]:
            if not isinstance(sample, dict) or sample.get("source") != "diagnostics":
                continue
            payload = sample.get("bytes")
            if not isinstance(payload, list) or len(payload) != 20:
                continue
            if not all(type(value) is int and 0 <= value <= 255 for value in payload):
                continue
            frames.append(bytes(payload).hex("-"))
        # Keep the same decoder, session splitting and deduplication as text.
        # Device address and wall-clock metadata are deliberately not exported.
        text = "\n".join(frames)
    rows = []
    previous = None
    session = 1
    start_pending = False
    for line in text.splitlines():
        for match in FRAME.finditer(line):
            row = decode(bytes.fromhex(match.group(1).replace("-", " ")))
            if row is None:
                continue
            if row["sample"] == 0:
                start_pending = previous is not None
                continue
            if previous is not None:
                if start_pending or row["sample"] < previous["sample"] or row["elapsed_ms"] < previous["elapsed_ms"]:
                    session += 1
                    previous = None
                elif row["sample"] == previous["sample"]:
                    continue  # Same physical sample from a repeated read/notify.
            row["session"] = session
            row["missing_samples"] = max(0, row["sample"] - (previous["sample"] if previous else 0) - 1)
            rows.append(row)
            previous = row
            start_pending = False
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, help="CSV destination (default stdout)")
    args = parser.parse_args()
    try:
        rows = extract(args.log.read_text(encoding="utf-8-sig", errors="replace"))
    except ValueError as error:
        parser.error(str(error))
    if not rows:
        parser.error("no measuring version-3 diagnostic frames; enable the custom diagnostic notifications, not only 0x2a37")
    stream = args.output.open("w", newline="", encoding="utf-8") if args.output else sys.stdout
    try:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    finally:
        if args.output:
            stream.close()
    for session in sorted({row["session"] for row in rows}):
        samples = [row for row in rows if row["session"] == session]
        intervals = [b["elapsed_ms"] - a["elapsed_ms"] for a, b in zip(samples, samples[1:])]
        cadence = f"{min(intervals)}..{max(intervals)} ms" if intervals else "unknown"
        print(f"Session {session}: {len(samples)} samples, PPG {min(r['ppg'] for r in samples)}..{max(r['ppg'] for r in samples)}, "
              f"missing {sum(r['missing_samples'] for r in samples)}, intervals {cadence}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
