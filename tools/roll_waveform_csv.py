#!/usr/bin/env python3
"""Export actual timestamped observations from a 1014D v1.0.0.3 roll waveform."""

import argparse
import csv
from pathlib import Path
import struct
import sys


def read_roll(data):
    """Validate the whole file; return metadata and observed columns only."""
    settings_size, width, offset = 240 * 4, 698, 168
    payload_size = width * 7
    if len(data) != settings_size + 6000 + payload_size:
        raise ValueError("wrong roll waveform length (expected 11846 bytes)")
    settings = struct.unpack_from("<240I", data)
    if settings[1:4] != (0x4F434550, 0x34313031, 0x01000003):
        raise ValueError("not a 1014D v1.0.0.3 roll waveform")
    magic, columns, timebase, interval_ms, snapshot, size = settings[offset:offset + 6]
    delays = (1000, 400, 200, 100, 40, 20, 10)
    if (magic != 0x4C4C4F52 or columns != width or timebase >= len(delays)
            or interval_ms != delays[timebase] or size != payload_size or settings[56] != timebase):
        raise ValueError("invalid roll timing metadata")
    payload = data[settings_size + 6000:]
    checksum = (sum(settings[1:]) + sum(struct.unpack_from("<1500I", data, settings_size))
                + sum(payload)) & 0xFFFFFFFF
    if checksum != settings[0]:
        raise ValueError("waveform checksum mismatch")
    ch1, ch2, valid = (payload[i * width:(i + 1) * width] for i in range(3))
    ticks = struct.unpack_from("<698I", payload, 3 * width)
    rows, segment = [], 0
    for column, flags in enumerate(valid):
        if flags & ~7:
            raise ValueError("invalid roll validity flags")
        if flags & 4:
            segment += 1
        if not flags & 3:
            continue
        rows.append({
            "column": column, "x": column + 7, "segment": segment,
            "time_ms": -((snapshot - ticks[column]) & 0xFFFFFFFF),
            "ch1_adc": ch1[column] if flags & 1 else "",
            "ch2_adc": ch2[column] if flags & 2 else "",
        })
    return {"ms_per_column": interval_ms, "snapshot_tick": snapshot}, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("waveform", type=Path)
    parser.add_argument("csv", type=Path, help="output CSV file")
    args = parser.parse_args()
    try:
        metadata, rows = read_roll(args.waveform.read_bytes())
        with args.csv.open("w", newline="") as output:
            writer = csv.DictWriter(output, ("column", "x", "segment", "time_ms", "ch1_adc", "ch2_adc"))
            writer.writeheader()
            writer.writerows(rows)
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1
    print("%d observed columns; %d ms/column; times relative to save (ADC codes, zero=128)"
          % (len(rows), metadata["ms_per_column"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
