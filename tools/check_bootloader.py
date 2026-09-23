#!/usr/bin/env python3
"""Validate the SRAM boot header, checksum and stack reserve of the 1014D loader."""

import argparse
from pathlib import Path
import struct


def check_bootloader(data):
    if len(data) < 32 or data[4:12] != b"eGON.BT0":
        raise ValueError("missing eGON.BT0 header")
    if struct.unpack_from("<I", data)[0] != 0xEA000006:
        raise ValueError("unexpected SRAM entry branch")
    checksum, length = struct.unpack_from("<II", data, 12)
    if length != len(data) or length % 512:
        raise ValueError("boot header length/alignment mismatch")
    if length > 0x7000:
        raise ValueError("loader reaches SRAM stack reserve")
    words = struct.unpack("<%dI" % (length // 4), data)
    computed = (sum(words) - checksum + 0x5F0A6C39) & 0xFFFFFFFF
    if computed != checksum:
        raise ValueError("boot header checksum mismatch")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("loader", type=Path)
    args = parser.parse_args()
    try:
        check_bootloader(args.loader.read_bytes())
    except ValueError as error:
        parser.exit(1, f"Bootloader error: {error}\n")
    print("1014D bootloader header/checksum/size check passed")


if __name__ == "__main__":
    main()
