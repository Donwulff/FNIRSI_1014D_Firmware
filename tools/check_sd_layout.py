#!/usr/bin/env python3
"""Reject 1014D boot images that reach persistent data in the reserved SD area."""

import argparse
from pathlib import Path
import struct
import subprocess


def check_layout(scope, loader, offset, defines):
    if not defines["PORT_1014D"]:
        return  # Keep the Atlan 1013D package contract unchanged.
    start = defines["SCOPE_START_SECTOR"]
    boot = defines["SD_BOOT_SECTOR"]
    reserved = defines["INPUT_CALIBRATION_SECTOR"]
    if offset != (start - boot) * 512:
        raise ValueError("scope offset does not match the 1014D loader")
    if len(scope) < 32 or scope[4:12] != b"eGON.EXE":
        raise ValueError("scope has no eGON.EXE header")
    if struct.unpack_from("<I", scope, 16)[0] != len(scope) or len(scope) % 512:
        raise ValueError("scope header length/alignment does not match the image")
    if max(len(loader), offset + len(scope)) > (reserved - boot) * 512:
        raise ValueError("packed SD image overlaps calibration/settings sectors")
    if not start < reserved < defines["SETTINGS_SECTOR"] < defines["SD_MIN_PARTITION_SECTOR"]:
        raise ValueError("invalid reserved-sector layout")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scope", type=Path)
    parser.add_argument("loader", type=Path)
    parser.add_argument("offset", type=lambda value: int(value, 0))
    args = parser.parse_args()
    header = Path(__file__).resolve().parents[1] / "fnirsi_101xd_scope/sd_card_layout.h"
    result = subprocess.run(
        ["arm-none-eabi-gcc", "-dM", "-E", "-x", "c", str(header)],
        check=True, capture_output=True, text=True,
    )
    defines = {}
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) == 3:
            try:
                defines[parts[1]] = int(parts[2], 0)
            except ValueError:
                pass
    try:
        check_layout(args.scope.read_bytes(), args.loader.read_bytes(), args.offset, defines)
    except ValueError as error:
        parser.exit(1, f"SD layout error: {error}\n")
    print("SD layout check passed")


if __name__ == "__main__":
    main()
