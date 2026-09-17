#!/usr/bin/env python3
"""build.py -- compile Teensy sketches into builds/teensy/<id>/latest.hex

Stub / documentation only. Wire this to arduino-cli or your preferred toolchain.

Expected layout after a successful build:
    FINAL/builds/teensy/<id>/latest.hex
    FINAL/builds/xiao/<id>/latest.bin   (optional)

Where <id> matches an entry in builds/manifest.json (e.g. ota_base, corner_full_law).

USB flash: use Arduino IDE / teensy_loader_cli with the sketch under firmware/teensy/<id>/.
Wireless flash: only ota_capable targets (currently ota_base) are intended for the
XIAO console OTA pipeline; other sketches need USB until they inherit the OTA base.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FINAL = HERE.parent.parent
MANIFEST = FINAL / "builds" / "manifest.json"
OUT_TEENSY = FINAL / "builds" / "teensy"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("target", nargs="?", help="manifest teensy id (e.g. ota_base)")
    ap.add_argument("--list", action="store_true", help="list teensy targets")
    args = ap.parse_args()

    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    teensy = {t["id"]: t for t in data.get("teensy", [])}

    if args.list or not args.target:
        for tid, t in teensy.items():
            flag = "ota" if t.get("ota_capable") else "usb"
            print(f"  {tid:24} [{flag}]  -> builds/teensy/{tid}/latest.hex")
        if not args.target:
            print("\nUsage: python build.py <id>")
            print("Not implemented: invoke arduino-cli compile and copy .hex to the path above.")
            return 0

    if args.target not in teensy:
        print(f"unknown target: {args.target}", file=sys.stderr)
        return 1

    out = OUT_TEENSY / args.target / "latest.hex"
    print(f"TODO: compile {teensy[args.target]['path']} -> {out}")
    print("Stub only; no compiler invoked.")
    return 2


if __name__ == "__main__":
    sys.exit(main())