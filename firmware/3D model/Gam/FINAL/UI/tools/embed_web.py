#!/usr/bin/env python3
"""embed_web.py -- gzip index.html into a PROGMEM byte array for the XIAO.

    python embed_web.py            # -> ../../firmware/xiao/console_host/web_index.h
    python embed_web.py --check    # exit 1 if the header is stale (CI / pre-flash)

WHY GZIP AND NOT A RAW STRING
-----------------------------
Two reasons, and the second is the binding one:

  1. Size. The page is ~24 KB of text and compresses to roughly a quarter of
     that. On a 4 MB part shared with the WiFi stack that is worth having.

  2. Escaping. A raw C string literal of an HTML page has to survive every
     backslash, quote and trigraph in the source, and Arduino's preprocessor
     mangles some of it anyway. A byte array cannot be mangled.

The XIAO serves the array with `Content-Encoding: gzip`, which every browser
since IE6 handles, so there is no decompression code on the device at all.

WHY A GENERATED HEADER IS COMMITTED
-----------------------------------
So that flashing the XIAO needs nothing but the Arduino IDE. A contributor who
edits index.html and forgets to re-run this would otherwise flash a stale page
with no warning -- hence --check, and hence the source hash embedded in the
header so a mismatch is visible in a diff.
"""

import argparse
import gzip
import hashlib
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "web" / "index.html"
DST = HERE.parent.parent / "firmware" / "xiao" / "console_host" / "web_index.h"

# ESP32-C6 has 512 KB SRAM, but PROGMEM here lands in flash and is streamed
# out, so the practical ceiling is the sketch partition rather than RAM. This
# is a smell test, not a hard limit: if the page ever gets this big it has
# stopped being the "simple console" it is supposed to be.
WARN_BYTES = 64 * 1024


def render(raw: bytes) -> str:
    # mtime=0 so the output is byte-identical for identical input -- otherwise
    # every run dirties the header and --check can never pass.
    blob = gzip.compress(raw, compresslevel=9, mtime=0)
    digest = hashlib.sha256(raw).hexdigest()[:16]

    lines = []
    for i in range(0, len(blob), 16):
        chunk = blob[i:i + 16]
        lines.append("  " + " ".join(f"0x{b:02x}," for b in chunk))
    body = "\n".join(lines)

    return f"""// web_index.h -- GENERATED FILE, DO NOT EDIT BY HAND.
//
// Source : UI/web/index.html
// SHA256 : {digest} (of the uncompressed source)
// Size   : {len(raw)} bytes raw -> {len(blob)} bytes gzipped
//
// Regenerate after editing index.html:
//     cd FINAL/UI/tools && python embed_web.py
//
// Served with Content-Encoding: gzip, so the device never decompresses it.

#pragma once
#include <Arduino.h>

#define WEB_INDEX_SHA "{digest}"

static const uint8_t kWebIndexGz[] PROGMEM = {{
{body}
}};

static const size_t kWebIndexGzLen = sizeof(kWebIndexGz);
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="verify the committed header matches index.html; "
                         "exit 1 if not. Writes nothing.")
    args = ap.parse_args()

    if not SRC.exists():
        print(f"error: {SRC} not found", file=sys.stderr)
        return 1

    raw = SRC.read_bytes()
    out = render(raw)

    if len(raw) > WARN_BYTES:
        print(f"warning: index.html is {len(raw) / 1024:.0f} KB -- this console "
              f"is meant to stay small enough to read in one sitting.",
              file=sys.stderr)

    if args.check:
        if not DST.exists():
            print(f"STALE: {DST.name} does not exist. Run: python embed_web.py",
                  file=sys.stderr)
            return 1
        if DST.read_text(encoding="utf-8") != out:
            print(f"STALE: {DST.name} does not match index.html. "
                  f"Run: python embed_web.py", file=sys.stderr)
            return 1
        print(f"ok: {DST.name} is current")
        return 0

    DST.parent.mkdir(parents=True, exist_ok=True)
    DST.write_text(out, encoding="utf-8")
    gz_len = out.count("0x")
    print(f"{SRC.name}: {len(raw)} B -> {gz_len} B gzipped "
          f"({100 * gz_len / len(raw):.0f}%)")
    print(f"wrote {DST}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
