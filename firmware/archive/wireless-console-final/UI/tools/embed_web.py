#!/usr/bin/env python3
"""embed_web.py -- gzip index.html into a PROGMEM byte array for the XIAO.

    python embed_web.py            # -> ../../firmware/xiao/console_host/web_index.h
    python embed_web.py --check    # exit 1 if the header is stale (CI / pre-flash)
    python embed_web.py --serve    # preview the page in a browser, no hardware

WHY THE FONT IS INLINED
-----------------------
index.html references `fonts/montserrat.woff2` with a plain relative URL so
that opening the page off disk during UI work just works. The XIAO, though,
serves exactly one route for the console and has no filesystem behind it, so
before gzipping we rewrite that URL into a `data:` URI. One request, one blob,
and the page renders identically on a bench with no internet -- which is the
whole point, since it is the instrument you watch while the cube is armed.

WHY --serve EXISTS
------------------
Double-clicking index.html mostly works, but Chrome gives every file:// page
its own opaque origin and then refuses the webfont as a cross-origin request,
so the one thing you are usually trying to look at is the one thing you cannot
see. --serve hands the *inlined* page over HTTP instead, which is byte-for-byte
what the XIAO will send, and the console notices it is on localhost and offers
the host box so the preview can still drive a real cube.

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
import base64
import gzip
import hashlib
import re
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
WEB = HERE.parent / "web"
SRC = WEB / "index.html"
DST = HERE.parent.parent / "firmware" / "xiao" / "console_host" / "web_index.h"

# url("fonts/…woff2") in a CSS src: descriptor -> data: URI. Kept deliberately
# narrow: it only matches the woff2 files under web/fonts/, so nothing else in
# the page can be rewritten by accident.
FONT_URL_RE = re.compile(rb'url\(\s*"(fonts/[A-Za-z0-9_.-]+\.woff2)"\s*\)')

# ESP32-C6 has 512 KB SRAM, but PROGMEM here lands in flash and is streamed
# out, so the practical ceiling is the sketch partition rather than RAM. This
# is a smell test, not a hard limit: if the page ever gets this big it has
# stopped being the "simple console" it is supposed to be.
WARN_BYTES = 64 * 1024


def inline_fonts(raw: bytes) -> bytes:
    """Replace url("fonts/x.woff2") with the file itself as a data: URI."""
    def sub(m: "re.Match[bytes]") -> bytes:
        path = WEB / m.group(1).decode()
        if not path.exists():
            raise FileNotFoundError(f"{SRC.name} references {m.group(1).decode()}, "
                                    f"which does not exist")
        b64 = base64.b64encode(path.read_bytes())
        return b'url(data:font/woff2;base64,' + b64 + b')'

    return FONT_URL_RE.sub(sub, raw)


def render(raw: bytes) -> str:
    # mtime=0 so the output is byte-identical for identical input -- otherwise
    # every run dirties the header and --check can never pass.
    blob = gzip.compress(raw, compresslevel=9, mtime=0)
    # Hash what is actually served, not index.html, so that changing a font
    # under web/fonts/ also moves the digest. /health reports this and it is
    # the only way to tell from the outside which page a XIAO is running.
    digest = hashlib.sha256(raw).hexdigest()[:16]

    lines = []
    for i in range(0, len(blob), 16):
        chunk = blob[i:i + 16]
        lines.append("  " + " ".join(f"0x{b:02x}," for b in chunk))
    body = "\n".join(lines)

    return f"""// web_index.h -- GENERATED FILE, DO NOT EDIT BY HAND.
//
// Source : UI/web/index.html (with UI/web/fonts/*.woff2 inlined as data: URIs)
// SHA256 : {digest} (of the served page, fonts inlined)
// Size   : {len(raw)} bytes raw -> {len(blob)} bytes gzipped
//
// Regenerate after editing index.html or the fonts:
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


def serve(page: bytes, port: int) -> int:
    """Serve the inlined page on localhost until Ctrl-C. Writes nothing."""
    class Handler(BaseHTTPRequestHandler):
        # Only "/" answers, same as the device: a preview that happily serves
        # paths the XIAO does not have is a preview that lies to you.
        def do_GET(self):
            if self.path.split("?")[0] not in ("/", "/index.html"):
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(page)

        def log_message(self, *_args):
            pass

    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"serving the inlined console at http://127.0.0.1:{port}/")
    print(f"({len(page)} bytes -- exactly what the XIAO would send)")
    print("the host box is shown on localhost, so you can point it at a real "
          "XIAO's IP to drive the cube from this preview.")
    print("Ctrl-C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        httpd.server_close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="verify the committed header matches index.html; "
                         "exit 1 if not. Writes nothing.")
    ap.add_argument("--serve", action="store_true",
                    help="serve the inlined page on localhost for a browser "
                         "preview instead of writing the header.")
    ap.add_argument("--port", type=int, default=8000,
                    help="port for --serve (default: 8000)")
    args = ap.parse_args()

    if not SRC.exists():
        print(f"error: {SRC} not found", file=sys.stderr)
        return 1

    src = SRC.read_bytes()
    raw = inline_fonts(src)

    if args.serve:
        return serve(raw, args.port)

    out = render(raw)

    # Measured on the hand-written page, not the inlined one: the budget exists
    # to stop the console growing features, and a font is not a feature.
    if len(src) > WARN_BYTES:
        print(f"warning: index.html is {len(src) / 1024:.0f} KB -- this console "
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
    print(f"{SRC.name}: {len(src)} B + {len(raw) - len(src)} B inlined fonts "
          f"= {len(raw)} B -> {gz_len} B gzipped "
          f"({100 * gz_len / len(raw):.0f}%)")
    print(f"wrote {DST}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
