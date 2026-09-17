# Cubli console — simplified web UI + wireless flashing

A lean operator console served **by the XIAO itself**, plus an over-the-air
firmware pipeline for the Teensy. No laptop process in the path.

```
browser ──HTTP :80 / WS :81──> XIAO ESP32C6 ──Serial1 @1 Mbaud──> Teensy 4.1
```

The existing stack is **untouched and still works**: `dashboard/` (Python +
FastAPI), `telemetry_python_wifi*.py`, `terminal_wifi.py` and `link_check.py`
all keep running over the same UDP `:4210` path, because
`xiao_cubli_console.ino` is a superset of `xiao_teensy_bridge.ino` and keeps
its UDP relay byte-for-byte. You can even run a browser and the Python
dashboard at the same time — the two paths are independent.

| | |
|---|---|
| `index.html` | the whole front end: markup, CSS, canvas charts, WS client, OTA client |
| `build_web.py` | gzips it into `../xiao_cubli_console/web_index.h` |
| `../xiao_cubli_console/` | XIAO firmware: HTTP + WebSocket + UDP relay + hex streamer |
| `../CubliOta_Teensy/` | Teensy template: mode state machine + FlasherX |

## Getting it running

**1. XIAO.** Install the **WebSockets** library by Markus Sattler (Links2004)
via Library Manager. Open `../xiao_cubli_console/xiao_cubli_console.ino`, set
`WIFI_PASSWORD`, select board `XIAO_ESP32C6`, flash.

**2. Teensy.** Copy `FlashTxx.c` and `FlashTxx.h` from
[FlasherX](https://github.com/joepasquariello/FlasherX) into
`../CubliOta_Teensy/` — the sketch refuses to compile without them and says so.
Flash it. *(Do not copy `FXUtil.*`; see that sketch's header for why.)*

**3. Browse** to `http://172.20.10.14/`.

Editing `index.html` requires `python build_web.py` afterwards, or the XIAO
keeps serving the old page. `python build_web.py --check` fails if the
committed header is stale — worth running before you flash.

You can also open `index.html` straight off disk for UI work; a host box
appears in the header so you can point it at the board.

## What's on the page

Four things, and deliberately nothing else:

- **Two charts** — IMU tilt (φx/φy/φz on corner, θ on edge) and wheel speed in
  RPM. 10 s window, auto-scaled, ~25 Hz.
- **Battery gauge** — real pack voltage, see below.
- **Mode + arm** — IDLE / EDGE BALANCE / CORNER BALANCE, ARM (two-step), and a
  re-resolve button.
- **Wireless flash** — file picker, progress bar, UPLOAD then FLASH.

Plus the header: WS and UART status pills, an ARMED pill, and **E-STOP**.
Spacebar does the same thing from anywhere on the page.

Dropped from the old dashboard: the three.js attitude cube, the four
six-series charts, the scrolling firmware console, the CSV recording panel and
the schema-override panel. Recording still exists — it lives in
`dashboard/app.py`, which is what `plot_session_csv.py` consumes.

> There were no AI features to remove. Grepping the tree for AI/LLM/anomaly
> hits only `dashboard/static/vendor/*.min.js` — Tailwind, Chart.js and
> three.js matching on substrings. There were no data tables either.

## The battery gauge is real, and it isn't from a divider

`Electrical-Design-Guide.md:174` lists a battery divider on Teensy A0 as
*optional*, and it isn't built. But `moteus_protocol.h:348` puts
`Resolution voltage = kInt8` in moteus's **default** query format, so DC bus
voltage has been arriving on every CAN reply all along, unused. The template
reads it with `last_result().values.voltage` and publishes it.

Resolution is **0.5 V/LSB**, which against a 6S pack (25.2 V full, 22.2 V
nominal, 18.0 V floor) is ~14 steps across the usable span. Coarse, but real —
enough to answer "is the pack sagging", not enough for coulomb counting. If
you want better, request `kFloat` for voltage in the query format; it costs 3
bytes per reply.

It rides on a `#H,vbat,mode,armed,tmax` console line at 2 Hz rather than a new
CSV column, because adding a column would take corner from 21 to 22 fields and
break the field-count schema detection that `dashboard/schemas.py`,
`plot_session_csv.py` and both matplotlib scripts all depend on. Every one of
those already ignores `#` lines.

## Safety: the cube disarms when the page stops talking

This is the part worth reading twice.

```
browser ping every 250 ms
  └─> XIAO emits 'k' every 100 ms, ONLY while a ping is < 500 ms old
        └─> Teensy disarms 300 ms after 'k' stops   (kLinkTimeoutMs)
```

Close the tab, drop WiFi, walk out of range or crash the browser and the cube
is disarmed within ~800 ms. The XIAO also fires an explicit `a0` when the last
WebSocket client disconnects, and again the moment the ping goes stale.

**Do not make the XIAO's keepalive unconditional.** It is tempting — it would
stop nuisance disarms — and it silently deletes the entire deadman chain.

Other interlocks:

- every mode change disarms, `SetStop()`s all three wheels and clears the tare;
- `a1` is refused in IDLE, and refused above the 1.0° tilt gate;
- OTA is refused unless IDLE **and** disarmed;
- browser commands are validated against a whitelist **on the XIAO**, not just
  in the page — same reasoning as `dashboard/schemas.py`'s `CMD_OK`. `u`
  (OTA) is not in it: flashing is reachable only through the HTTP endpoints,
  so a stray socket message can never interleave with a hex stream.

## Wireless flashing

Two phases, because they fail differently and you want to know which one did.

```
UPLOAD   POST /ota/upload   XIAO streams hex records to the Teensy, which
                            checksums each one and writes it into the FlasherX
                            staging buffer (upper flash). Nothing committed.
FLASH    POST /ota/commit   Teensy validates the staged image, calls
                            flash_move(), reboots into it.
ABORT    POST /ota/abort    Teensy frees the buffer and reboots unchanged.
```

**Flow control is load-bearing.** A 4 KB sector erase on the Teensy stalls for
tens of milliseconds; at 1 Mbaud that is several KB of hex still arriving. So
the XIAO sends at most 2048 bytes, emits a `u?` probe, and blocks until
`#OTA ACK`. Because that block happens inside the WebServer's upload callback,
TCP's own window throttles the browser to exactly the rate the Teensy can
absorb — no application-level rate guessing anywhere. The Teensy's Serial1 RX
buffer is enlarged to 8 KB (4× the chunk) so a worst-case stall mid-chunk
still can't overflow it.

On-wire protocol (Serial1, line-based ASCII so it coexists with the normal
command grammar):

| | |
|---|---|
| `u1` → | `#OTA READY <bufsize>` or `#OTA REFUSED <reason>` |
| `:...` → | Intel HEX records; failures answer `#OTA ERR <reason>` |
| `u?` → | `#OTA ACK <records> <bytes>` |
| `u2` → | validate + `flash_move()`. Reboots; no reply |
| `u0` → | free buffer, reboot unchanged |

`u` is free in every existing grammar. An explicit sentinel starts OTA rather
than sniffing for lines beginning with `:` — a telemetry glitch must never be
able to put a balancing cube into a flashing state.

### Things that will surprise you

**Any image you upload must itself include FlasherX.** `check_flash_id()`
searches the staged image for the literal `fw_teensy41` that `FlashTxx.c`
compiles in. An image built without it is refused — correctly, because it
would also be the last one you could ever flash wirelessly.

**Abort always reboots the Teensy, and that's mandatory.**
`flash_write_block()` keeps `static` `buf_count`/`next_addr` across calls, so a
transfer that stops part-way through a 4-byte group leaves `buf_count > 0` and
every later attempt fails with its error 2 until the statics re-initialise.
Only a reboot does that. FlasherX's own example reboots after a failed update
for exactly this reason.

**`flash_write_block()` requires 4-byte-aligned addresses and counts.** Teensy
hex files are 16- or 32-byte records throughout so this holds in practice, but
a hex file with an unaligned final data record will be rejected rather than
padded — padding mid-stream would break the sequential-address requirement and
corrupt the image, which is worse than a clear error.

**The cube is uncontrolled during the reboot** and through the ~2 s gyro
re-calibration that follows. Flash with it resting on a flat surface. The FLASH
button confirms this before it fires.

## Status

**None of this has run on hardware.** No arduino-cli in this environment, so
the two sketches have not been compile-checked either; the page's JavaScript
parses clean under `node --check`. The Teensy template's control laws are
stubs that command zero torque — see its own README before changing that.

First bench session, in order: flash both boards → load the page → confirm
telemetry and battery → confirm E-STOP → confirm mode buttons → *then* try OTA
with a known-good hex, cube on the bench, USB cable attached so you can watch
the Teensy's own console and recover if the commit is refused.
