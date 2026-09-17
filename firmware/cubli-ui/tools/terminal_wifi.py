"""terminal_wifi.py  --  the Serial Monitor, over WiFi. No plotting.

Reproduces the Arduino Serial Monitor for any of the corner WiFi builds:
every line the Teensy emits is printed verbatim, and anything you type is
sent back to it. That is the whole contract. There is no parsing, no column
count, no chart -- which is exactly why one script drives every stage:

    corner-bringup/Stage1_CornerIDAndPulse_WiFi/   13 tab-separated fields
    corner-bringup/Stage2_RateOnly_WiFi/           18
    corner-bringup/Stage3_PositionDamping_WiFi/    18
    corner-bringup/Stage5_Release_WiFi/            21 (last col trip_reason)
    CornerBalance_WiFi/  (Stage 4)                 21, if you set its
                                                   TELEMETRY_MODE to
                                                   SERIALMONITORMODE

For Stage 4 with plots instead, use telemetry_python_wifi_corner.py -- that
one speaks PLOTMODE (comma-separated, 21 columns) and knows what each column
means. It is the ONLY corner build with a plotting option; the bring-up
stages are meant to be read.

Path: laptop <--UDP :4210--> XIAO ESP32C6 <--Serial1 @ 1 Mbaud--> Teensy 4.1
The XIAO must be in x1 (TEENSY_BRIDGE) mode; it learns this laptop's address
from the first packet it receives, which the heartbeat below provides.

---------------------------------------------------------------------------
COMMANDS -- type into this terminal, press Enter. Each stage keeps the exact
grammar of its USB twin:

    Stage 1   w<0/1/2> select wheel   p fire pulse   t<Nm> pulse size
    Stage 2   a<0/1>
    Stage 3   a<0/1>   g<0..1>   m<deg>
    Stage 5   a<0/1>   r<val> log marker
    all       c re-resolve corner     h<0/1> halt

    all WiFi builds   d<N> telemetry decimation (d2 = 250 Hz, d20 = 25 Hz)
                      l<0/1> link mode (USB / WiFi telemetry)
                      k keepalive (sent automatically, see below)
    the XIAO itself   x<0/1> WIFI_TEST / TEENSY_BRIDGE -- consumed by the
                      bridge, never relayed to the Teensy

'k' IS THE KEEPALIVE, and 'h' IS HALT. A background thread sends a bare "k"
every HEARTBEAT_INTERVAL_S; the firmware auto-disarms (Stage 1: cancels its
pulse) if nothing arrives on Serial1 for kLinkTimeoutMs = 300 ms. This is the
reverse of CornerBalance_WiFi.ino, the older Stage 4 build, where 'h' is the
keepalive and halt moved to 'p'. Sending 'k' works on BOTH -- that build
accepts it as a keepalive alias -- which is why this script drives all five
stages unmodified. Nothing here ever sends a bare 'h' on its own.

---------------------------------------------------------------------------
LOGGING: every received line is also appended to
../telemetry/serial/session_<tag>_<stamp>.log, unless --no-log. That folder
holds the SERIALMONITORMODE captures; the live plotters' PLOTMODE csv files
live apart from them in ../telemetry/plot/.

Since the tab-delimited stream carries its header row, ../plot_session_csv.py
can open a STAGE 5 log directly (21 columns, though it will label the last one
gain_scale when it is really trip_reason). Stage 1's 13 and Stages 2/3's 18
columns are outside that script's 10/21 auto-detection -- for those the log is
a raw record to read or post-process, not something to plot.

Requires: nothing outside the standard library.

Usage:
    python terminal_wifi.py --tag stage1
    python terminal_wifi.py --ip 172.20.10.14 --tag stage5
    python terminal_wifi.py --no-log
"""

import argparse
import socket
import sys
import threading
from datetime import datetime
from pathlib import Path

# ---- settings ----
DEFAULT_XIAO_IP = "172.20.10.14"   # must match XIAO_IP in xiao_teensy_bridge.ino
DEFAULT_PORT    = 4210             # must match UDP_PORT in xiao_teensy_bridge.ino
HEARTBEAT_INTERVAL_S = 0.1         # well under the firmware's 300 ms timeout
QUIET_WARN_S = 2.0                 # warn if nothing arrives for this long

# Every recording in the FINAL tree lands under FINAL/telemetry/, split by the
# mode that wrote it: SERIALMONITORMODE logs here, the live plotters' PLOTMODE
# csv next door in ../telemetry/plot/. These two are not interchangeable --
# tab-delimited, width set by whichever bring-up stage is flashed, and only
# Stage 5's 21 columns are plottable -- so they do not belong in one heap.
OUT_DIR = Path(__file__).resolve().parent.parent / "telemetry" / "serial"

QUIET_HELP = (
    "... nothing received for {:.0f}s. Check that: the XIAO is in x1 "
    "(TEENSY_BRIDGE) mode, the Teensy is in l1 (WiFi telemetry), --ip "
    "matches XIAO_IP in xiao_teensy_bridge.ino, and the board is not halted "
    "(h1). This also fires on a genuine WiFi link loss -- in which case the "
    "firmware has already disarmed itself."
)


def command_thread(sock, xiao_addr, stop_event):
    """Forwards lines typed here as UDP packets. Runs on its own thread since
    input() blocks; the main thread stays on the receive loop so output keeps
    flowing while you are mid-keystroke, exactly like a Serial Monitor."""
    while not stop_event.is_set():
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            break
        line = line.strip()
        if not line:
            continue
        sock.sendto((line + "\n").encode("ascii"), xiao_addr)
    stop_event.set()


def heartbeat_thread(sock, xiao_addr, stop_event):
    """Feeds the firmware's link watchdog and keeps the XIAO's learned laptop
    address fresh during idle periods. 'k' is a no-op on every corner build."""
    while not stop_event.is_set():
        sock.sendto(b"k\n", xiao_addr)
        stop_event.wait(HEARTBEAT_INTERVAL_S)


def main():
    parser = argparse.ArgumentParser(
        description="Serial-Monitor-over-WiFi terminal for the corner WiFi builds.")
    parser.add_argument("--ip", default=DEFAULT_XIAO_IP,
                        help=f"XIAO static IP (default {DEFAULT_XIAO_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"UDP port, both ends (default {DEFAULT_PORT})")
    parser.add_argument("--tag", default="corner",
                        help="label used in the log filename (default 'corner')")
    parser.add_argument("--no-log", action="store_true",
                        help="do not write a session log")
    args = parser.parse_args()

    xiao_addr = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(1.0)

    log_file = None
    if not args.no_log:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        log_path = OUT_DIR / f"session_{args.tag}_{stamp}.log"
        log_file = log_path.open("w", encoding="ascii", errors="replace",
                                 newline="\n")

    print(f"Listening on UDP :{args.port}, sending to {args.ip}:{args.port}.")
    if log_file is not None:
        print(f"Logging to telemetry/serial/{log_path.name}")
    print("Type a command and press Enter. 'k' keepalive is automatic; "
          "halt is h1/h0. Ctrl+C to quit.")
    print("-" * 72)

    stop_event = threading.Event()
    for target in (command_thread, heartbeat_thread):
        threading.Thread(target=target, args=(sock, xiao_addr, stop_event),
                         daemon=True).start()

    lines = 0
    quiet_ticks = 0
    try:
        while not stop_event.is_set():
            try:
                raw, _addr = sock.recvfrom(2048)
            except socket.timeout:
                quiet_ticks += 1
                # settimeout(1.0) above, so one tick is one second.
                if quiet_ticks == int(QUIET_WARN_S):
                    print(QUIET_HELP.format(QUIET_WARN_S))
                continue

            quiet_ticks = 0
            # One UDP datagram is one line -- xiao_teensy_bridge.ino sends
            # them that way -- so no reassembly is needed here.
            text = raw.decode("ascii", errors="replace").rstrip("\r\n")
            if not text:
                continue
            lines += 1
            print(text)
            if log_file is not None:
                log_file.write(text + "\n")
                log_file.flush()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        sock.close()
        if log_file is not None:
            log_file.close()
        print()
        print("-" * 72)
        if log_file is not None:
            if lines:
                print(f"Wrote {lines} lines to {log_path}")
                # The plotter resolves a bare filename against every recording
                # under FINAL/, so the folder does not have to be typed here.
                print(f"Stage 5 logs can be plotted with:  "
                      f'python ../plot_session_csv.py "{log_path.name}"')
            else:
                print(f"No lines received -- {log_path.name} is empty.")
        else:
            print(f"Received {lines} lines.")


if __name__ == "__main__":
    sys.exit(main())
