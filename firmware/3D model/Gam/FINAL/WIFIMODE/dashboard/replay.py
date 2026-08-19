"""replay.py  --  pretend to be the cube, so the dashboard can be built at a desk.

Reads one of the recordings in FINAL/telemetry/plot/ and re-emits it to
127.0.0.1:4210 as UDP packets in the firmware's PLOTMODE wire format, paced by
the real t_ms deltas in the file. Also injects the '#' console lines the
firmware would print, because the resolved corner/edge ID exists ONLY in those
lines -- the data stream never carries it -- and the 3D highlight, the console
pane and the arm-gate learning all hang off them.

    # terminal 1
    python app.py --target 127.0.0.1
    # terminal 2
    python replay.py                     # newest corner recording, real timing
    python replay.py --speed 4           # 4x, for scrubbing through a long run
    python replay.py --schema edge       # synthesized 10-field stream

Why --target 127.0.0.1 on the server: replay listens for the dashboard's
heartbeat and command packets so it can echo firmware-style acks back. Point
the server at the XIAO's real IP instead and the replay still works, you just
lose the acks.

The edge mode is synthesized rather than replayed -- there are no 10-column
recordings in the tree, all eight files are 21-column corner runs -- so it
exists to exercise auto-detect, the schema switch and the edge command
grammar, not to be physically meaningful.
"""

import argparse
import math
import socket
import sys
import threading
import time
from pathlib import Path

# parents[] here counts from the FILE, so parents[2] == FINAL/
# (dashboard -> WIFIMODE -> FINAL).
OUT_DIR = Path(__file__).resolve().parents[2] / "telemetry" / "plot"
DEST = ("127.0.0.1", 4210)

# Console lines lifted from the real print sequences, so the parsers in
# schemas.py are tested against the exact text the firmware emits.
BOOT_CORNER = [
    "# BMI270 connected!",
    "# CAN3 up, moteus ids 1/2/3 responding",
    "# corner resolved: [+1,-1,+1]  place_offset=2.609 deg  "
    "(best_dot=0.9912 runner_up=[+1,+1,+1] dot=0.8123)",
    "# STARTS DISARMED. a1 refused unless norm3(phi) < ARM_GATE",
]
BOOT_EDGE = [
    "# BMI270 connected!",
    "# axis=Y  edge candidate resolved: Y[+1,+1] (+X,+Z up)  "
    "K=-9.3380,-1.1087,-0.00693  (dot0=0.9981 dot1=0.0012)",
]


def newest_corner_csv():
    files = sorted(OUT_DIR.glob("telemetry_corner_*.csv"))
    if not files:
        sys.exit(f"No telemetry_corner_*.csv in {OUT_DIR}")
    return files[-1]


def ack_thread(sock, stop):
    """Answer the dashboard's commands the way the firmware would.

    Only the echoes that carry state the UI depends on -- this is a stub, not
    a simulator of the control law. 'h' is the keepalive and is deliberately
    silent, same as the real build.
    """
    armed = False
    while not stop.is_set():
        try:
            raw, addr = sock.recvfrom(512)
        except socket.timeout:
            continue
        except OSError:
            break
        cmd = raw.decode("ascii", errors="ignore").strip()
        if not cmd or cmd in ("h", "k"):
            continue

        reply = None
        if cmd == "a1":
            # Refuse once, so the UI's arm-gate learning path gets exercised
            # with the corner build's real 1.00 deg value rather than the
            # 0.5 deg every script in the tree assumes.
            if not armed:
                reply = ("# ARM REFUSED: |phi|=3.412 deg exceeds ARM_GATE=1.00 "
                         "deg. Get closer to the resolved equilibrium and retry.")
                armed = False
        elif cmd == "a0":
            reply = "# gArmed = FALSE"
            armed = False
        elif cmd.startswith("g"):
            reply = f"# gGainScale = {float(cmd[1:] or 1.0):.3f}"
        elif cmd == "c":
            reply = BOOT_CORNER[2]
        elif cmd == "e":
            reply = BOOT_EDGE[1]
        elif cmd.startswith("z"):
            reply = ("# gPhiOffset TARED to: 0.012,-0.004,0.008 deg"
                     if cmd == "z1" else "# gPhiOffset cleared to 0,0,0")
        elif cmd.startswith("p"):
            reply = ("# gHalted = TRUE (idle -- no IMU reads, no CAN traffic)"
                     if cmd == "p1" else
                     "# gHalted = FALSE (resumed, still DISARMED -- send a1)")
        elif cmd.startswith("o"):
            reply = f"# kPhiOffset = {cmd[1:]} deg"

        if reply:
            sock.sendto((reply + "\n").encode("ascii"), addr)
            print(f"  <- {cmd}   -> {reply}")


def replay_corner(sock, path, speed, loop_forever):
    rows = []
    with path.open() as f:
        header = f.readline()           # discard the 21 column names
        for line in f:
            line = line.strip()
            if line:
                rows.append(line)
    if not rows:
        sys.exit(f"{path} has no data rows")

    print(f"Replaying {len(rows)} rows from {path.name} at {speed}x")

    while True:
        t_prev = None
        wall0 = time.perf_counter()
        t0 = None
        for i, line in enumerate(rows):
            t_ms = float(line.split(",", 1)[0])
            if t0 is None:
                t0 = t_ms
            # Pace against the wall clock rather than sleeping per-row, so
            # errors do not accumulate over a 220 s file.
            due = wall0 + ((t_ms - t0) / 1000.0) / speed
            delay = due - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            sock.sendto((line + "\n").encode("ascii"), DEST)
            t_prev = t_ms
        if not loop_forever:
            return
        print("  ...looping")


def replay_edge(sock, speed, loop_forever):
    """Synthesize a 10-field edge stream: a decaying oscillation about the
    balance point, which is enough to drive every chart and the 3D view."""
    print("Synthesizing a 10-field EDGE stream at 500 Hz")
    period = 0.002 / speed
    t0 = time.perf_counter()
    i = 0
    while True:
        t = i * 0.002
        env = math.exp(-t / 12.0)
        theta = 2.4 * env * math.sin(2 * math.pi * 0.9 * t)
        theta_dot = 2.4 * env * 2 * math.pi * 0.9 * math.cos(2 * math.pi * 0.9 * t)
        tau_cmd = -0.11 * theta - 0.013 * theta_dot
        tau = max(-0.12, min(0.12, tau_cmd))
        wheel_lp = 3.1 * (1 - env)
        row = [t * 1000.0, theta, theta_dot, tau, tau_cmd, 1, 1.0,
               wheel_lp, wheel_lp * t / 6.283, wheel_lp / 6.283]
        sock.sendto((",".join(f"{v:.5f}" for v in row) + "\n").encode("ascii"),
                    DEST)
        i += 1
        due = t0 + i * period
        delay = due - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        if not loop_forever and t > 60:
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--file", type=Path, default=None,
                    help="csv to replay (default: newest telemetry_corner_*)")
    ap.add_argument("--schema", choices=("corner", "edge"), default="corner")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--once", action="store_true", help="do not loop")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind an ephemeral port; the dashboard learns our address from the first
    # packet and sends commands back to it, same as the XIAO does.
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(0.5)
    print(f"Sending to {DEST[0]}:{DEST[1]} from :{sock.getsockname()[1]}")

    stop = threading.Event()
    threading.Thread(target=ack_thread, args=(sock, stop), daemon=True).start()

    for line in (BOOT_CORNER if args.schema == "corner" else BOOT_EDGE):
        sock.sendto((line + "\n").encode("ascii"), DEST)
        time.sleep(0.05)

    try:
        if args.schema == "corner":
            replay_corner(sock, args.file or newest_corner_csv(),
                          args.speed, not args.once)
        else:
            replay_edge(sock, args.speed, not args.once)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        sock.close()


if __name__ == "__main__":
    main()
