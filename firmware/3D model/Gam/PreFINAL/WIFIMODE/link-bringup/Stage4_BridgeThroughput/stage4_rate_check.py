"""stage4_rate_check.py -- does the full pipeline carry 250 lines/s intact?

Pairs with Stage4_BridgeThroughput/teensy_csv_faker/teensy_csv_faker.ino on
the Teensy and the REAL ../../../xiao_teensy_bridge/xiao_teensy_bridge.ino on
the XIAO. Nothing in this stage can move a wheel -- the faker never touches
CAN.

    python stage4_rate_check.py
    python stage4_rate_check.py --seconds 60          # soak it
    python stage4_rate_check.py --ip 192.168.1.42
    python stage4_rate_check.py --command a1          # test the command path

WHY EXACT LOSS, NOT A RATE

The faker sets t_ms = seq*4, so seq = t_ms/4 is recoverable from every line.
That turns "telemetry looks choppy" into a number: how many of the lines the
Teensy emitted never arrived, and whether they went missing in ones (radio
noise) or in long runs (the bridge stalling, or a WiFi roam). A plain
packets-per-second average hides both.

WHAT GOOD LOOKS LIKE

    rate 249.6/s   loss 0.2%   longest gap 2 lines   out-of-order 0

Under ~1% loss in ones and twos is normal for UDP over WiFi and the plotter
will look perfectly smooth. What is NOT normal, and what this stage exists to
catch before the cube is ever armed:

  rate collapses to ~0.5/s   the XIAO's USB is plugged in with nothing
                             reading it. Its CDC blocks for ~2 s per print and
                             the bridge relays one line every two seconds.
                             Unplug it, or keep a monitor open and reading.
  loss in long runs          WiFi roaming, or the bridge falling behind.
                             Check RSSI on the XIAO's [status] line.
  rate ~125/s, loss ~50%     the bridge is handling one line per loop() and
                             loop() is running at half the needed rate.
  0 lines, but '#' answers   commands work, telemetry does not: the Teensy is
                             in t0 (USB) mode. Send t1.
  0 lines, nothing at all    go back to Stage 3; the WiFi half is not up.

CROSS-CHECK IT against the Teensy's own USB [faker] line:

    lines the Teensy SENT (lines_out)  vs  lines this script RECEIVED

  equal              -> perfect
  received is lower  -> lost after the Teensy: XIAO relay or the air link
  tx_skipped nonzero -> lost BEFORE the XIAO: Serial1 TX buffer full, i.e.
                        the Teensy could not even hand the line over
"""

import argparse
import socket
import sys
import time

DEFAULT_IP = "172.20.10.14"
DEFAULT_PORT = 4210
NUM_COLS = 21
RECV = 4096
HEARTBEAT_S = 0.1        # the firmware watchdog is 300 ms


def send(sock, payload, addr):
    """The socket is non-blocking during the measurement, so a full kernel
    send buffer raises instead of waiting. Dropping one keepalive is harmless
    (they go out at 10 Hz against a 300 ms watchdog); crashing the run 40 s in
    is not."""
    try:
        sock.sendto(payload, addr)
    except OSError:
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--ip", default=DEFAULT_IP)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--expect-hz", type=float, default=250.0,
                   help="250 for the corner faker, 500 for edge")
    p.add_argument("--command", default=None,
                   help="send this command once after 2 s (e.g. a1, c, g0.5) "
                        "to prove the laptop->Teensy direction")
    args = p.parse_args()

    addr = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", args.port))
    except OSError:
        sock.bind(("0.0.0.0", 0))
        print(f"  [warn] UDP {args.port} busy -- using {sock.getsockname()[1]}")

    print(f"Stage 4 rate check -> XIAO {args.ip}:{args.port}, "
          f"expecting {args.expect_hz:.0f} lines/s of {NUM_COLS}-field CSV")
    print("Cube power OFF; both boards on their own USB cable.")
    print("The XIAO only learns this laptop's address once it hears from us,")
    print("so nothing arrives until the first heartbeat lands.\n")

    # x1 puts the bridge in TEENSY_BRIDGE mode; the XIAO eats these, they never
    # reach the Teensy. Sent three times -- UDP has no delivery guarantee and a
    # single dropped mode switch looks exactly like a dead link.
    for _ in range(3):
        sock.sendto(b"x1\n", addr)
        time.sleep(0.1)

    start = time.monotonic()
    end = start + args.seconds
    next_beat = start
    next_report = start + 1.0
    sent_command = args.command is None

    seqs = []            # every seq seen, in arrival order
    n_lines = n_console = n_bad = 0
    first_seq = last_seq = None
    out_of_order = 0
    window_lines = 0

    sock.setblocking(False)
    while time.monotonic() < end:
        now = time.monotonic()

        if now >= next_beat:
            send(sock, b"h\n", addr)           # keepalive: feeds the 300 ms watchdog
            next_beat += HEARTBEAT_S

        if not sent_command and now - start > 2.0:
            print(f"  -> sending {args.command!r}")
            send(sock, f"{args.command}\n".encode(), addr)
            sent_command = True

        try:
            raw, _ = sock.recvfrom(RECV)
        except (BlockingIOError, socket.timeout, OSError):
            time.sleep(0.0005)
        else:
            for line in raw.decode("ascii", errors="replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    n_console += 1
                    print(f"  [teensy] {line}")
                    continue
                parts = line.split(",")
                if len(parts) != NUM_COLS:
                    n_bad += 1
                    if n_bad <= 3:
                        print(f"  [warn] {len(parts)} fields, expected {NUM_COLS}: "
                              f"{line[:70]}")
                        if "\t" in line:
                            print("         tab-delimited -> firmware is in "
                                  "SERIALMONITORMODE, not PLOTMODE")
                    continue
                try:
                    seq = int(round(float(parts[0]) / 4.0))
                except ValueError:
                    n_bad += 1
                    continue
                n_lines += 1
                window_lines += 1
                if first_seq is None:
                    first_seq = seq
                if last_seq is not None and seq < last_seq:
                    out_of_order += 1
                last_seq = seq
                seqs.append(seq)

        if now >= next_report:
            print(f"  t={now - start:5.1f}s  {window_lines:4d} lines in the "
                  f"last second")
            window_lines = 0
            next_report += 1.0

    sock.setblocking(True)
    sock.close()

    span = time.monotonic() - start
    print("\n" + "-" * 60)
    if not n_lines:
        print("  NO telemetry at all.")
        if n_console:
            print(f"  But {n_console} '#' console lines DID arrive, so the")
            print("  command path works and the transport is fine. The Teensy")
            print("  is not streaming: it is halted (send p0) or in USB link")
            print("  mode (send t1).")
        else:
            print("  And no console lines either. Re-run Stage 3 -- if that")
            print("  passes, the fault is Serial1: re-run Stage 1.")
        return 1

    rate = n_lines / span
    expected = (last_seq - first_seq + 1) if last_seq is not None else 0
    lost = max(0, expected - n_lines)
    loss_pct = 100.0 * lost / expected if expected else 0.0

    # Gap histogram: one long gap and a hundred single drops are very
    # different faults and must not average into the same number.
    gaps = {}
    longest = 0
    ordered = sorted(set(seqs))
    for a, b in zip(ordered, ordered[1:]):
        d = b - a - 1
        if d > 0:
            gaps[d] = gaps.get(d, 0) + 1
            longest = max(longest, d)

    print(f"  received   {n_lines} lines in {span:.1f}s = {rate:.1f}/s "
          f"(expected {args.expect_hz:.0f}/s)")
    print(f"  seq range  {first_seq} .. {last_seq}  ({expected} emitted)")
    print(f"  lost       {lost} = {loss_pct:.2f}%")
    print(f"  longest gap {longest} consecutive lines")
    print(f"  out-of-order {out_of_order}   malformed {n_bad}   "
          f"console '#' {n_console}")
    if gaps:
        top = sorted(gaps.items(), key=lambda kv: -kv[1])[:4]
        print("  gap sizes  " + ", ".join(f"{k} line(s) x{v}" for k, v in top))

    print("-" * 60)
    ok = True
    if rate < args.expect_hz * 0.5:
        ok = False
        print("  FAIL: less than half the expected rate arrives.")
        if rate < 5:
            print("  A rate near 0.5/s is the classic signature of the XIAO's")
            print("  USB cable being attached with nothing reading the port.")
            print("  Unplug it (power the XIAO from the rail) or keep a serial")
            print("  monitor open and draining.")
        else:
            print("  The bridge cannot keep up. Check RSSI, and check the")
            print("  Teensy's tx_skipped counter to see which side is losing.")
    elif loss_pct > 5:
        ok = False
        print("  FAIL: too much loss to trust a plot. Check RSSI and distance.")
    elif loss_pct > 1:
        print("  MARGINAL: usable, but tighten the RF path if you can.")
    else:
        print("  PASS: the pipeline carries the real data rate essentially")
        print("  intact. The transport is proven -- go to Stage 5 (rail power).")

    if args.command and not n_console:
        ok = False
        print(f"  NOTE: {args.command!r} produced no '#' answer. The")
        print("  laptop -> Teensy direction is NOT proven. Check the XIAO D6 ->")
        print("  Teensy pin 0 wire (Stage 1 tests exactly this).")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
