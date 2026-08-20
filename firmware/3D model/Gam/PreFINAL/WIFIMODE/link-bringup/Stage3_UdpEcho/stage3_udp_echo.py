"""stage3_udp_echo.py -- laptop side of the Stage 3 UDP round-trip test.

Pairs with Stage3_UdpEcho/xiao_udp_echo/xiao_udp_echo.ino. No Teensy, no
wires, no matplotlib -- a plain socket. If this passes, the WiFi half of the
link is proven and every later failure is on the Teensy/Serial1 side.

    python stage3_udp_echo.py
    python stage3_udp_echo.py --ip 192.168.1.42
    python stage3_udp_echo.py --rate 250 --seconds 20     # sustained load

WHAT IT MEASURES

  1. RTT       50 pings at 10 Hz. min / median / p95 / max, and loss.
  2. BEAT      unprompted packets from the XIAO. These prove the XIAO->laptop
               direction even when your request is what went missing.
  3. LOAD      a sustained burst at --rate. The corner build emits 250
               lines/s and the edge build 500/s; a link that answers one ping
               nicely and collapses under load will pass every casual test and
               still show you a dead plot window.

READING IT AGAINST THE XIAO'S OWN [status] LINE -- do this, it is the point:

  | script says     | XIAO's udp_in  | verdict                                |
  |-----------------|----------------|----------------------------------------|
  | nothing back    | 0/s            | your packets never arrive: CLIENT       |
  |                 |                | ISOLATION on the AP, wrong IP, or the   |
  |                 |                | board is on another network             |
  | nothing back    | climbing       | laptop-side inbound block: Windows      |
  |                 |                | Firewall (python.exe), or a VPN         |
  | echoes arrive   | climbing       | WiFi half is healthy -> go to Stage 4   |
  | echoes trickle  | climbing       | the XIAO's loop is stalled -- usually   |
  | ~2 s apart      |                | its USB cable attached with no monitor  |
  |                 |                | reading it (see ../../README.md)        |

Windows will pop a firewall prompt the first time this binds. Allow it on
PRIVATE networks. A phone hotspot is often classified Public -- if you clicked
the wrong box once, no prompt appears again and everything silently fails:
  Windows Security > Firewall & network protection > Allow an app...
"""

import argparse
import socket
import statistics
import sys
import threading
import time

DEFAULT_IP = "172.20.10.14"     # must match XIAO_IP in the sketch
DEFAULT_PORT = 4210
RECV = 2048

OK, BAD, WARN, INFO = "  ok  ", " FAIL ", " warn ", " info "


def say(mark, msg):
    print(f"  [{mark}] {msg}")


def head(title):
    print(f"\n--- {title} " + "-" * max(0, 56 - len(title)))


def make_socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", port))
    except OSError:
        s.bind(("0.0.0.0", 0))
        say(WARN, f"UDP {port} busy (telemetry script still open?) -- "
                  f"using {s.getsockname()[1]}")
    return s


def rtt_test(sock, addr, count=50, hz=10.0):
    """Send `count` pings at `hz` and match every echo back to its own token.

    Tokens rather than a per-ping deadline, because a reply that arrives
    after the next ping went out is LATE, not LOST, and the difference
    matters enormously: late points at the ESP32's WiFi modem sleep (fixed in
    firmware with WiFi.setSleep(false)), lost points at RF or a firewall. A
    deadline-per-ping loop cannot tell them apart and reports both as loss."""
    head(f"1. Round trip: {count} pings at {hz:.0f} Hz")
    period = 1.0 / hz
    window_ms = period * 1000.0

    outstanding = {}        # token -> send time
    rtts, beats, late = [], 0, 0

    def collect(until):
        nonlocal beats, late
        while time.monotonic() < until:
            sock.settimeout(max(0.002, until - time.monotonic()))
            try:
                raw, _ = sock.recvfrom(RECV)
            except (socket.timeout, OSError):
                return
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("BEAT:"):
                beats += 1
                continue
            if line.startswith("ECHO:"):
                token = line[5:]
                t0 = outstanding.pop(token, None)
                if t0 is not None:
                    ms = (time.monotonic() - t0) * 1000.0
                    rtts.append(ms)
                    if ms > window_ms:
                        late += 1

    for i in range(count):
        token = f"P{i}"
        outstanding[token] = time.monotonic()
        sock.sendto(f"{token}\n".encode(), addr)
        collect(time.monotonic() + period)

    collect(time.monotonic() + 0.5)      # let stragglers in before judging

    lost = len(outstanding)
    if not rtts:
        say(BAD, f"0 of {count} pings came back")
        say(INFO, "check the XIAO's [status] line NOW: is udp_in climbing?")
        say(INFO, "  udp_in = 0  -> your packets never reach it (AP isolation)")
        say(INFO, "  udp_in > 0  -> replies are blocked coming back (firewall)")
        return False, beats

    loss_pct = 100.0 * lost / count
    srt = sorted(rtts)
    p95 = srt[min(len(srt) - 1, int(0.95 * len(srt)))]
    med = statistics.median(rtts)
    print(f"        rtt  min={min(rtts):.1f}  med={med:.1f}  "
          f"p95={p95:.1f}  max={max(rtts):.1f} ms")
    print(f"        lost {lost}/{count} = {loss_pct:.1f}%"
          + (f", plus {late} late (over {window_ms:.0f} ms)" if late else ""))

    if loss_pct > 20:
        say(BAD, "heavy loss -- weak signal, or the AP is dropping us")
        return False, beats

    # The signature of WiFi modem sleep: a fast best case (that is the real RF
    # cost) with a median an order of magnitude worse and a hard ceiling at
    # the AP's beacon interval. Worth naming explicitly, because the obvious
    # readings -- weak signal, congestion -- are both wrong, and the fix is
    # one line of firmware.
    if med > 25 and min(rtts) < med / 5:
        say(WARN, f"min {min(rtts):.1f} ms but median {med:.1f} ms. That gap "
                  f"is WIFI MODEM SLEEP on the ESP32, not the network: the "
                  f"radio parks between the AP's beacons. Fix: WiFi.setSleep"
                  f"(false) -- already applied in this folder's sketches and "
                  f"in xiao_teensy_bridge.ino. REFLASH the XIAO and re-run.")
        return True, beats
    if med > 200:
        say(WARN, "median RTT over 200 ms. Something is stalling the XIAO's "
                  "loop -- most often its USB cable attached with no monitor "
                  "reading it. Commands will feel laggy and telemetry will "
                  "be dropped in bulk at Stage 4.")
        return True, beats
    if loss_pct > 2:
        say(WARN, f"{loss_pct:.1f}% loss. Usable for commands, marginal for "
                  f"telemetry. Move closer to the AP.")
        return True, beats
    say(OK, "round trip works, loss and latency are healthy")
    return True, beats


def beat_test(sock, seconds=3.0):
    """The XIAO sends BEAT: unprompted every 500 ms. Receiving these proves
    XIAO->laptop on its own, without us having asked for anything."""
    head(f"2. Unprompted XIAO -> laptop traffic ({seconds:.0f} s)")
    end = time.monotonic() + seconds
    beats = []
    while time.monotonic() < end:
        sock.settimeout(max(0.05, end - time.monotonic()))
        try:
            raw, _ = sock.recvfrom(RECV)
        except (socket.timeout, OSError):
            break
        line = raw.decode("ascii", errors="replace").strip()
        if line.startswith("BEAT:"):
            beats.append(line)

    expected = int(seconds / 0.5)
    print(f"        {len(beats)} BEAT packets (expected about {expected})")
    if not beats:
        say(BAD, "none. Either the XIAO never heard us (so it does not know "
                 "where to send), or its packets are blocked inbound here.")
        return False
    if len(beats) < expected * 0.6:
        say(WARN, "fewer than expected -- lossy link")
        return True
    say(OK, "the XIAO reaches this laptop unprompted")
    return True


def load_test(sock, addr, rate, seconds):
    """Sustained load. This is the number that predicts whether telemetry will
    actually flow: the bridge handles ONE packet per loop() iteration, so the
    rate at which it can echo is the rate at which it can relay."""
    head(f"3. Sustained load: {rate}/s for {seconds:.0f} s")
    while True:                      # flush
        sock.settimeout(0.2)
        try:
            sock.recvfrom(RECV)
        except (socket.timeout, OSError):
            break

    # Send from a dedicated thread. Interleaving sends with non-blocking
    # receives on one thread caps this script at ~238/s on Windows -- the
    # sleep granularity and the per-iteration BlockingIOError cost more than
    # the 4 ms period. That is a LAPTOP limit, and it used to read as though
    # the link could not carry 250/s when it was delivering 99.8% of
    # everything it was given.
    n_sent = [0]
    stop = threading.Event()
    start = time.monotonic()
    end = start + seconds

    def sender():
        period_s = 1.0 / rate
        next_t = time.monotonic()
        while not stop.is_set():
            now = time.monotonic()
            if now >= end:
                return
            if now >= next_t:
                try:
                    sock.sendto(f"L{n_sent[0]}\n".encode(), addr)
                    n_sent[0] += 1
                except OSError:
                    pass
                next_t += period_s
                # If we fall behind (a scheduling hiccup), resynchronise
                # instead of emitting a catch-up burst -- a burst would
                # measure the AP's buffer, not the steady-state link.
                if next_t < now:
                    next_t = now + period_s
            else:
                # Sleep only the coarse part; spin the last millisecond,
                # because Windows' sleep resolution is the whole problem.
                slack = next_t - now
                if slack > 0.002:
                    time.sleep(slack - 0.001)

    tx = threading.Thread(target=sender, daemon=True)
    tx.start()

    got = 0
    sock.setblocking(True)
    while time.monotonic() < end:
        sock.settimeout(max(0.01, end - time.monotonic()))
        try:
            raw, _ = sock.recvfrom(RECV)
        except (socket.timeout, OSError):
            continue
        if raw.decode("ascii", errors="replace").startswith("ECHO:"):
            got += 1

    stop.set()
    tx.join(timeout=1.0)

    drain_end = time.monotonic() + 0.5          # late stragglers
    while time.monotonic() < drain_end:
        sock.settimeout(max(0.01, drain_end - time.monotonic()))
        try:
            raw, _ = sock.recvfrom(RECV)
        except (socket.timeout, OSError):
            break
        if raw.decode("ascii", errors="replace").startswith("ECHO:"):
            got += 1

    # Rates are per second OF TRAFFIC, so the denominator is the send window,
    # not the wall clock. Including the 0.5 s straggler drain here understated
    # both rates by ~5% and made a link sending a true 249.6/s report 238/s --
    # which then tripped the "cannot keep up" warning on a healthy link.
    span = seconds
    sent = n_sent[0]
    in_rate = sent / span
    out_rate = got / span
    delivered = 100.0 * got / sent if sent else 0.0
    print(f"        sent {sent} ({in_rate:.0f}/s), echoed back {got} "
          f"({out_rate:.0f}/s) = {delivered:.1f}% delivered")

    # Judge the LINK on what fraction of our packets it carried, not on an
    # absolute rate this script may have failed to generate. Those are
    # different questions and only the first one is about the hardware.
    if delivered < 90.0:
        say(BAD, f"only {delivered:.1f}% of what we sent came back. The link "
                 f"is dropping traffic under load: check RSSI, and check the "
                 f"XIAO's USB is not attached with nothing reading it. Fix "
                 f"this before believing anything you see at Stage 4.")
        return False
    if delivered < 99.0:
        # Say what the number actually costs, rather than "move closer".
        # A few percent is invisible in a 20 fps plot of a 250 Hz stream --
        # but a single-shot command is a single packet, and that is the part
        # worth knowing before you are holding a balancing cube.
        one_in = int(round(100.0 / max(0.01, 100.0 - delivered)))
        say(WARN, f"{delivered:.1f}% delivered. Invisible in the plot (a "
                  f"20 fps redraw of a 250 Hz stream will not show it), but "
                  f"roughly 1 packet in {one_in} is lost -- so a single-shot "
                  f"command like a0 can need sending twice. Send safety "
                  f"commands until you SEE the armed column change.")
        return True

    if in_rate < rate * 0.95:
        say(OK, f"{delivered:.1f}% delivered -- the link carried essentially "
                f"everything. It only reached {in_rate:.0f}/s because THIS "
                f"SCRIPT could not send faster, not because the link could "
                f"not carry it. Stage 4 settles the real headroom question, "
                f"with the Teensy as the source at a true {rate}/s.")
        return True
    say(OK, f"{out_rate:.0f} round trips/s at {delivered:.1f}% delivered -- "
            f"enough for the corner build (250 lines/s). Edge needs 500/s; "
            f"re-run with --rate 500 if that is the build you are flashing.")
    return True


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--ip", default=DEFAULT_IP)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--rate", type=int, default=250,
                   help="load-test packets/s (250 corner, 500 edge)")
    p.add_argument("--seconds", type=float, default=10.0,
                   help="load-test duration")
    p.add_argument("--pings", type=int, default=50)
    args = p.parse_args()

    addr = (args.ip, args.port)
    print(f"Stage 3 UDP echo test -> XIAO {args.ip}:{args.port}")
    print("Cube power OFF. Keep the XIAO's Serial Monitor open next to this")
    print("window and compare its udp_in counter against what you read here.")

    sock = make_socket(args.port)
    try:
        ok_rtt, _ = rtt_test(sock, addr, args.pings)
        ok_beat = beat_test(sock)
        ok_load = load_test(sock, addr, args.rate, args.seconds) if ok_rtt else False
    finally:
        sock.close()

    head("Verdict")
    if ok_rtt and ok_beat and ok_load:
        print("  WiFi half is PROVEN, in both directions, under load.")
        print("  Go to Stage 4 -- everything from here on is Teensy/Serial1.")
        return 0
    if not ok_rtt and not ok_beat:
        print("  Nothing came back at all. Look at the XIAO's [status] line:")
        print("    udp_in = 0/s   -> laptop -> XIAO is blocked (AP client")
        print("                      isolation, wrong IP, wrong network)")
        print("    udp_in > 0/s   -> XIAO -> laptop is blocked (this laptop's")
        print("                      firewall, or a VPN)")
        print("  Do not go on to Stage 4 -- it cannot pass.")
        return 1
    print("  Partial. The link works but not well enough to trust telemetry.")
    print("  Fix signal/latency here; Stage 4 will only be harder.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
