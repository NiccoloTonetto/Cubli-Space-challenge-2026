"""link_check.py  --  is the XIAO working, and if not, which hop is broken?

Run this INSTEAD of the telemetry script when nothing is arriving. It walks
the pipeline one hop at a time and names the broken one, rather than leaving
you to guess from a silent plot window.

    python link_check.py                      # network probe only
    python link_check.py --serial COM10       # + read the XIAO over USB  <-- best
    python link_check.py --list-ports
    python link_check.py --ip 192.168.1.42 --teensy edge

----------------------------------------------------------------------
USE --serial. IT IS THE ONLY CONCLUSIVE TEST.

Without it, a total silence is ambiguous: your packets might never reach
the XIAO, or its replies might never get back, and from the laptop those
look identical. With the XIAO's USB attached, this script reads its
once-a-second [status] line while sending a measured burst of UDP, and
watches the board's own `udp_in` counter. That distinguishes the two
directions definitively, and it keeps working on a network that blocks
device-to-device traffic entirely -- exactly when you most need an answer.

Wiring for --serial: cube power OFF (or the XIAO's 5V pin lifted off the
LM2596 rail), XIAO USB-C to the laptop, GND still common. Close the
Arduino Serial Monitor first -- it holds the port and this cannot share
it.

----------------------------------------------------------------------
THE STAGES

  1. LOCAL      Which interface would reach the XIAO, and is it on the
                same subnet? A static XIAO_IP left over from a different
                network is the most common cause of total silence.
  2. REACHABLE  ICMP ping + ARP. Separates "not on the network" from
                "there but blocked". ARP is cached, so it is a hint only.
  3. UDP RTT    Puts the XIAO in WIFI_TEST and does PING/PONG. Needs NO
                Teensy -- isolates the WiFi half completely.
  4. TEENSY     Bridge mode, then a re-resolve ('c' corner / 'e' edge),
                waiting for the '#' answer. Exercises the whole loop:
                laptop -> XIAO -> Serial1 -> Teensy -> Serial1 -> XIAO.
  5. USB        (--serial only) Correlates a UDP burst against the XIAO's
                own counters for a directional verdict.

----------------------------------------------------------------------
THE x0/x1 POLARITY

xiao_teensy_bridge.ino documents x0 = WIFI_TEST, x1 = TEENSY_BRIDGE. An
older revision had that condition reversed in code while the comments
said otherwise, which made PING unreachable via the documented command
(PING is only answered in WIFI_TEST). Rather than assume which revision
is flashed, stage 3 TRIES BOTH and reports which one your board honours.
A board answering on x1 is running the old firmware and should be
reflashed from this folder.

SAFETY: run with the motor bus DEAD. The Teensy boots and streams without
the moteus powered -- CAN3 initialises against the local peripheral, not
the bus. Nothing here needs a wheel able to spin.
"""

import argparse
import json
import re
import socket
import subprocess
import sys
import threading
import time

DEFAULT_XIAO_IP = "172.20.10.14"   # cubli1 hotspot; override with --ip
DEFAULT_PORT = 4210
STATUS_BAUD = 115200
RECV_BUF = 2048

PASS, FAIL, WARN, INFO = "PASS", "FAIL", "WARN", "INFO"
_MARK = {PASS: "  ok  ", FAIL: " FAIL ", WARN: " warn ", INFO: " info "}


def say(status, msg):
    print(f"  [{_MARK[status]}] {msg}")
    return status


def stage(n, title):
    print(f"\n─── {n}. {title} " + "─" * max(0, 52 - len(title)))


# ---------------------------------------------------------------------------
# The XIAO's [status] line
# ---------------------------------------------------------------------------

_STATUS_RE = {
    "wifi": re.compile(r"wifi=(\S+)"),
    "rssi": re.compile(r"rssi=(-?\d+)dBm"),
    "laptop": re.compile(r"laptop=(none yet|\S+)"),
    "serial1_lines": re.compile(r"serial1_lines=(\d+)"),
    "udp_in": re.compile(r"udp_in=(\d+)"),
    "udp_out": re.compile(r"udp_out=(\d+)"),
    "mode": re.compile(r"mode=(\S+)"),
}


def parse_status(line):
    """-> dict of the [status] fields, or None if this isn't one.

    The counters are PER-SECOND -- xiao_teensy_bridge.ino zeroes them after
    every print -- so they are rates, not running totals."""
    if "[status]" not in line:
        return None
    out = {}
    for key, rx in _STATUS_RE.items():
        m = rx.search(line)
        if m:
            out[key] = m.group(1)
    for k in ("serial1_lines", "udp_in", "udp_out"):
        if k in out:
            out[k] = int(out[k])
    return out or None


class XiaoUsbMonitor(threading.Thread):
    """Reads the XIAO's USB serial in the background, collecting [status]
    lines with the time each arrived so they can be correlated against a
    UDP burst sent from the main thread."""

    def __init__(self, port):
        super().__init__(daemon=True)
        import serial   # imported here so the script still runs without it
        self.ser = serial.Serial(port, STATUS_BAUD, timeout=0.4)
        self.samples = []      # (monotonic_time, dict)
        self.other = []        # non-status lines (boot banner etc.)
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            st = parse_status(line)
            if st:
                self.samples.append((time.monotonic(), st))
            else:
                self.other.append(line)

    def close(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass

    def since(self, t0):
        return [s for (t, s) in self.samples if t >= t0]


# ---------------------------------------------------------------------------
# Stage 1: local interface + subnet
# ---------------------------------------------------------------------------

def outbound_ip_for(dest):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((dest, 1))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def check_local(xiao_ip):
    local = outbound_ip_for(xiao_ip)
    if not local:
        return say(FAIL, f"no route to {xiao_ip} — is Wi-Fi connected?")
    say(PASS, f"this laptop would use {local} to reach {xiao_ip}")
    if local.startswith("169.254."):
        return say(FAIL, f"{local} is a link-local (APIPA) address — the "
                          "Wi-Fi adapter never got a DHCP lease")
    if local.split(".")[:3] != xiao_ip.split(".")[:3]:
        return say(FAIL,
                   f"{local} and {xiao_ip} are on DIFFERENT subnets. The "
                   "XIAO's static IP is left over from another network — set "
                   "XIAO_IP/XIAO_GATEWAY in xiao_teensy_bridge.ino to "
                   f"{'.'.join(local.split('.')[:3])}.x and reflash.")
    return say(PASS, "laptop and XIAO are on the same subnet")


# ---------------------------------------------------------------------------
# Stage 2: reachability
# ---------------------------------------------------------------------------

def icmp_ping(ip):
    try:
        flag = "-n" if sys.platform == "win32" else "-c"
        wait = ["-w", "1000"] if sys.platform == "win32" else ["-W", "1"]
        r = subprocess.run(["ping", flag, "2", *wait, ip],
                           capture_output=True, text=True, timeout=12)
        return r.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return None


def arp_mac(ip):
    try:
        r = subprocess.run(["arp", "-a", ip], capture_output=True,
                           text=True, timeout=8)
        m = re.search(r"(([0-9a-fA-F]{2}[-:]){5}[0-9a-fA-F]{2})", r.stdout)
        return m.group(1) if m else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def check_reachable(xiao_ip):
    pinged, mac = icmp_ping(xiao_ip), arp_mac(xiao_ip)
    if pinged:
        say(PASS, f"ICMP ping to {xiao_ip} answered")
        if mac:
            say(PASS, f"ARP resolves to {mac}")
        return PASS
    if mac:
        say(WARN, f"ping did not answer, but ARP resolves to {mac}")
        say(WARN, "ARP entries are CACHED — this does not prove the link is "
                  "live now. Stage 3 (and stage 5) are the real tests.")
        return WARN
    say(FAIL, f"no ping and no ARP entry for {xiao_ip}")
    say(FAIL, "the XIAO is not powered, not joined to this network, on a "
              "different IP, or the AP blocks device-to-device traffic.")
    return FAIL


# ---------------------------------------------------------------------------
# Packet classification
# ---------------------------------------------------------------------------

def classify(line):
    if line.startswith("PONG:"):
        return "PONG (XIAO WIFI_TEST reply)"
    if line.startswith("#"):
        return "firmware console line (Teensy)"
    if line.startswith("{"):
        try:
            if json.loads(line).get("type") == "telemetry":
                return "XIAO synthetic telemetry (WIFI_TEST mode)"
        except ValueError:
            pass
        return "JSON"
    if "\t" in line:
        return (f"TAB-delimited, {len(line.split(chr(9)))} fields — firmware "
                f"is in SERIALMONITORMODE, not PLOTMODE")
    parts = line.split(",")
    if len(parts) > 1:
        try:
            [float(p) for p in parts]
            kind = {10: " = EDGE build", 21: " = CORNER build"}.get(len(parts), "")
            return f"CSV, {len(parts)} fields{kind}"
        except ValueError:
            return f"comma text, {len(parts)} fields"
    return "unrecognised"


def drain(sock, seconds, label, want=None, quiet=False):
    end = time.monotonic() + seconds
    lines, matched = [], False
    while time.monotonic() < end:
        sock.settimeout(max(0.05, end - time.monotonic()))
        try:
            raw, addr = sock.recvfrom(RECV_BUF)
        except (socket.timeout, OSError):
            break
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        lines.append(line)
        if not quiet and len(lines) <= 6:
            shown = line if len(line) <= 88 else line[:85] + "..."
            print(f"        <- {addr[0]}:{addr[1]}  {shown}")
            print(f"           ({classify(line)})")
        if want and want in line:
            matched = True
            break
    if not quiet:
        if len(lines) > 6:
            print(f"        ... and {len(lines) - 6} more packets")
        if not lines:
            print(f"        (nothing received in {seconds:.0f}s — {label})")
    return lines, matched


# ---------------------------------------------------------------------------
# Stage 3: UDP round trip, polarity-agnostic
# ---------------------------------------------------------------------------

def check_udp(sock, addr):
    """Returns (status, polarity) where polarity is 'x0' (current firmware),
    'x1' (old inverted firmware) or None."""
    for cmd, note in (("x0", "current firmware"),
                      ("x1", "OLD firmware — x0/x1 inverted")):
        token = str(int(time.monotonic() * 1000) % 100000)
        say(INFO, f"trying {cmd} to enter WIFI_TEST ({note})")
        # Sent three times: UDP gives no delivery guarantee, and a single
        # dropped mode-switch leaves the XIAO in the wrong mode for the rest
        # of the run -- which looks like a link fault but isn't one.
        for _ in range(3):
            sock.sendto(f"{cmd}\n".encode(), addr)
            time.sleep(0.15)
        time.sleep(0.3)
        sock.sendto(f"PING:{token}\n".encode(), addr)
        lines, got = drain(sock, 2.5, "no UDP coming back",
                           want=f"PONG:{token}")
        if got:
            if cmd == "x1":
                say(WARN, "this board answers on x1 — it is running the OLD "
                          "bridge firmware. Reflash xiao_teensy_bridge.ino "
                          "from this folder.")
            return say(PASS, "PING/PONG round trip works — the WiFi half is "
                             "fine"), cmd
        if any(l.startswith("{") for l in lines):
            return say(PASS, "no PONG, but the XIAO's synthetic telemetry is "
                             "arriving — XIAO→laptop works"), cmd

    return say(FAIL,
               "nothing came back with either polarity. Your packets are not "
               "reaching the XIAO, or its replies are not reaching you — and "
               "from here those look identical. Re-run with --serial COMxx "
               "to tell them apart."), None


def check_rate(sock, addr, which):
    """Measure the bridge's effective loop() rate.

    pollUdpIn() and pollSerial1Out() each handle exactly ONE item per loop()
    iteration, so the rate at which the XIAO can answer a burst of pings IS
    the rate at which it can forward Teensy telemetry lines. That makes this
    the number that predicts whether the link will work at all -- a bridge
    that answers a ping fine but iterates at 0.5 Hz will still drop 499 of
    every 500 telemetry lines."""
    needed = 250 if which == "corner" else 500

    while True:                      # flush anything queued
        sock.settimeout(0.2)
        try:
            sock.recvfrom(RECV_BUF)
        except (socket.timeout, OSError):
            break

    n = 15
    for i in range(n):
        sock.sendto(f"PING:{i}\n".encode(), addr)
        time.sleep(0.03)
    sent_done = time.monotonic()

    got, last = 0, None
    end = time.monotonic() + 20.0
    while time.monotonic() < end and got < n:
        sock.settimeout(max(0.05, end - time.monotonic()))
        try:
            raw, _ = sock.recvfrom(RECV_BUF)
        except (socket.timeout, OSError):
            break
        if raw.decode("ascii", errors="replace").strip().startswith("PONG"):
            got += 1
            last = time.monotonic()

    if got == 0:
        return say(FAIL, "no replies to the burst — cannot measure throughput")

    span = max(last - sent_done, 1e-3)
    rate = got / span
    print(f"        {got}/{n} replies in {span:.2f}s")

    if got == n and span < 2.0:
        return say(PASS, f"bridge keeps up (>{rate:.0f} lines/s available, "
                         f"{which} needs {needed}/s)")

    return say(FAIL,
               f"BRIDGE IS DEGRADED — about {rate:.1f} lines/s, but the "
               f"{which} build emits {needed}/s. Telemetry will be dropped "
               f"almost entirely. Usual causes: the Teensy is flooding "
               f"Serial1 faster than the bridge can relay, or the XIAO's USB "
               f"is attached with nothing reading it (see README).")


def check_teensy(sock, addr, which, polarity):
    bridge = "x1" if polarity == "x0" else "x0"
    say(INFO, f"switching XIAO back to bridge mode ({bridge})")
    for _ in range(3):          # see the note in check_udp on repeating this
        sock.sendto(f"{bridge}\n".encode(), addr)
        time.sleep(0.15)
    time.sleep(0.3)

    cmd = "c" if which == "corner" else "e"
    print(f"        -> sending {cmd!r} (re-resolve; harmless, arms nothing)")
    sock.sendto(f"{cmd}\n".encode(), addr)

    lines, _ = drain(sock, 3.0, "the Teensy is not answering")

    # The XIAO's own synthetic telemetry is NOT a Teensy reply. It proves the
    # opposite: the board is still in WIFI_TEST, so the mode switch never
    # landed. Counting it as "a reply" is what made this stage report success
    # against an unpowered Teensy.
    # Both the synthetic JSON telemetry and a PONG come from the XIAO itself,
    # not the Teensy. A PONG in particular can be a LATE reply to stage 3's
    # ping arriving inside this window -- the hotspot's latency is easily a
    # few seconds -- so counting either as "the Teensy answered" reports
    # success against a board that is not even powered.
    synthetic = [l for l in lines if l.startswith("{")]
    from_teensy = [l for l in lines
                   if not l.startswith("{") and not l.startswith("PONG:")]

    if synthetic and not from_teensy:
        return say(FAIL,
                   "only the XIAO's own synthetic telemetry came back, so it "
                   f"is still in WIFI_TEST and the {bridge} switch was "
                   "dropped. Re-run; if it persists, the hotspot is losing "
                   "packets.")

    if not from_teensy:
        return say(FAIL,
                   "no reply from the Teensy. If the Teensy is UNPOWERED this "
                   "is expected — power the logic rail (motor bus still dead) "
                   "and re-run. Otherwise: Serial1 wiring (TX1 pin 1→D7, "
                   "RX1 pin 0→D6, CROSSED), common GND, or it is halted (p0).")

    if any(l.startswith("#") for l in from_teensy):
        say(PASS, "the Teensy answered — the full loop works")

    csv = [l for l in from_teensy if "," in l and not l.startswith("#")]
    if csv:
        n = len(csv[-1].split(","))
        expect = 21 if which == "corner" else 10
        if n == expect:
            return say(PASS, f"telemetry streaming, {n} fields as expected")
        return say(FAIL, f"telemetry is {n} fields but --teensy {which} "
                          f"expects {expect}. Wrong firmware flashed, or "
                          f"wrong --teensy argument.")
    if any("\t" in l for l in lines):
        return say(FAIL, "telemetry is TAB-delimited — firmware is in "
                          "SERIALMONITORMODE. Reflash with PLOTMODE.")
    return say(WARN, "reply received but no telemetry stream — is the board "
                      "halted? Send p0 (corner) and retry.")


# ---------------------------------------------------------------------------
# Stage 5: correlate a UDP burst against the XIAO's own counters
# ---------------------------------------------------------------------------

def check_usb(mon, sock, addr, burst_s=4.0, rate_hz=10):
    if not mon.samples and not mon.other:
        return say(FAIL, "nothing at all on the serial port. Wrong COM port, "
                         "the XIAO is not running, or 'USB CDC On Boot' is "
                         "disabled in the board settings."), None

    if not mon.samples:
        say(WARN, "serial is alive but no [status] lines yet — showing what "
                  "did arrive:")
        for line in mon.other[-6:]:
            print(f"        | {line}")
        return WARN, None

    last = mon.samples[-1][1]
    say(INFO, f"XIAO reports: wifi={last.get('wifi')} "
              f"rssi={last.get('rssi')}dBm mode={last.get('mode')} "
              f"laptop={last.get('laptop')}")

    if last.get("wifi") != "up":
        return say(FAIL, "the XIAO is NOT on the network (wifi is down). "
                         "Credentials, or the network needs a captive-portal "
                         "login."), None
    say(PASS, "the XIAO is joined to the network")

    n = int(burst_s * rate_hz)
    say(INFO, f"sending {n} packets over {burst_s:.0f}s and watching the "
              f"XIAO's udp_in counter...")
    t0 = time.monotonic()
    for i in range(n):
        sock.sendto(f"PING:{i}\n".encode(), addr)
        time.sleep(1.0 / rate_hz)
    got, _ = drain(sock, 1.5, "no replies", quiet=True)
    time.sleep(1.2)          # let one more [status] line land

    during = mon.since(t0)
    if not during:
        return say(WARN, "no [status] lines arrived during the burst — "
                         "cannot correlate"), None

    peak_in = max(s.get("udp_in", 0) for s in during)
    peak_out = max(s.get("udp_out", 0) for s in during)
    peak_ser = max(s.get("serial1_lines", 0) for s in during)

    print(f"        peak udp_in={peak_in}/s  udp_out={peak_out}/s  "
          f"serial1_lines={peak_ser}/s   (we sent {rate_hz}/s)")

    if peak_ser:
        say(PASS, f"the Teensy is talking to the XIAO ({peak_ser} lines/s)")
    else:
        say(INFO, "no Serial1 traffic — expected if the Teensy is unpowered")

    if peak_in == 0:
        return say(FAIL,
                   "the XIAO received NOTHING while we sent "
                   f"{rate_hz} packets/s. Your packets are not crossing the "
                   "network. On a guest/campus AP this is CLIENT ISOLATION: "
                   "both devices reach the internet but not each other. No "
                   "firmware change can fix it — use a phone hotspot."), "out"

    say(PASS, f"the XIAO IS receiving our packets ({peak_in}/s) — the "
              f"laptop→XIAO direction works")

    if got:
        return say(PASS, "and replies are getting back — link is healthy"), None
    return say(FAIL,
               "but nothing comes back. The XIAO→laptop direction is "
               "blocked: check Windows Firewall inbound UDP for python.exe, "
               "and that no VPN is capturing the route."), "back"


# ---------------------------------------------------------------------------

def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("pyserial is not installed:  pip install pyserial")
        return 1
    ports = list(lp.comports())
    if not ports:
        print("No serial ports found — nothing is plugged in by USB.")
        return 1
    print("Serial ports:")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")
    print("\nThe XIAO appears as a generic 'USB Serial Device' once "
          "'USB CDC On Boot' is enabled.")
    return 0


def main():
    p = argparse.ArgumentParser(description="Diagnose the XIAO/Teensy link.")
    p.add_argument("--ip", default=DEFAULT_XIAO_IP, help="XIAO static IP")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--serial", metavar="COMx", default=None,
                   help="XIAO's USB serial port — enables the conclusive "
                        "directional test. Close the Arduino Serial Monitor "
                        "first.")
    p.add_argument("--list-ports", action="store_true")
    p.add_argument("--teensy", choices=["corner", "edge"], default="corner")
    args = p.parse_args()

    if args.list_ports:
        return list_ports()

    print(f"Link check → XIAO {args.ip}:{args.port}, "
          f"expecting the {args.teensy} build")
    print("Motor bus should be DEAD for this.")
    if not args.serial:
        print("TIP: add --serial COMxx for a conclusive answer. Without it, "
              "'no packets' cannot\n     be attributed to a direction.")

    mon = None
    if args.serial:
        try:
            mon = XiaoUsbMonitor(args.serial)
            mon.start()
            print(f"Reading the XIAO on {args.serial} at {STATUS_BAUD}.")
            time.sleep(2.5)      # collect a baseline
        except ImportError:
            print("pyserial is not installed:  pip install pyserial")
            return 1
        except Exception as e:
            print(f"\nCannot open {args.serial}: {e}")
            print("The Arduino Serial Monitor is probably holding it. Close "
                  "it, or run --list-ports to find the right one.")
            return 1

    try:
        stage(1, "Local interface and subnet")
        if check_local(args.ip) == FAIL:
            print("\nStop here — fix the addressing before anything else.")
            return 1

        stage(2, "Is the XIAO reachable?")
        check_reachable(args.ip)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", args.port))
        except OSError:
            sock.bind(("0.0.0.0", 0))
            say(WARN, f"UDP {args.port} is in use (telemetry script still "
                      f"running) — using {sock.getsockname()[1]} instead")

        try:
            stage(3, "UDP round trip (no Teensy involved)")
            udp, polarity = check_udp(sock, (args.ip, args.port))

            rate = None
            if udp != FAIL:
                stage("3b", "Bridge throughput (the number that matters)")
                rate = check_rate(sock, (args.ip, args.port), args.teensy)

            stage(4, "Full loop through the Teensy")
            if udp == FAIL:
                say(WARN, "skipping — stage 3 failed, this cannot pass")
                teensy = FAIL
            else:
                teensy = check_teensy(sock, (args.ip, args.port),
                                      args.teensy, polarity)

            direction = None
            if mon:
                stage(5, "XIAO's own counters over USB (conclusive)")
                _, direction = check_usb(mon, sock, (args.ip, args.port))
        finally:
            sock.close()
    finally:
        if mon:
            mon.close()

    stage("=", "Verdict")
    if direction == "out":
        print("  The XIAO is healthy and on the network. Your packets are")
        print("  not reaching it. CLIENT ISOLATION on this access point.")
        print("  Fix: phone hotspot, or any network you control.")
    elif direction == "back":
        print("  The XIAO is healthy and hears you, but its replies are not")
        print("  arriving. Laptop-side: firewall inbound UDP, or a VPN.")
    elif udp == FAIL:
        print("  No UDP completes, and without --serial the direction cannot")
        print("  be determined. Re-run with --serial COMxx.")
    elif rate == FAIL:
        print("  The link is up but the BRIDGE CANNOT KEEP UP. Commands will")
        print("  work and telemetry will not — most lines are dropped before")
        print("  they ever leave the XIAO. Fix the throughput before reading")
        print("  anything into the telemetry you do receive.")
    elif teensy == FAIL:
        print("  The WiFi half is HEALTHY — laptop <-> XIAO works both ways.")
        print("  The Teensy did not answer. If it is unpowered that is")
        print("  EXPECTED: power the logic rail (motor bus still dead) and")
        print("  re-run to test the full loop. Otherwise check Serial1 wiring")
        print("  (crossed TX/RX, common GND), Teensy power, or a halt (p0).")
    elif teensy == WARN:
        print("  WiFi is healthy and the Teensy replied, but it is not")
        print("  streaming telemetry. Check that it is not halted (p0).")
    else:
        print("  Link is healthy end to end.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
