"""telemetry_python_wifi_corner.py  --  CORNER build (21 columns)

Live viewer / logger / command console for
CornerBalance_WiFi/CornerBalance_WiFi.ino + xiao_teensy_bridge/
xiao_teensy_bridge.ino. For the EDGE build use telemetry_python_wifi.py
instead -- that one is 10 columns and the two formats are not
interchangeable.

Wire format (PLOTMODE, comma-separated, one line per emitted cycle):

    t_ms,
    phi_x_deg, phi_y_deg, phi_z_deg,
    om_x_dps, om_y_dps, om_z_dps,
    rho_x, rho_y, rho_z,
    rho_x_lp, rho_y_lp, rho_z_lp,
    tau_x, tau_y, tau_z,
    tau_cmd_x, tau_cmd_y, tau_cmd_z,
    armed, gain_scale

= 21 fields, identical in content and order to the USB build's
tab-delimited header. The firmware decimates telemetry to 250 Hz (the
control loop still runs at 500 Hz) because 21 fields at 500 Hz would
overrun the 1 Mbaud Serial1 link -- see CornerBalance_WiFi.ino's
TELEMETRY RATE note.

'#'-PREFIXED LINES are firmware console output (command echoes, the
z1 tare readback, arm refusals, boot diagnostics). They are printed to
this terminal rather than parsed, so nothing the firmware says is lost
in PLOTMODE and the CSV stays clean. This is a deliberate difference
from the 10-column edge script, which has no such handling.

FOUR PANELS, all sharing a scrolling time axis:
    1. Tilt      -- phi_x / phi_y / phi_z, plus |phi| with the 0.5 deg
                    arm gate drawn as a dashed reference line. |phi| is
                    the number the firmware's arm gate actually tests,
                    so this is the panel to watch before sending a1.
    2. Body rate -- om_x / om_y / om_z
    3. Wheels    -- rho_x / rho_y / rho_z solid, their 5 s low-pass
                    dashed. The low-pass settling toward zero is the
                    "wheels are unwinding" health check.
    4. Torque    -- tau_x/y/z solid, tau_cmd_x/y/z dashed. They diverge
                    exactly when disarmed or saturated.

COMMANDS -- type into the terminal you launched this from, press Enter:
    a1 / a0     arm / disarm   (CORNER MODE HAS NO ARM GATE -- a1 arms at
                                any tilt. Place the cube near the resolved
                                corner first; nothing in the firmware will.)
    g<0..1>     gain scale
    c           re-resolve the corner candidate
    h1 / h0     HALT / resume  <-- 'h' IS HALT: see below
    d<N>        telemetry decimation for this mode (d2 = 250 Hz)
    l0 / l1     Teensy link mode (USB / WiFi)
    m0 / m1     cube control law (EDGE / CORNER). This plot window only
                parses the 26-field corner format, so m0 makes the stream
                go quiet here -- that is the edge build talking, not a
                fault. Use telemetry_python_wifi.py or the dashboard.
    x0 / x1     XIAO's own mode (WIFI_TEST / TEENSY_BRIDGE)

'k' IS THE KEEPALIVE AND 'h' IS HALT. A background thread sends a bare "k"
every HEARTBEAT_INTERVAL_S -- the firmware auto-disarms if its WiFi link
goes quiet for kLinkTimeoutMs (3 s). This script used to send "h", which
was correct against the older CornerBalance_WiFi build where h was the
keepalive and halt was on 'p'. Against CubliBalance.ino that would parse as
h0 = RESUME ten times a second and quietly defeat the HALT command.

'l', not 't', and 'p' is gone: the fused build kept the corner grammar.

'x' packets never reach the Teensy at all -- xiao_teensy_bridge.ino
consumes them to switch its own mode.

The session is saved to a timestamped .csv in ../telemetry/plot/ when the
plot window is closed -- that folder holds the PLOTMODE recordings from
both live plotters, with terminal_wifi.py's SERIALMONITORMODE logs kept
separately in ../telemetry/serial/. Run ../plot_session_csv.py with no
arguments afterwards and press d: its default is the newest recording,
which is the one you just made.

Requires: pip install matplotlib   (no pyserial needed -- plain UDP socket)

Usage:
    1. In CornerBalance_WiFi.ino, TELEMETRY_MODE should be PLOTMODE
       (it is by default).
    2. Set XIAO_IP below to the XIAO's static IP (must match XIAO_IP in
       xiao_teensy_bridge.ino).
    3. Run this script. The window opens once the XIAO has heard the
       first heartbeat and starts relaying telemetry.
    4. Type commands in the terminal (not the plot window).
    5. Close the window when done -- the session is written to
       ../telemetry/plot/telemetry_corner_<stamp>.csv.
"""

import csv
import queue
import socket
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---- settings ----
XIAO_IP   = "172.20.10.14"   # <-- must match XIAO_IP in xiao_teensy_bridge.ino
XIAO_PORT = 4210             # <-- must match UDP_PORT in xiao_teensy_bridge.ino
LOCAL_PORT = 4210            # local port this script listens on
NUM_COLS = 21
WINDOW_S = 10        # seconds visible in the scrolling plot
REDRAW_MS = 50       # plot redraw period (~20 fps)
HEARTBEAT_INTERVAL_S = 0.1   # keep well under the firmware's kLinkTimeoutMs (300 ms)
# Reference line only -- CORNER MODE HAS NO ARM GATE. This is cubli_gains.h's
# ARM_GATE, i.e. how close to equilibrium you should be before arming, not a
# threshold the firmware will enforce for you. See the COMMANDS note above.
ARM_GATE_DEG = 0.5

# Every recording in the FINAL tree lands under FINAL/telemetry/, split by the
# mode that wrote it: PLOTMODE csv here, SERIALMONITORMODE logs from
# terminal_wifi.py next door in ../telemetry/serial/. Keeping them apart means
# a directory listing answers "which of these can I plot?" on its own -- these
# are comma-delimited and always 21 columns, those are tab-delimited and
# whatever width their bring-up stage emits.
OUT_DIR = Path(__file__).resolve().parent.parent / "telemetry" / "plot"

COL_NAMES = [
    "t_ms",
    "phi_x_deg", "phi_y_deg", "phi_z_deg",
    "om_x_dps", "om_y_dps", "om_z_dps",
    "rho_x", "rho_y", "rho_z",
    "rho_x_lp", "rho_y_lp", "rho_z_lp",
    "tau_x", "tau_y", "tau_z",
    "tau_cmd_x", "tau_cmd_y", "tau_cmd_z",
    "armed", "gain_scale",
]

# Column indices, named so the plotting code below reads as physics rather
# than as arithmetic on magic numbers.
I_T = 0
I_PHI = (1, 2, 3)
I_OM = (4, 5, 6)
I_RHO = (7, 8, 9)
I_RHO_LP = (10, 11, 12)
I_TAU = (13, 14, 15)
I_TAU_CMD = (16, 17, 18)
I_ARMED = 19

AXES = ("x", "y", "z")
AXIS_COLORS = ("C0", "C1", "C2")


def reader_thread(sock, q, stop_event):
    """Blocking recvfrom() lives entirely on this thread -- the GUI thread
    never waits on I/O, so it can keep servicing its event loop."""
    last_valid = time.monotonic()
    last_warn = 0.0
    warned_bad_shape = False
    last_armed = None   # PLOTMODE's '#' echoes are printed, but the armed
                         # column is still the authoritative confirmation that
                         # a1/a0 landed -- and it also catches the control law
                         # self-disarming on a trip (overtilt/overspeed/NaN/
                         # link-loss).

    while not stop_event.is_set():
        try:
            raw, _addr = sock.recvfrom(2048)
        except socket.timeout:
            now = time.monotonic()
            if now - last_valid > 2.0 and now - last_warn > 2.0:
                last_warn = now
                print("... no valid PLOTMODE lines in the last 2s -- check "
                      "that CornerBalance_WiFi.ino's TELEMETRY_MODE is set to "
                      "PLOTMODE and its link mode is t1 (WiFi), that "
                      "xiao_teensy_bridge.ino is in x1 (TEENSY_BRIDGE) mode, "
                      "and that XIAO_IP/XIAO_PORT above match the XIAO. This "
                      "also fires on a genuine WiFi link loss, and while the "
                      "board is halted (p1).")
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            continue

        # Firmware console output -- echo it, don't parse it. This is how
        # command acks, the z1 tare readback and arm refusals stay visible
        # while the data stream stays pure CSV.
        if line.startswith("#"):
            print(line)
            continue

        parts = line.split(",")
        if len(parts) != NUM_COLS:
            if not warned_bad_shape:
                warned_bad_shape = True
                print(f"Got a packet but it isn't {NUM_COLS}-field PLOTMODE "
                      f"CSV ({len(parts)} fields): {line!r}")
                if len(parts) == 10:
                    print("10 fields means that's the EDGE build -- use "
                          "telemetry_python_wifi.py for it.")
                else:
                    print("If that looks tab-delimited or has a header row, "
                          "the firmware is still in SERIALMONITORMODE.")
            continue    # malformed/partial line -- skip it
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue

        last_valid = time.monotonic()

        armed = int(vals[I_ARMED])
        if last_armed is not None and armed != last_armed:
            print(f"Teensy: {'ARMED' if armed else 'DISARMED'}")
        last_armed = armed

        q.put(vals)


def command_thread(sock, xiao_addr, stop_event):
    """Forwards lines typed in this terminal as UDP packets to the XIAO --
    the a/g/c/z/p commands CornerBalance_WiFi.ino's handleCommandLine()
    understands, plus t<0/1> (Teensy link mode) and x<0/1> (XIAO mode).
    Runs on its own thread since input() blocks."""
    while not stop_event.is_set():
        try:
            line = input()
        except EOFError:
            break
        line = line.strip()
        if not line:
            continue
        sock.sendto((line + "\n").encode("ascii"), xiao_addr)


def heartbeat_thread(sock, xiao_addr, stop_event):
    """Keeps the firmware's WiFi-link watchdog alive and the XIAO's learned
    laptop address fresh during idle periods between commands.

    SENDS 'k', NOT 'h'. This used to send "h" because the older
    CornerBalance_WiFi build treated it as a no-op keepalive and put halt on
    'p'. CubliBalance.ino binds 'h' to HALT, where a bare "h" parses as
    atof("") = 0.0 -> h0 -> RESUME, ten times a second -- which would hold the
    HALT command down in the released position. 'k' is a no-op keepalive on
    every build in the tree.
    """
    while not stop_event.is_set():
        sock.sendto(b"k\n", xiao_addr)
        stop_event.wait(HEARTBEAT_INTERVAL_S)


def main():
    xiao_addr = (XIAO_IP, XIAO_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LOCAL_PORT))
    sock.settimeout(1.0)
    print(f"Listening on UDP :{LOCAL_PORT}, sending to {XIAO_IP}:{XIAO_PORT}. "
          f"Waiting for CORNER PLOTMODE data ({NUM_COLS} comma-separated "
          f"fields per packet)...")

    q = queue.Queue()
    stop_event = threading.Event()

    for target in (reader_thread, command_thread, heartbeat_thread):
        args = ((sock, q, stop_event) if target is reader_thread
                else (sock, xiao_addr, stop_event))
        threading.Thread(target=target, args=args, daemon=True).start()

    print("Type a1 / a0 / g<0..1> / c / z<0/1> / p<0/1> / t<0/1> / x<0/1> "
          "and press Enter to send to the XIAO.")
    print("NOTE: halt is p1/p0 on this build -- 'h' is the link keepalive.")

    fig, (ax_phi, ax_om, ax_rho, ax_tau) = plt.subplots(
        4, 1, sharex=True, figsize=(11, 10))
    fig.canvas.manager.set_window_title("Corner Balance Telemetry (WiFi/PLOTMODE)")

    ax_phi.set_title("Tilt (phi)"); ax_phi.set_ylabel("deg")
    ax_om.set_title("Body rate (omega)"); ax_om.set_ylabel("deg/s")
    ax_rho.set_title("Wheel speed (rho, dashed = 5s low-pass)"); ax_rho.set_ylabel("rad/s")
    ax_tau.set_title("Torque (dashed = commanded)"); ax_tau.set_ylabel("N*m")
    ax_tau.set_xlabel("t (s)")

    lines = {}
    for ax_name, ax in (("phi", ax_phi), ("om", ax_om)):
        for j, axis in enumerate(AXES):
            (lines[f"{ax_name}_{axis}"],) = ax.plot(
                [], [], color=AXIS_COLORS[j], label=f"{ax_name}_{axis}")

    for j, axis in enumerate(AXES):
        (lines[f"rho_{axis}"],) = ax_rho.plot(
            [], [], color=AXIS_COLORS[j], label=f"rho_{axis}")
        (lines[f"rho_{axis}_lp"],) = ax_rho.plot(
            [], [], color=AXIS_COLORS[j], linestyle="--", alpha=0.7,
            label=f"rho_{axis}_lp")
        (lines[f"tau_{axis}"],) = ax_tau.plot(
            [], [], color=AXIS_COLORS[j], label=f"tau_{axis}")
        (lines[f"tau_cmd_{axis}"],) = ax_tau.plot(
            [], [], color=AXIS_COLORS[j], linestyle="--", alpha=0.7,
            label=f"tau_cmd_{axis}")

    # |phi| is what kArmGate actually tests -- give it its own emphasis and
    # draw the reference tolerance itself, so "am I close enough to arm?" is
    # answered by looking rather than by arithmetic. It is a TARGET, not an
    # interlock -- corner mode arms at any tilt.
    (lines["phi_norm"],) = ax_phi.plot(
        [], [], color="k", linewidth=1.8, label="|phi|")
    ax_phi.axhline(ARM_GATE_DEG, color="r", linestyle=":", linewidth=1.2,
                   label=f"target ({ARM_GATE_DEG} deg, NOT enforced)")

    for ax in (ax_phi, ax_om, ax_rho, ax_tau):
        ax.legend(loc="upper left", fontsize=7, ncol=3)
        ax.grid(alpha=0.25)

    txt_phi = ax_phi.text(0.995, 0.95, "", transform=ax_phi.transAxes,
                          ha="right", va="top", fontsize=9, fontweight="bold")

    keys = (["phi_" + a for a in AXES] + ["om_" + a for a in AXES]
            + ["rho_" + a for a in AXES] + ["rho_" + a + "_lp" for a in AXES]
            + ["tau_" + a for a in AXES] + ["tau_cmd_" + a for a in AXES]
            + ["phi_norm"])
    log = {"t": [], "rows": [], "t0": None}
    log.update({k: [] for k in keys})

    def update(_frame):
        got_any = False
        while True:
            try:
                vals = q.get_nowait()
            except queue.Empty:
                break
            got_any = True

            log["rows"].append(vals)
            if log["t0"] is None:
                log["t0"] = vals[I_T]
            log["t"].append((vals[I_T] - log["t0"]) / 1000.0)

            for j, axis in enumerate(AXES):
                log[f"phi_{axis}"].append(vals[I_PHI[j]])
                log[f"om_{axis}"].append(vals[I_OM[j]])
                log[f"rho_{axis}"].append(vals[I_RHO[j]])
                log[f"rho_{axis}_lp"].append(vals[I_RHO_LP[j]])
                log[f"tau_{axis}"].append(vals[I_TAU[j]])
                log[f"tau_cmd_{axis}"].append(vals[I_TAU_CMD[j]])

            phi_norm = sum(vals[i] ** 2 for i in I_PHI) ** 0.5
            log["phi_norm"].append(phi_norm)

        artists = tuple(lines.values()) + (txt_phi,)
        if not got_any:
            return artists

        for k, line in lines.items():
            line.set_data(log["t"], log[k])

        txt_phi.set_text(f"|phi| = {log['phi_norm'][-1]:.3f} deg "
                         f"(target {ARM_GATE_DEG}, no gate)")

        t_s = log["t"][-1]
        lo, hi = max(0.0, t_s - WINDOW_S), max(WINDOW_S, t_s)
        for ax in (ax_phi, ax_om, ax_rho, ax_tau):
            ax.set_xlim(lo, hi)
            ax.relim()
            ax.autoscale_view(scalex=False)

        return artists

    fig.tight_layout()

    # FuncAnimation drives redraws on a timer owned by the GUI's own event
    # loop; plt.show() below is the ONLY blocking call on this thread, so the
    # window's message pump is serviced continuously.
    ani = FuncAnimation(fig, update, interval=REDRAW_MS, cache_frame_data=False)
    plt.show()

    stop_event.set()
    sock.close()

    rows = log["rows"]
    if rows:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        out_file = OUT_DIR / f"telemetry_corner_{stamp}.csv"
        with out_file.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(COL_NAMES)
            writer.writerows(rows)
        print(f"Saved {len(rows)} samples to {out_file}")
        # No filename to retype: the plotter's default is the newest recording.
        print("Plot it with:  python ../plot_session_csv.py   (press d)")
    else:
        print("No data captured -- nothing saved.")


if __name__ == "__main__":
    main()
