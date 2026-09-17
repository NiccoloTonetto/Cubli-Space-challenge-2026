"""telemetry_python_wifi.py  --  EDGE build (10 columns)

Copy of matlab/2Dmodel/Validation/telemetry_python_wifi.py, placed here
so the parser sits next to the firmware it parses. One functional change
from the original: '#'-prefixed lines are printed to the terminal as
firmware console output instead of being treated as malformed CSV, so an
untethered bring-up can still see the boot diagnostics and arm refusals
with USB unplugged. The original in matlab/ is untouched.

Live viewer / logger / command console for
EdgeBalance_WiFi/EdgeBalance_WiFi.ino + xiao_teensy_bridge/
xiao_teensy_bridge.ino. For the CORNER build use
telemetry_python_wifi_corner.py instead -- that one is 21 columns and
this script will reject its lines outright.

Same PLOTMODE CSV wire format, same three-panel live scrolling plot
(tilt / torque / wheel), same behavior of saving the full session to a
timestamped .csv when the window is closed -- only the transport is UDP
rather than a COM port:

    t_ms,theta_deg,theta_dot_dps,tau_Nm,tau_cmd_Nm,armed,gain_scale,wheel_omega_lp,wheel_pos,wheel_vel

The USB-side tooling (telemetry_python.py, telemetry_matlab.m) and the
USB firmware in ../USBMODE/ are untouched and still work exactly as
before -- this is a separate script for the separate WiFi build.

The session lands in ../telemetry/plot/ as telemetry_edge_<stamp>.csv,
alongside the corner plotter's telemetry_corner_<stamp>.csv; both formats
are named so a directory listing says which build produced a run without
opening it. terminal_wifi.py's SERIALMONITORMODE logs are kept apart in
../telemetry/serial/. Afterwards run ../plot_session_csv.py with no
arguments and press d -- its default is the newest recording.

Serial is read on a background thread into a queue; the main thread only
runs matplotlib's own FuncAnimation + plt.show() loop -- same reasoning as
telemetry_python.py (blocking I/O and GUI redraw in the same loop starves
the window's message pump under Windows).

This script doubles as the command console, same as telemetry_python.py:
type the same commands EdgeBalance_WiFi.ino's handleCommandLine()
understands (a1, a0, g<0..1>, o<deg>, e) into the terminal you launched
this script from and press Enter -- they're sent as UDP packets to the
XIAO.
Two new commands specific to this build:
    t0 / t1   -- switch the Teensy's link mode (USB / WiFi) live, no reboot
    x0 / x1   -- switch the XIAO's own mode (WIFI_TEST / TEENSY_BRIDGE)
A background thread also sends a lightweight "h" heartbeat every
HEARTBEAT_INTERVAL_S seconds -- EdgeBalance_WiFi.ino auto-disarms if it
doesn't hear anything on its WiFi link for kLinkTimeoutMs (300 ms in the
firmware), so this keeps idle periods between real commands from tripping
that failsafe. It also keeps the XIAO's learned "laptop address" fresh,
since xiao_teensy_bridge.ino only knows where to send telemetry once it's
heard from this script at least once.

Requires: pip install matplotlib   (no pyserial needed -- plain UDP socket)

Usage:
    1. In EdgeBalance_WiFi.ino, TELEMETRY_MODE should be PLOTMODE
       (it is by default).
    2. Set XIAO_IP below to the XIAO's static IP (must match XIAO_IP in
       xiao_teensy_bridge.ino).
    3. Run this script. A window opens with a live scrolling plot once the
       XIAO has heard the first heartbeat and starts relaying telemetry.
    4. In the terminal (not the plot window), type commands like a1, g0.5,
       o-2.3, t0, x1 and press Enter to send them.
    5. Close the window when you're done -- the full session is saved to
       ../telemetry/plot/telemetry_edge_<stamp>.csv.
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
NUM_COLS = 10
WINDOW_S = 10        # seconds visible in the scrolling plot
REDRAW_MS = 50       # plot redraw period (~20 fps)
HEARTBEAT_INTERVAL_S = 0.1   # keep well under the firmware's kLinkTimeoutMs (300 ms)

# Every recording in the FINAL tree lands under FINAL/telemetry/, split by the
# mode that wrote it: PLOTMODE csv here, SERIALMONITORMODE logs from
# terminal_wifi.py next door in ../telemetry/serial/.
OUT_DIR = Path(__file__).resolve().parent.parent / "telemetry" / "plot"

COL_NAMES = ["t_ms", "theta_deg", "theta_dot_dps", "tau_Nm", "tau_cmd_Nm",
             "armed", "gain_scale", "wheel_omega_lp", "wheel_pos", "wheel_vel"]


def reader_thread(sock, q, stop_event):
    """Blocking recvfrom() lives entirely on this thread -- the GUI thread
    never waits on I/O, so it can keep servicing its event loop."""
    last_valid = time.monotonic()
    last_warn = 0.0
    warned_bad_shape = False
    last_armed = None   # PLOTMODE suppresses the "# gArmed = TRUE" echo, so
                         # this is the only confirmation a1/a0 landed -- and
                         # it also catches the control law self-disarming on
                         # a safety trip (overtilt/overspeed/NaN/link-loss).

    while not stop_event.is_set():
        try:
            raw, _addr = sock.recvfrom(2048)
        except socket.timeout:
            now = time.monotonic()
            if now - last_valid > 2.0 and now - last_warn > 2.0:
                last_warn = now
                print("... no valid PLOTMODE lines in the last 2s -- check "
                      "that EdgeBalance_WiFi.ino's TELEMETRY_MODE is set "
                      "to PLOTMODE and its link mode is t1 (WiFi), that "
                      "xiao_teensy_bridge.ino is in x1 (TEENSY_BRIDGE) mode, "
                      "and that XIAO_IP/XIAO_PORT above match the XIAO. This "
                      "also fires on a genuine WiFi link loss.")
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            continue

        # Firmware console output -- echo it, don't parse it. During an
        # untethered bring-up this is the ONLY way to see the boot
        # diagnostics (CAN check, mount DCM, "BMI270 connected!", the
        # resolved edge candidate) and any arm refusal, since USB is
        # unplugged. Without this the script just sits silent and you
        # cannot tell a wiring fault from a WiFi fault.
        if line.startswith("#"):
            print(line)
            continue

        parts = line.split(",")
        if len(parts) != NUM_COLS:
            if not warned_bad_shape:
                warned_bad_shape = True
                print(f"Got a packet but it isn't {NUM_COLS}-field PLOTMODE "
                      f"CSV ({len(parts)} fields): {line!r}")
                print("If that looks tab-delimited or has a header/'#' "
                      "text, the firmware is still in SERIALMONITORMODE.")
            continue    # malformed/partial line -- skip it
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue

        last_valid = time.monotonic()

        armed = int(vals[5])
        if last_armed is not None and armed != last_armed:
            print(f"Teensy: {'ARMED' if armed else 'DISARMED'}")
        last_armed = armed

        q.put(vals)


def command_thread(sock, xiao_addr, stop_event):
    """Forwards lines typed in this terminal as UDP packets to the XIAO --
    the same a<0/1> / g<0..1> / o<deg> commands handleCommandLine()
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
    """Keeps the firmware's WiFi-link watchdog alive and the XIAO's
    learned laptop address fresh during idle periods between commands."""
    while not stop_event.is_set():
        sock.sendto(b"h\n", xiao_addr)
        stop_event.wait(HEARTBEAT_INTERVAL_S)


def main():
    xiao_addr = (XIAO_IP, XIAO_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LOCAL_PORT))
    sock.settimeout(1.0)
    print(f"Listening on UDP :{LOCAL_PORT}, sending to {XIAO_IP}:{XIAO_PORT}. "
          f"Waiting for PLOTMODE data ({NUM_COLS} comma-separated fields per packet)...")

    q = queue.Queue()
    stop_event = threading.Event()

    thread = threading.Thread(target=reader_thread, args=(sock, q, stop_event), daemon=True)
    thread.start()

    cmd_thread = threading.Thread(target=command_thread, args=(sock, xiao_addr, stop_event), daemon=True)
    cmd_thread.start()

    hb_thread = threading.Thread(target=heartbeat_thread, args=(sock, xiao_addr, stop_event), daemon=True)
    hb_thread.start()

    print("Type a1 / a0 / g<0..1> / o<deg> / t<0/1> / x<0/1> and press Enter "
          "to send to the XIAO.")

    fig_theta, ax_theta = plt.subplots(figsize=(7, 4))
    fig_theta.canvas.manager.set_window_title("Edge Balance Telemetry (WiFi/PLOTMODE) - Theta")
    ax_theta.set_title("Tilt (theta)"); ax_theta.set_ylabel("deg"); ax_theta.set_xlabel("t (s)")
    ax_theta.set_ylim(-30, 30)
    (lTheta,) = ax_theta.plot([], [], color="C0", label="theta (deg)")
    txtTheta = ax_theta.text(0.98, 0.95, "", transform=ax_theta.transAxes,
                              ha="right", va="top", fontsize=12, color="C0", fontweight="bold")

    fig_thetadot, ax_thetadot = plt.subplots(figsize=(7, 4))
    fig_thetadot.canvas.manager.set_window_title("Edge Balance Telemetry (WiFi/PLOTMODE) - Theta dot")
    ax_thetadot.set_title("Tilt rate (theta dot)"); ax_thetadot.set_ylabel("deg/s"); ax_thetadot.set_xlabel("t (s)")
    (lThetaDot,) = ax_thetadot.plot([], [], color="C1", label="theta dot (deg/s)")
    txtThetaDot = ax_thetadot.text(0.98, 0.95, "", transform=ax_thetadot.transAxes,
                                    ha="right", va="top", fontsize=12, color="C1", fontweight="bold")

    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    fig.canvas.manager.set_window_title("Edge Balance Telemetry (WiFi/PLOTMODE) - Torque & Wheel")
    ax2.set_title("Torque"); ax2.set_ylabel("N*m")
    ax3.set_title("Wheel");  ax3.set_ylabel("rad/s"); ax3.set_xlabel("t (s)")

    (lTau,)    = ax2.plot([], [], color="C0", label="tau")
    (lTauCmd,) = ax2.plot([], [], color="C1", linestyle="--", label="tau cmd")
    ax2.legend(loc="upper left")
    txtTau = ax2.text(0.98, 0.95, "", transform=ax2.transAxes,
                       ha="right", va="top", fontsize=10, color="C0", fontweight="bold")
    txtTauCmd = ax2.text(0.98, 0.82, "", transform=ax2.transAxes,
                          ha="right", va="top", fontsize=10, color="C1", fontweight="bold")

    (lWheelLp,) = ax3.plot([], [], color="C0", label="wheel omega lp")
    ax3.legend(loc="upper left")
    txtWheelLp = ax3.text(0.98, 0.95, "", transform=ax3.transAxes,
                           ha="right", va="top", fontsize=10, color="C0", fontweight="bold")

    fig_theta.tight_layout()
    fig_thetadot.tight_layout()
    fig.tight_layout()

    log = {"t": [], "theta": [], "theta_dot": [], "tau": [], "tau_cmd": [],
           "wheel_lp": [], "rows": [], "t0": None}

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
                log["t0"] = vals[0]
            t_s = (vals[0] - log["t0"]) / 1000.0

            log["t"].append(t_s)
            log["theta"].append(vals[1])
            log["theta_dot"].append(vals[2])
            log["tau"].append(vals[3])
            log["tau_cmd"].append(vals[4])
            log["wheel_lp"].append(vals[7])

        artists = (lTheta, lThetaDot, lTau, lTauCmd, lWheelLp,
                   txtTheta, txtThetaDot, txtTau, txtTauCmd, txtWheelLp)
        if not got_any:
            return artists

        lTheta.set_data(log["t"], log["theta"])
        lThetaDot.set_data(log["t"], log["theta_dot"])
        lTau.set_data(log["t"], log["tau"])
        lTauCmd.set_data(log["t"], log["tau_cmd"])
        lWheelLp.set_data(log["t"], log["wheel_lp"])

        txtTheta.set_text(f"{log['theta'][-1]:.2f} deg")
        txtThetaDot.set_text(f"{log['theta_dot'][-1]:.2f} deg/s")
        txtTau.set_text(f"tau: {log['tau'][-1]:.3f} N*m")
        txtTauCmd.set_text(f"tau cmd: {log['tau_cmd'][-1]:.3f} N*m")
        txtWheelLp.set_text(f"{log['wheel_lp'][-1]:.3f} rad/s")

        t_s = log["t"][-1]
        lo, hi = max(0.0, t_s - WINDOW_S), max(WINDOW_S, t_s)

        ax_theta.set_xlim(lo, hi)   # ylim stays fixed at (-30, 30), no autoscale

        ax_thetadot.set_xlim(lo, hi)
        ax_thetadot.relim()
        ax_thetadot.autoscale_view(scalex=False)

        for ax in (ax2, ax3):
            ax.set_xlim(lo, hi)
            ax.relim()
            ax.autoscale_view(scalex=False)

        # FuncAnimation only auto-redraws the figure it's attached to (fig);
        # the theta/theta_dot figures share the same GUI event loop but need
        # an explicit nudge to pick up their new data each tick.
        fig_theta.canvas.draw_idle()
        fig_thetadot.canvas.draw_idle()

        return artists

    # FuncAnimation drives redraws on a timer owned by the GUI's own event
    # loop; plt.show() below is the ONLY blocking call on this thread, so
    # the window's message pump is serviced continuously. plt.show() blocks
    # until ALL open figures (theta, theta_dot, torque/wheel) are closed.
    ani = FuncAnimation(fig, update, interval=REDRAW_MS, cache_frame_data=False)
    plt.show()

    stop_event.set()
    thread.join(timeout=1.0)
    sock.close()

    rows = log["rows"]
    if rows:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        # _edge_, matching the corner plotter's _corner_: both builds write into
        # the same telemetry/plot/ folder, and the two are different wire
        # formats (10 vs 21 columns), so the name has to say which one this is.
        out_file = OUT_DIR / f"telemetry_edge_{stamp}.csv"
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
