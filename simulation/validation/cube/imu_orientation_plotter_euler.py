"""imu_orientation_plotter_euler.py

Live 3D orientation viewer for firmware streaming Euler angles instead of a
quaternion. Expects one plain CSV line per control cycle, no header/comment
lines, with roll/pitch/yaw in place of q0..q3:

    t_ms,roll_deg,pitch_deg,yaw_deg,wx_dps,wy_dps,wz_dps,tau_x,tau_y,tau_z,
    armed,gain_scale,wheelX_pos,wheelX_vel,wheelY_pos,wheelY_vel,wheelZ_pos,
    wheelZ_vel

roll/pitch/yaw are in degrees, intrinsic Z-Y-X (yaw then pitch then roll,
i.e. the standard aerospace/body-321 sequence), body -> inertial -- the
Euler-angle counterpart to the (w,x,y,z) Hamilton quaternion convention
documented in docs/dynamics/Quaternions-Complete-Guide.md. wx/wy/wz is body
angular rate in deg/s, shown here purely as a live readout so you can
cross-check it against the rendered rotation (e.g. rotate the IMU about its
body X axis and confirm both wx's sign and the cube's on-screen rotation
match what you expect for that axis).

This is the Euler-input counterpart to
../Quaternion/imu_orientation_plotter_quaternion.py, which expects q0..q3
fields instead of roll_deg/pitch_deg/yaw_deg -- pick whichever one matches
what the firmware is actually printing, since the two wire formats aren't
interchangeable (18 fields here vs 19 for the quaternion build, and the
field layout differs starting at index 1).

Python equivalent in spirit to matlab/2Dmodel/Validation/telemetry_python.py:
same background-thread-reads-serial-into-a-queue architecture, same
FuncAnimation-drives-the-only-blocking-call-on-the-main-thread structure,
same session-CSV-on-close behavior. See that file for the full rationale
on why serial I/O and the GUI loop are split across threads.

NOTE: as of this writing, neither Skeleton_3Axis.ino nor Skeleton_3Axis_WiFi.ino
actually emit this roll_deg/pitch_deg/yaw_deg wire format -- their PLOTMODE
telemetry only prints the quaternion fields (see the Quaternion/ sibling
script). Use this script once/if the firmware's telemetryPlot() is changed
to print Euler angles directly instead of (or in addition to) q0..q3; the
field layout above is this script's assumption for that case and is easy
to adjust in NUM_COLS/COL_NAMES/the update() unpacking below if the actual
firmware format ends up different.

Requires: pip install pyserial matplotlib numpy

Usage:
    1. Point the firmware's PLOTMODE telemetry at the CSV format above and
       re-upload.
    2. Set PORT below to the Teensy's serial port (Windows: check Device
       Manager -> Ports, e.g. "COM9").
    3. Run this script. A window opens with a live-rotating cube.
       Or run with --demo to preview the renderer with synthetic rotation
       and no hardware attached at all:
       python imu_orientation_plotter_euler.py --demo
    4. In the terminal (not the plot window), type commands like a1, g0.5,
       h1 and press Enter to send them to the Teensy (same grammar as
       telemetry_python.py; add t0/t1/k if using the _WiFi build's link-mode
       commands -- note the _WiFi build uses 'k' as its heartbeat, not 'h',
       since 'h' means HALT there).
    5. Close the window when you're done -- the full session is saved to
       a timestamped .csv file next to this script.
"""

import argparse
import csv
import math
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---- settings ----
PORT = "COM9"        # <-- set to your Teensy's port
BAUD = 115200
NUM_COLS = 18
REDRAW_MS = 33        # plot redraw period (~30 fps)
OUT_DIR = Path(__file__).resolve().parent

COL_NAMES = ["t_ms", "roll_deg", "pitch_deg", "yaw_deg",
             "wx_dps", "wy_dps", "wz_dps",
             "tau_x", "tau_y", "tau_z", "armed", "gain_scale",
             "wheelX_pos", "wheelX_vel", "wheelY_pos", "wheelY_vel",
             "wheelZ_pos", "wheelZ_vel"]

# Unit cube (edge length 1, centered at origin) with a distinct color per
# face, so a 90-degree-off rotation is obvious to spot on screen -- an
# all-white cube looks the same rotated wrong.
_CUBE_VERTS = np.array([
    [-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [-0.5, 0.5, -0.5],
    [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5],
])
_CUBE_FACES = [
    ([0, 1, 2, 3], "#b71c1c"),  # -Z, dark red
    ([4, 5, 6, 7], "#e53935"),  # +Z, red
    ([0, 1, 5, 4], "#1b5e20"),  # -Y, dark green
    ([2, 3, 7, 6], "#43a047"),  # +Y, green
    ([0, 3, 7, 4], "#0d47a1"),  # -X, dark blue
    ([1, 2, 6, 5], "#1e88e5"),  # +X, blue
]

AXIS_LEN = 1.0


def euler_to_rotmat(roll_deg, pitch_deg, yaw_deg):
    """Intrinsic Z-Y-X (yaw-pitch-roll), body -> inertial: R = Rz(yaw) @
    Ry(pitch) @ Rx(roll). Same convention as quat_to_euler_deg() in the
    Quaternion/ sibling script, so both scripts agree on what "roll 10 deg"
    looks like."""
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def reader_thread(ser, q, stop_event):
    """Blocking serial reads live entirely on this thread -- the GUI thread
    never waits on I/O, so it can keep servicing its event loop."""
    last_valid = time.monotonic()
    last_warn = 0.0
    warned_bad_shape = False
    last_armed = None

    while not stop_event.is_set():
        raw = ser.readline()
        if not raw:
            now = time.monotonic()
            if now - last_valid > 2.0 and now - last_warn > 2.0:
                last_warn = now
                print("... no valid PLOTMODE lines in the last 2s -- check "
                      "that the firmware's TELEMETRY_MODE is set to "
                      "PLOTMODE (re-upload after changing it), that it's "
                      "printing roll/pitch/yaw fields, and that PORT/BAUD "
                      "above match the Teensy.")
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            continue

        parts = line.split(",")
        if len(parts) != NUM_COLS:
            if not warned_bad_shape:
                warned_bad_shape = True
                print(f"Got a line but it isn't {NUM_COLS}-field PLOTMODE "
                      f"CSV ({len(parts)} fields): {line!r}")
                print("If that looks tab-delimited or has a header/'#' "
                      "text, the firmware is still in SERIALMONITORMODE. If "
                      "the field count is close but off by one, the "
                      "firmware may be printing a quaternion instead -- use "
                      "../Quaternion/imu_orientation_plotter_quaternion.py "
                      "for that.")
            continue
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue

        last_valid = time.monotonic()

        armed = int(vals[10])
        if last_armed is not None and armed != last_armed:
            print(f"Teensy: {'ARMED' if armed else 'DISARMED'}")
        last_armed = armed

        q.put(vals)


def demo_thread(q, stop_event):
    """Synthetic rotating roll/pitch/yaw, no hardware needed -- lets you
    check the renderer/axes/colors are right before trusting it against
    real IMU data. Each angle oscillates at its own slow rate so all three
    axes visibly move."""
    t0 = time.monotonic()
    last = t0
    while not stop_event.is_set():
        now = time.monotonic()
        t = now - t0
        dt = now - last
        last = now

        roll_deg = 30.0 * math.sin(2 * math.pi * 0.10 * t)
        pitch_deg = 20.0 * math.sin(2 * math.pi * 0.07 * t + 1.0)
        yaw_deg = 40.0 * math.sin(2 * math.pi * 0.05 * t + 2.0)

        # Numerically differentiate for the wx/wy/wz readout -- a real
        # firmware would report gyro rate directly, but for the demo this
        # keeps the two consistent with each other.
        eps = 1e-3
        roll2 = 30.0 * math.sin(2 * math.pi * 0.10 * (t + eps))
        pitch2 = 20.0 * math.sin(2 * math.pi * 0.07 * (t + eps) + 1.0)
        yaw2 = 40.0 * math.sin(2 * math.pi * 0.05 * (t + eps) + 2.0)
        wx = (roll2 - roll_deg) / eps
        wy = (pitch2 - pitch_deg) / eps
        wz = (yaw2 - yaw_deg) / eps

        t_ms = t * 1000.0
        vals = [t_ms, roll_deg, pitch_deg, yaw_deg, wx, wy, wz,
                0.0, 0.0, 0.0, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q.put(vals)
        time.sleep(0.01)


def command_thread(ser, stop_event):
    """Forwards lines typed in this terminal straight to the Teensy -- same
    a<0/1> / g<0..1> / h<0/1> (/ t<0/1> / k for the _WiFi build) commands
    handleSerialCommands() already understands. Runs on its own thread
    since input() blocks."""
    while not stop_event.is_set():
        try:
            line = input()
        except EOFError:
            break
        line = line.strip()
        if not line:
            continue
        ser.write((line + "\n").encode("ascii"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true",
                         help="synthetic rotating cube, no serial hardware needed")
    args = parser.parse_args()

    q = queue.Queue()
    stop_event = threading.Event()
    ser = None

    if args.demo:
        print("Demo mode: synthetic rotation, no serial connection opened.")
        thread = threading.Thread(target=demo_thread, args=(q, stop_event), daemon=True)
        thread.start()
    else:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        ser.reset_input_buffer()
        print(f"Opened {PORT} @ {BAUD} baud. Waiting for PLOTMODE data "
              f"({NUM_COLS} comma-separated fields per line)...")
        thread = threading.Thread(target=reader_thread, args=(ser, q, stop_event), daemon=True)
        thread.start()
        cmd_thread = threading.Thread(target=command_thread, args=(ser, stop_event), daemon=True)
        cmd_thread.start()
        print("Type a1 / a0 / g<0..1> / h<0/1> and press Enter to send to "
              "the Teensy (same commands as the Serial Monitor).")

    fig = plt.figure(figsize=(8, 8))
    fig.canvas.manager.set_window_title(
        "IMU Orientation - Euler" + (" (DEMO)" if args.demo else " (PLOTMODE)"))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect([1, 1, 1])
    lim = 1.3
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

    # Fixed world/inertial axes, dashed and gray, drawn once -- compare
    # these against the solid colored body axes below to validate the
    # IMU's reference frame.
    for vec, label in [((AXIS_LEN, 0, 0), "X"), ((0, AXIS_LEN, 0), "Y"), ((0, 0, AXIS_LEN), "Z")]:
        ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], linestyle="--", color="gray", linewidth=1)
        ax.text(vec[0] * 1.1, vec[1] * 1.1, vec[2] * 1.1, f"{label}_world", color="gray", fontsize=8)

    cube = Poly3DCollection([], edgecolor="k", linewidths=0.5)
    ax.add_collection3d(cube)

    # Solid body axes: red=X, green=Y, blue=Z, rotating with the cube.
    body_axis_lines = [ax.plot([], [], [], linewidth=3, color=c)[0]
                        for c in ("red", "green", "blue")]

    txt = ax.text2D(0.02, 0.98, "", transform=ax.transAxes, va="top",
                     fontsize=9, family="monospace")
    ax.text2D(0.02, 0.02,
              "body axes: X=red Y=green Z=blue (solid)\nworld axes: dashed gray",
              transform=ax.transAxes, va="bottom", fontsize=8, color="dimgray")

    log = {"rows": []}

    def update(_frame):
        latest = None
        while True:
            try:
                latest = q.get_nowait()
                log["rows"].append(latest)
            except queue.Empty:
                break
        if latest is None:
            return (cube, txt, *body_axis_lines)

        roll_deg, pitch_deg, yaw_deg = latest[1], latest[2], latest[3]
        w_dps = latest[4:7]
        R = euler_to_rotmat(roll_deg, pitch_deg, yaw_deg)

        verts_rot = _CUBE_VERTS @ R.T
        faces = [[verts_rot[i] for i in idx] for idx, _ in _CUBE_FACES]
        cube.set_verts(faces)
        cube.set_facecolor([c for _, c in _CUBE_FACES])

        for line, axis_vec in zip(body_axis_lines, np.eye(3) * AXIS_LEN):
            end = R @ axis_vec
            line.set_data([0, end[0]], [0, end[1]])
            line.set_3d_properties([0, end[2]])

        txt.set_text(
            f"rpy= [{roll_deg:+7.2f} {pitch_deg:+7.2f} {yaw_deg:+7.2f}] deg\n"
            f"w  = [{w_dps[0]:+7.2f} {w_dps[1]:+7.2f} {w_dps[2]:+7.2f}] deg/s"
        )

        return (cube, txt, *body_axis_lines)

    ani = FuncAnimation(fig, update, interval=REDRAW_MS, cache_frame_data=False)
    plt.show()

    stop_event.set()
    thread.join(timeout=1.0)
    if ser is not None:
        ser.close()

    rows = log["rows"]
    if rows:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_file = OUT_DIR / f"imu_orientation_euler_{stamp}.csv"
        with out_file.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(COL_NAMES)
            writer.writerows(rows)
        print(f"Saved {len(rows)} samples to {out_file}")
    else:
        print("No data captured -- nothing saved.")


if __name__ == "__main__":
    main()
