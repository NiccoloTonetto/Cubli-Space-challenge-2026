# USBMODE — the hardware-validated builds

Both sketches here are **byte-identical copies** of their bring-up sources.
Only the filename changed, so the Arduino IDE's folder-name rule is satisfied.

| Sketch | Copy of | Wheels |
|---|---|---|
| [`CornerBalance/CornerBalance.ino`](CornerBalance/CornerBalance.ino) | `../../corner-bringup/Stage4_FullLaw/Stage4_FullLaw.ino` | 3 (ids 1/2/3) |
| [`EdgeBalance/EdgeBalance.ino`](EdgeBalance/EdgeBalance.ino) | `../../edge-bringup/Stage4_FullLaw/Stage4_FullLaw.ino` | 1 (X/Y/Z selectable) |

Verify no drift at any time — both should print nothing:

```powershell
git diff --no-index "CornerBalance/CornerBalance.ino" `
  "../../corner-bringup/Stage4_FullLaw/Stage4_FullLaw.ino"

git diff --no-index "EdgeBalance/EdgeBalance.ino" `
  "../../edge-bringup/Stage4_FullLaw/Stage4_FullLaw.ino"
```

**This is where you retune.** Change gains here, confirm on hardware, then
copy the changed constants into the matching `../WIFIMODE/` build.

## Toolchain

Arduino IDE + Teensyduino, Board: **Teensy 4.1**. Libraries: `MoteusTeensy`
(brings in `ACAN_T4`) and `SparkFun_BMI270_Arduino_Library`.

## CornerBalance

3 wheels, coupled 9-state law (`phi(3)`, `om(3)`, `rho(3)`). Corner candidate
resolved from measurement at boot against 8 candidates.

Telemetry: 21 fields, tab-delimited, 500 Hz, with a header row.

```
t_ms  phi_x_deg phi_y_deg phi_z_deg  om_x_dps om_y_dps om_z_dps
      rho_x rho_y rho_z  rho_x_lp rho_y_lp rho_z_lp
      tau_x tau_y tau_z  tau_cmd_x tau_cmd_y tau_cmd_z  armed gain_scale
```

| Cmd | Effect |
|---|---|
| `a1` / `a0` | arm / disarm. `a1` passes the 0.5° gate on `norm3(phi)`. |
| `g<0..1>` | gain scale |
| `c` | re-resolve the corner candidate |
| `z1` / `z0` | tare `gPhiOffset` to the current `phi` / clear it |
| `h1` / `h0` | **HALT** (idle — no IMU reads, no CAN traffic) / resume |

`h` is HALT here. In the WiFi build it is a keepalive and halt moves to `p`
— see the top-level README.

**If `a1` is refused even resting naturally**, that is the COM offset from
the unmounted battery/DC-DC, not a bad hold. Rest the cube at its natural
balance point, send `z1` to tare, then arm. Re-tare or `z0` once the missing
mass is fitted.

## EdgeBalance

1 axis at a time. Set the axis at the top of the file:

```cpp
static const Axis kAxis = AXIS_X;   // AXIS_X / AXIS_Y / AXIS_Z
```

`kAxis` drives everything else automatically — moteus CAN id (X=2, Y=3, Z=1),
edge-candidate table, wheel sign, boot confirmation print.

Telemetry: 10 fields. `TELEMETRY_MODE` selects `SERIALMONITORMODE`
(tab-delimited, default) or `PLOTMODE` (CSV).

```
t_ms, phi_edge_deg, om_edge_dps, tau_Nm, tau_cmd_Nm,
armed, gain_scale, wheel_omega_lp, wheel_pos, wheel_vel
```

| Cmd | Effect |
|---|---|
| `a1` / `a0` | arm / disarm. `a1` passes the 0.5° gate on `\|phi_edge\|`. |
| `g<0..1>` | gain scale |
| `o<deg>` | `kPhiOffset` — **per axis**, does not carry over between axes |
| `e` | re-resolve the edge candidate |

## Bring-up sequence (both)

1. Power the moteus / cube. Keep the cube **perfectly still** through boot —
   `setup()` spends ~2 s calibrating gyro bias and bakes in any motion.
2. Check the boot lines: `mount DCM check` with det ≈ 1 and all row norms
   ≈ 1, `BMI270 connected!`, and the resolved candidate. If it warns the top
   two candidates are close, re-seat the cube and re-resolve (`c` / `e`)
   before going further.
3. Read the resting tilt. Correct it — `z1` (corner) or `o<deg>` (edge).
4. `a1` to arm. It only takes inside the 0.5° gate; otherwise it refuses and
   says why.
5. `a0` to disarm. With the velocity cap loosened at this stage, **that is
   the safety net.** Keep a hand on the cube.

## PC-side tooling

`matlab/2Dmodel/Validation/telemetry_python.py` (edge PLOTMODE, over the COM
port). For plotting a saved session afterwards, use
[`../plot_session_csv.py`](../plot_session_csv.py) — it reads both formats.
