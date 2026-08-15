# Edge Balance — final firmware (USB + WiFi)

The two builds that actually balance the cube on an edge, kept side by side.
Both are Stage 4 full law (`K[0]` position + `K[1]` rate + `K[2]` momentum
management + friction feedforward + 0.5° arm gate), axis-selectable X / Y / Z.

| Folder | Link | Use it when |
|---|---|---|
| [`Stage4_FullLaw/`](Stage4_FullLaw/Stage4_FullLaw.ino) | USB only | Bench work with a laptop cable attached. **Byte-identical copy** of `../edge-bringup/Stage4_FullLaw/Stage4_FullLaw.ino` — the hardware-validated reference. |
| [`Stage4_FullLaw_WiFi/`](Stage4_FullLaw_WiFi/Stage4_FullLaw_WiFi.ino) | USB **and** Serial1→XIAO→WiFi/UDP | Running untethered. Identical control path; adds the link, a watchdog, and non-blocking command parsing. |

The WiFi build changes **nothing** in the physics, estimator, gains, trips or
arm gate. If you retune, retune in `Stage4_FullLaw/` and re-copy the changed
constants across — do not let the two drift.

## What the WiFi build adds

- **Second channel on Serial1** (1 Mbaud) to a XIAO ESP32C6 running
  [`firmware/XIAO/xiao_teensy_bridge/`](../../../XIAO/xiao_teensy_bridge/xiao_teensy_bridge.ino),
  which relays lines to/from the laptop over UDP. That sketch is reused
  **as-is** — no edits beyond WiFi credentials and the static IP.
- **Link watchdog**: in WiFi mode, if no line arrives on Serial1 for 300 ms,
  it auto-disarms — exactly what an `a0` does. USB mode never auto-disarms.
- **Non-blocking line reading** on both streams. The USB build's
  `Serial.readStringUntil()` can block up to its timeout on a partial line,
  which is not acceptable inside a 2 ms control cycle.
- **Boot diagnostics mirrored to both streams**, so an untethered bring-up
  still shows the CAN check, IMU status, the "hold still" gyro-calibration
  prompt and the resolved edge candidate in the WiFi console.
- `TELEMETRY_MODE` defaults to `PLOTMODE` here (the USB build defaults to
  `SERIALMONITORMODE`) — the point of this build is running with the PC-side
  plotter.

### Commands (accepted on **either** stream, always)

| Cmd | Effect |
|---|---|
| `a1` / `a0` | arm / disarm. `a1` still goes through the 0.5° arm gate. |
| `g<0..1>` | gain scale |
| `o<deg>` | `kPhiOffset` — per axis, live |
| `e` | re-resolve the edge candidate, answer echoed back |
| `t0` / `t1` | link mode: USB / WiFi (boot default `t1`) |
| `h` or `k` | no-op keepalive |

Both `h` and `k` are accepted, so
[`matlab/2Dmodel/Validation/telemetry_python_wifi.py`](../../../../matlab/2Dmodel/Validation/telemetry_python_wifi.py)
— which sends `h` every 100 ms — works against this sketch **unmodified**.
The PLOTMODE CSV is the same 10-field shape that script already parses:

```
t_ms, phi_edge_deg, om_edge_dps, tau_Nm, tau_cmd_Nm, armed, gain_scale, wheel_omega_lp, wheel_pos, wheel_vel
```

Its plot labels say `theta` / `theta_dot`; read those as `phi_edge` /
`om_edge`. Same positions, same units (deg, deg/s).

---

## Flashing — end to end

Two boards, two separate USB cables, two separate flashes. Do the XIAO first:
once it's programmed you never need to touch it again, and you can then verify
the WiFi half of the link before the cube is ever armed.

### 0. One-time toolchain setup

**Teensy 4.1** — Arduino IDE + Teensyduino. Libraries needed (Library Manager
or manual): `MoteusTeensy` (brings in `ACAN_T4`) and
`SparkFun_BMI270_Arduino_Library`.

**XIAO ESP32C6** —
1. *File → Preferences → Additional Board Manager URLs*, add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. *Tools → Board → Boards Manager* → install **esp32** (Espressif).
3. *Tools → Board* → **XIAO_ESP32C6**.
4. *Tools → USB CDC On Boot* → **Enabled** (boot logs only — the operational
   bridge runs entirely over Serial1 + WiFi).

### 1. Wiring (check before powering anything)

```
Teensy Serial1 TX1 (pin 1)  ->  XIAO D7 (RX)
Teensy Serial1 RX1 (pin 0)  <-  XIAO D6 (TX)
GND                          --  GND        <- required, common ground
```

RX/TX crossed, and the common ground is not optional — without it you get
garbage or nothing at all on the link.

### 2. Flash the XIAO

Open `firmware/XIAO/xiao_teensy_bridge/xiao_teensy_bridge.ino` and set, at the
top of the file:

- `WIFI_SSID` / `WIFI_PASSWORD`
- `XIAO_IP`, `XIAO_GATEWAY`, `XIAO_SUBNET` — a free static address on your
  actual subnet. Static so the PC script always has one fixed address to send
  to, with no discovery handshake.

⚠️ The credentials currently committed in that file point at a guest network
(`ES-Guest`). Guest/campus networks very often use **client isolation**, which
blocks laptop↔XIAO traffic entirely and will break this bridge no matter how
correct everything else is. Use a home network or a phone hotspot.

Plug the XIAO in by its own USB-C, select its port, upload. Open its Serial
Monitor at 115200 — it prints the connect progress, its IP, RSSI, and then a
`[status]` line once a second.

### 3. Flash the Teensy

Open `Stage4_FullLaw_WiFi/Stage4_FullLaw_WiFi.ino` and check two things near
the top:

```cpp
static const Axis kAxis = AXIS_X;   // <<< the edge you're actually balancing on
#define TELEMETRY_MODE PLOTMODE     // PLOTMODE for the python plotter
```

`kAxis` drives everything else automatically — moteus CAN id (X=2, Y=3, Z=1),
edge-candidate table, wheel sign, boot confirmation print. Use
`SERIALMONITORMODE` instead if you'd rather read raw tab-delimited text.

Plug the Teensy in by **its** USB, select its port (Board: Teensy 4.1), upload.

### 4. Bring-up sequence

1. Power the moteus / cube. Keep the cube **perfectly still** through boot —
   `setup()` spends ~2 s calibrating gyro bias and will bake in any motion.
2. Confirm the boot lines: active axis + moteus id, `mount DCM check` with
   det ≈ 1 and all row norms ≈ 1, `BMI270 connected!`, and the resolved edge
   candidate. If it warns that `dot0` and `dot1` are close to each other,
   re-seat the cube and send `e` to re-resolve before going any further.
3. On the laptop, set `XIAO_IP` in `telemetry_python_wifi.py` to match the
   XIAO's static IP, then run it. The XIAO only learns the laptop's address
   once it has heard from it, so telemetry starts flowing after the first
   keepalive lands.
4. Watch `phi_edge`. Hold the cube at the edge equilibrium and read off the
   resting value; if it's consistently off zero, send `o<deg>` with that
   measured offset. **This is per axis** — it does not carry over from the
   axis you tuned last.
5. `a1` to arm. It only takes when `|phi_edge| < 0.5°`; otherwise the gate
   refuses and says so.
6. `a0` to disarm — with the velocity cap removed at this stage, **that is the
   safety net**. Keep a hand on the cube and stay ready to send it.

### How to tell an arm actually took

In PLOTMODE the `# ARM REFUSED …` line lands in the CSV stream and the PC
parser drops it. The authoritative signal is the **`armed` column** — the plot
script prints `Teensy: ARMED` / `Teensy: DISARMED` whenever it flips. If `a1`
appears to do nothing, the gate refused it: get closer to vertical, or set
`o<deg>` first.

### If nothing shows up

Work down the pipeline in this order — the XIAO's once-a-second `[status]`
line over USB tells you which stage is broken:

| Symptom on `[status]` | Where the problem is |
|---|---|
| `serial1_lines=0` | Teensy side — wiring (RX/TX swapped, no common GND), or the Teensy is in `t0`/USB mode. Send `t1`. |
| `serial1_lines` climbing, `udp_out=0` | The XIAO has never heard from the laptop. Firewall, wrong `XIAO_IP` in the script, or client isolation on the network. |
| both counts climbing, laptop still blank | Look at the PC side: wrong `LOCAL_PORT`, or firewall blocking inbound UDP. |
| `wifi=DOWN` | Credentials, or the network needs a captive-portal login. |

If the CSV shape looks wrong to the script, the firmware is still in
`SERIALMONITORMODE` — reflash with `PLOTMODE`.

### Fallback

If the WiFi link is misbehaving and you just need to balance: flash
`Stage4_FullLaw/Stage4_FullLaw.ino` over USB and use
`matlab/2Dmodel/Validation/telemetry_python.py`. It is the same control law
and needs no XIAO at all. You can also leave the WiFi build flashed and send
`t0` to move telemetry back onto USB live, without a reflash.

---

## Next

`../edge-bringup/Stage5_Release/` is the unsupported release attempt — same
law, but with the real `DISARM`/`OMEGA_CAP` trip policy restored (velocity cap
back to 40 rad/s) and a latched trip reason. This folder deliberately stops at
Stage 4, hand-held. A WiFi variant of Stage 5 is not written yet; when it is,
it should reuse this folder's link/watchdog/command layer verbatim.
