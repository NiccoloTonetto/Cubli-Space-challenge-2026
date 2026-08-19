# Corner bring-up, over WiFi

The five-stage corner ladder from [`../../../corner-bringup/`](../../../corner-bringup/), running untethered. Commands and telemetry travel

```
laptop  <--UDP :4210-->  XIAO ESP32C6  <--Serial1 @ 1 Mbaud-->  Teensy 4.1
```

and you read them in a terminal that reproduces the Arduino Serial Monitor exactly — same fields, same tab separators, same `#` console lines, same header. **No plotting in the bring-up stages.** Plotting exists for Stage 4 only, and is optional even there.

The control law in every file here is a verbatim copy of its USB source. **Retune in `../../../corner-bringup/`, then re-copy** — never the other way round.

---

## The stages

| Stage | Sketch | Fields | What is live |
|---|---|---|---|
| 1 | [`Stage1_CornerIDAndPulse_WiFi/`](Stage1_CornerIDAndPulse_WiFi/) | 13 | open loop — corner ID + one-wheel torque pulse |
| 2 | [`Stage2_RateOnly_WiFi/`](Stage2_RateOnly_WiFi/) | 18 | om block only (phi, rho masked) |
| 3 | [`Stage3_PositionDamping_WiFi/`](Stage3_PositionDamping_WiFi/) | 18 | phi + om (rho masked), hand-ramped gain |
| 4 | [`../CornerBalance_WiFi/`](../CornerBalance_WiFi/) | 21 | full 9-state law + friction feedforward |
| 4-AT | [`Stage4_AutoTrim_WiFi/`](Stage4_AutoTrim_WiFi/) | 26 | same law, manual `z1` tare replaced by **automatic trim** |
| 5 | [`Stage5_Release_WiFi/`](Stage5_Release_WiFi/) | 21 | full law, **real** trip policy, latched reason |

**Stage 4 is not in this folder and is not duplicated.** `../CornerBalance_WiFi/` already was the WiFi Stage 4 — it is the validated build with recorded sessions, and it is left exactly as it was. It is also the only stage with two ways to read it:

- `TELEMETRY_MODE PLOTMODE` (its default) → [`../telemetry_python_wifi_corner.py`](../telemetry_python_wifi_corner.py), live plots
- `TELEMETRY_MODE SERIALMONITORMODE` → [`../terminal_wifi.py`](../terminal_wifi.py), same as every other stage

Stage 5's 21 fields end in `trip_reason`, not `gain_scale` — the one layout difference from Stage 4.

Stage 4-AT is Stage 4's law with the manual snapshot tare swapped for the closed-loop adaptive trim from *Automatic Trim — Replacing the Hardcoded IMU Offset.md*. Its 26 fields are Stage 4's 21 plus `trim_x_deg trim_y_deg trim_z_deg trim_com_mm trim_enabled`. Watching trim converge takes minutes, so `d20` (25 Hz) is the rate to read it at.

---

## Command grammar

Every stage keeps its USB command letters **byte-identical** — with two exceptions in Stage 4-AT that the transport forced, called out below. That was the point: the terminal should behave like the Serial Monitor, including what you type into it.

| | Stage 1 | Stage 2 | Stage 3 | Stage 4-AT | Stage 5 |
|---|---|---|---|---|---|
| arm / disarm | — | `a<0/1>` | `a<0/1>` | `a<0/1>` (gated) | `a<0/1>` (gated, clears trip latch) |
| gain scale | — | — | `g<0..1>` | `g<0..1>` | — |
| tilt limit | — | — | `m<deg>` | — | — |
| select wheel | `w<0/1/2>` | — | — | — | — |
| fire pulse | `p` | — | — | — | — |
| pulse size | `t<Nm>` | — | — | — | — |
| log marker | — | — | — | — | `r<val>` |
| seed / clear trim | — | — | — | `z<0/1>` | — |
| freeze / resume adapt | — | — | — | `y<0/1>` *(USB: `x`)* | — |
| adaptation gain | — | — | — | `n<value>` *(USB: `k`)* | — |
| re-resolve corner | `c` | `c` | `c` | `c` | `c` |
| halt | `h<0/1>` | `h<0/1>` | `h<0/1>` | `h<0/1>` | `h<0/1>` |

The WiFi layer therefore could not use `h`, `p` or `t`. Across all the stages the letters `a c g h m n p r t w y z` are taken, so the link controls sit on free ones:

| | |
|---|---|
| `k` | keepalive no-op — feeds the link watchdog. `terminal_wifi.py` sends it every 100 ms |
| `l<0/1>` | link mode: `l0` telemetry on USB, `l1` telemetry on Serial1 (boot default) |
| `d<N>` | telemetry decimation, 1..100. `d2` = 250 Hz (default), `d20` = 25 Hz readable |
| `x<0/1>` | the XIAO's own mode — consumed by the bridge, **never reaches the Teensy** |

### Stage 4-AT's two remapped letters

The USB `Stage4_AutoTrim` binds `x<0/1>` to the trim freeze and `k<value>` to the adaptation gain. Neither survives the link:

- **`x` never reaches the Teensy** — the bridge consumes it (`x0` = WIFI_TEST, `x1` = TEENSY_BRIDGE). A freeze on `x` would work over USB and be silently dead over WiFi. It is **`y<0/1>`** here, same polarity.
- **`k` is the keepalive** — `terminal_wifi.py` sends a bare `k` every 100 ms, which would parse as `atof("") = 0.0` and zero the adaptation gain ten times a second. Trim would sit frozen at zero and *look* like a converged trim of zero in the telemetry. It is **`n<value>`** here.

### The one legacy difference

`../CornerBalance_WiFi/` predates this folder and binds `h` = keepalive, `p` = halt, `t` = link mode. It was not changed. It already accepts `k` as a keepalive alias, which is why `terminal_wifi.py` drives it and the four stages here without knowing which is which. If you drop into that sketch, remember halt is `p1`.

---

## Telemetry rate

Control stays at 500 Hz in every stage. Telemetry is emitted every `gTelemetryDecim`-th cycle — default 2, i.e. 250 Hz, matching the validated build.

Raising it is rarely what you want. 21 fields at 500 Hz is ~85 kB/s against a 1 Mbaud link, and the UDP hop is the tighter constraint: link-bringup Stage 6 measured **~105 lines/s** actually reaching the laptop. Lost datagrams show up as missing lines, not corrupt ones.

Going the other way is genuinely useful: `d20` drops to 25 Hz, which a human can read. `d2` puts it back for capture. Changing the rate re-prints the header, so a log stays self-describing across the change.

The header is also re-printed whenever the link comes back from dead — that is how a terminal started *after* the Teensy booted still learns what the columns are.

---

## Running a stage

1. Flash the stage to the Teensy, and [`../xiao_teensy_bridge/`](../xiao_teensy_bridge/) to the XIAO (once — it is stage-agnostic).
2. Power up. **Keep the cube perfectly still through boot** — `setup()` spends ~2 s calibrating gyro bias and bakes in any motion.
3. `python terminal_wifi.py --tag stage1`
4. Confirm the boot block arrives: `mount DCM check` with det ≈ 1 and row norms ≈ 1, `BMI270 connected!`, `# corner resolved: [+1,+1,+1] …`, then the header and the stage's instruction lines.
5. Work the stage's own checklist, in its sketch header.

Only `[+1,+1,+1]` and `[-1,-1,-1]` are mechanically reachable; the other six corners foul the frame. See [`../../../corner-bringup/README.md`](../../../corner-bringup/README.md).

### Before the first armed run, on every stage

Kill `terminal_wifi.py` while armed and confirm the board disarms within 300 ms. In WiFi mode the laptop is your disarm button, and the watchdog is what covers you when it disappears — but it is **not** the safety net. The physical stop/catch and the e-stop are.

| | on link loss |
|---|---|
| Stage 1 | cancels an in-flight pulse (there is no `gArmed`) |
| Stages 2, 3 | `gArmed = false` |
| Stage 4-AT | `gArmed = false`, which also freezes trim adaptation — `updateTrim()` is gated on `gArmed`, so "do not learn from a dropped link" costs no extra state. The converged value is **kept**, not cleared |
| Stage 5 | `gArmed = false`, and `trip_reason` is deliberately **left alone** — losing the radio is not something the cube did, and overwriting a latched tilt/omega/nan would destroy the evidence you need before re-arming |

---

## Logs

`terminal_wifi.py` tees everything received to `session_<tag>_<stamp>.log` in [`../../telemetry/serial/`](../../telemetry/README.md) (`--no-log` to skip) — the SERIALMONITORMODE half of the recordings folder, kept apart from the live plotters' PLOTMODE csv in `telemetry/plot/`. Because the header row is in the stream, [`../../plot_session_csv.py`](../../plot_session_csv.py) opens a **Stage 5** log directly — 21 columns, though it will label the last one `gain_scale` when it is really `trip_reason`. Run it with no arguments and press `f` to pick the log from a list.

Stage 1's 13, Stages 2/3's 18 and Stage 4-AT's 26 columns fall outside that script's 10/21 auto-detection. Those logs are raw records to read or post-process, which is what the bring-up stages want anyway.

---

## If nothing arrives

Work the ladder in [`../link-bringup/`](../link-bringup/) rather than guessing — it isolates one hop per stage. [`../link_check.py`](../link_check.py) gives a staged verdict in one shot, and [`../link-bringup/Stage4_BridgeThroughput/stage4_rate_check.py`](../link-bringup/Stage4_BridgeThroughput/stage4_rate_check.py) proves the pipeline with no control law attached.

Two settings must match across all three devices: **1000000 baud** and **UDP port 4210**. Five more must match between the XIAO sketch and the laptop script: `WIFI_SSID`, `WIFI_PASSWORD`, `XIAO_IP`, `XIAO_GATEWAY`, `XIAO_SUBNET`.

One recurring trap worth naming: with the XIAO's USB attached and nothing reading its CDC port, its `loop()` collapses to ~0.5 iterations/s and the bridge relays almost nothing. Unplug the XIAO's USB, or set `ENABLE_LINK_STATUS 0`.
