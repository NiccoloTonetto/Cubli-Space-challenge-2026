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
| 4-ATF | [`Stage4_AutoTrim_RateFilter_WiFi/`](Stage4_AutoTrim_RateFilter_WiFi/) | 29 | 4-AT plus a **rate low-pass** on the om block, and a 34× faster trim |
| 4-FO | [`Stage4_FixedOffset/`](Stage4_FixedOffset/) | 26 | 4-AT with trim **hardcoded** to its converged value, and **no arm gate** |
| 4-FOF | [`Stage4_FixedOffset_RateFilter/`](Stage4_FixedOffset_RateFilter/) | 29 | 4-FO plus 4-ATF's rate low-pass |
| 4-FOK | [`Stage4_FixedOffset_Kalman/`](Stage4_FixedOffset_Kalman/) | 32 | 4-FO with the low-pass replaced by a **mode-augmented Kalman filter** — *experimental* |
| 5 | [`Stage5_Release_WiFi/`](Stage5_Release_WiFi/) | 21 | full law, **real** trip policy, latched reason |

**Stage 4 is not in this folder and is not duplicated.** `../CornerBalance_WiFi/` already was the WiFi Stage 4 — it is the validated build with recorded sessions, and it is left exactly as it was. It is also the only stage with two ways to read it:

- `TELEMETRY_MODE PLOTMODE` (its default) → [`../telemetry_python_wifi_corner.py`](../telemetry_python_wifi_corner.py), live plots
- `TELEMETRY_MODE SERIALMONITORMODE` → [`../terminal_wifi.py`](../terminal_wifi.py), same as every other stage

Stage 5's 21 fields end in `trip_reason`, not `gain_scale` — the one layout difference from Stage 4.

Stage 4-AT is Stage 4's law with the manual snapshot tare swapped for the closed-loop adaptive trim from *Automatic Trim — Replacing the Hardcoded IMU Offset.md*. Its 26 fields are Stage 4's 21 plus `trim_x_deg trim_y_deg trim_z_deg trim_com_mm trim_enabled`. Watching trim converge takes minutes, so `d20` (25 Hz) is the rate to read it at.

Stage 4-ATF is 4-AT plus the two fixes *hw-run-analysis.md* drew out of a 373.5 s continuous balance:

- **Fix 4.1** — a first-order low-pass (default 20 Hz, `f<Hz>` to sweep 15–25) on the body rate, feeding **only** the control law's om block. That run saturated torque 71.8 % of the time because a ~35 Hz structural mode was riding on the gyro and the K2 term kept amplifying it; a 1.3 Hz loop cannot damp a 35 Hz mode, only feed it. `w_b` stays raw everywhere else — estimator, safety trips, and the `om_x/y/z_dps` telemetry columns.
- **Fix 4.5** — `gKAdapt` default raised from 2.922e-6 (τ_a = 60 s) to **1e-4**, ~21× below the re-derived stability limit. Trim now converges in ~10 s instead of ~4 min. The response was measured **non-monotonic** (1e-4 beats 1e-3), so do not tune past it by feel.

Its 29 fields are 4-AT's 26 plus `om_x_filt_dps om_y_filt_dps om_z_filt_dps`, printed next to the raw ones deliberately: the gap between the two is the only direct evidence the filter is doing anything. `d20` is still the rate to read it at — convergence is fast now, but the raw-vs-filtered comparison is much easier to eyeball slowly.

### The fixed-offset family (4-FO / 4-FOF / 4-FOK)

These three are the auto-trim builds with the adaptation **retired**. Trim already did its job: `perfect_equilibrium_2.log` (2026-08-20) held a converged `gTrim` for 132.41 s with per-axis std < 0.0012 °, which is a genuine fixed point rather than a value still drifting. That mean is now the compile-time `kPhiOffset`, and `gTrim`/`gKAdapt`/`updateTrim()`/`applyTrimGuards()` are gone with it. The trim five columns stay in the telemetry as constants (`trim_enabled` = 0 always) so the line keeps a width the analysis tools already know.

Two consequences worth being explicit about:

- **The offset goes stale on any mechanical change** — battery moved, cable re-routed, bolt re-torqued. Nothing in these files can detect that. Re-run the USB `Stage4_AutoTrim`, let it re-converge, and update `kPhiOffset`.
- **The arm gate is gone.** `a1` arms from any tilt. The law is only validated for small angles (recovery envelope ~2.7–3.1 ° worst case), so placing the cube near equilibrium first is now operator discipline, not a code-enforced check. Over WiFi that leaves the link watchdog and `kMaxTilt`'s 25 ° trip as the only automatic protections — see the link-loss table below.

4-FOK additionally replaces the first-order low-pass with a per-axis 3-state Kalman filter that carries the ~30–40 Hz structural mode as its **own state** (`n<Hz>` / `z<0-1>` sweep it live), and adds `mode_x_dps mode_y_dps mode_z_dps` for its estimate of that mode's contribution. It is **experimental and unvalidated** — the mode's real frequency has never been pinned down by a tap test, and observed peaks spread 29.7–41.0 Hz across three sessions. A wrong mode model is not neutral here: unlike a notch that simply stops helping off-frequency, this filter trusts its own model. Read the sketch header before flashing it.

One WiFi-specific trap on 4-FOK: judging `mode_x/y/z_dps` needs sampling well above the mode, so **do not decimate it** — at `d20` you are looking at aliasing, not at the filter. Even `d2`'s 250 Hz arrives irregularly once the ~105 lines/s UDP ceiling bites, so treat a WiFi capture as qualitative and take anything you intend to FFT (including [`../../kalman_mode_hint.py`](../../kalman_mode_hint.py) input) over USB with `l0`.

---

## Command grammar

Every stage keeps its USB command letters **byte-identical** — with two exceptions in the Stage 4 auto-trim builds that the transport forced, called out below. That was the point: the terminal should behave like the Serial Monitor, including what you type into it.

The fixed-offset family has its own table further down; it fits none of these columns and, unlike the auto-trim builds, needed **no** remapping at all.

| | Stage 1 | Stage 2 | Stage 3 | Stage 4-AT | Stage 4-ATF | Stage 5 |
|---|---|---|---|---|---|---|
| arm / disarm | — | `a<0/1>` | `a<0/1>` | `a<0/1>` (gated) | `a<0/1>` (gated) | `a<0/1>` (gated, clears trip latch) |
| gain scale | — | — | `g<0..1>` | `g<0..1>` | `g<0..1>` | — |
| tilt limit | — | — | `m<deg>` | — | — | — |
| select wheel | `w<0/1/2>` | — | — | — | — | — |
| fire pulse | `p` | — | — | — | — | — |
| pulse size | `t<Nm>` | — | — | — | — | — |
| log marker | — | — | — | — | — | `r<val>` |
| seed / clear trim | — | — | — | `z<0/1>` | `z<0/1>` | — |
| freeze / resume adapt | — | — | — | `y<0/1>` *(USB: `x`)* | `y<0/1>` *(USB: `x`)* | — |
| adaptation gain | — | — | — | `n<value>` *(USB: `k`)* | `n<value>` *(USB: `k`)* | — |
| rate filter corner freq | — | — | — | — | `f<Hz>` | — |
| re-resolve corner | `c` | `c` | `c` | `c` | `c` | `c` |
| halt | `h<0/1>` | `h<0/1>` | `h<0/1>` | `h<0/1>` | `h<0/1>` | `h<0/1>` |

The WiFi layer therefore could not use `h`, `p` or `t`. Across all the stages the letters `a c f g h m n p r t w y z` are taken, so the link controls sit on free ones:

| | |
|---|---|
| `k` | keepalive no-op — feeds the link watchdog. `terminal_wifi.py` sends it every 100 ms |
| `l<0/1>` | link mode: `l0` telemetry on USB, `l1` telemetry on Serial1 (boot default) |
| `d<N>` | telemetry decimation, 1..100. `d2` = 250 Hz (default), `d20` = 25 Hz readable |
| `x<0/1>` | the XIAO's own mode — consumed by the bridge, **never reaches the Teensy** |

### The auto-trim stages' two remapped letters

The USB `Stage4_AutoTrim` and `Stage4_AutoTrim_RateFilter` bind `x<0/1>` to the trim freeze and `k<value>` to the adaptation gain. Neither survives the link, so both WiFi builds remap them the same way:

- **`x` never reaches the Teensy** — the bridge consumes it (`x0` = WIFI_TEST, `x1` = TEENSY_BRIDGE). A freeze on `x` would work over USB and be silently dead over WiFi. It is **`y<0/1>`** here, same polarity.
- **`k` is the keepalive** — `terminal_wifi.py` sends a bare `k` every 100 ms, which would parse as `atof("") = 0.0` and zero the adaptation gain ten times a second. Trim would sit frozen at zero and *look* like a converged trim of zero in the telemetry. It is **`n<value>`** here.

Stage 4-ATF's third command, **`f<Hz>`**, did *not* have to move — `f` is free across every stage and the bridge relays it verbatim.

### The fixed-offset stages keep every letter

| | Stage 4-FO | Stage 4-FOF | Stage 4-FOK |
|---|---|---|---|
| arm / disarm | `a<0/1>` **(no gate)** | `a<0/1>` **(no gate)** | `a<0/1>` **(no gate)** |
| gain scale | `g<0..1>` | `g<0..1>` | `g<0..1>` |
| rate filter corner freq | — | `f<Hz>` | — |
| mode natural freq | — | — | `n<Hz>` |
| mode damping ζ | — | — | `z<0-1>` |
| re-resolve corner | `c` | `c` | `c` |
| halt | `h<0/1>` | `h<0/1>` | `h<0/1>` |

Nothing moved, and the reason is the mechanism these files removed: the two collisions that forced 4-AT and 4-ATF to remap were both in the **trim machinery**, and there is no trim machinery here. Nothing is bound to `x`, and nothing is bound to `k`.

That frees `n` and `z` — which 4-FOK then binds to its own, completely different meanings. **Do not read letters across from 4-ATF**: there `n` is the adaptation gain and `z` seeds trim; here `n` is the mode's natural frequency and `z` its damping ratio. Same keys, same folder, unrelated effects.

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

Kill `terminal_wifi.py` while armed and confirm the board disarms. The deadline is the stage's own `kLinkTimeoutMs`: **300 ms** on Stages 1, 2, 3 and 5, **3 s** on the whole Stage 4 family. In WiFi mode the laptop is your disarm button, and the watchdog is what covers you when it disappears — but it is **not** the safety net. The physical stop/catch and the e-stop are.

| | on link loss |
|---|---|
| Stage 1 | cancels an in-flight pulse (there is no `gArmed`) |
| Stages 2, 3 | `gArmed = false` |
| Stage 4-AT | `gArmed = false`, which also freezes trim adaptation — `updateTrim()` is gated on `gArmed`, so "do not learn from a dropped link" costs no extra state. The converged value is **kept**, not cleared |
| Stage 4-ATF | same as 4-AT. The **rate filter is deliberately not** frozen or reset — it is a measurement filter, not learned state, so it keeps tracking `w_b` while disarmed and the om block sees a settled signal the moment you re-arm |
| Stages 4-FO, 4-FOF, 4-FOK | `gArmed = false`. There is no trim to freeze, and the rate filter / Kalman filter are left running for the same reason 4-ATF's is. **Check this one first:** with the arm gate gone, this watchdog and `kMaxTilt` are the *only* automatic protections these three have |
| Stage 5 | `gArmed = false`, and `trip_reason` is deliberately **left alone** — losing the radio is not something the cube did, and overwriting a latched tilt/omega/nan would destroy the evidence you need before re-arming |

---

## Logs

`terminal_wifi.py` tees everything received to `session_<tag>_<stamp>.log` in [`../../telemetry/serial/`](../../telemetry/README.md) (`--no-log` to skip) — the SERIALMONITORMODE half of the recordings folder, kept apart from the live plotters' PLOTMODE csv in `telemetry/plot/`. Because the header row is in the stream, [`../../plot_session_csv.py`](../../plot_session_csv.py) opens a **Stage 5** log directly — 21 columns, though it will label the last one `gain_scale` when it is really `trip_reason`. Run it with no arguments and press `f` to pick the log from a list.

Stage 1's 13, Stages 2/3's 18, the 26 of 4-AT and 4-FO, the 29 of 4-ATF and 4-FOF, and 4-FOK's 32 columns all fall outside that script's 10/21 auto-detection. Those logs are raw records to read or post-process, which is what the bring-up stages want anyway — though 26, 29 and 32 are all widths `plot_session_csv.py` knows (`CORNER_TRIM` / `CORNER_TRIM_FILT` / `CORNER_TRIM_KALMAN`), so passing one by name still works.

A fixed-offset log is width-identical to its auto-trim twin, so **width alone will not tell you which build produced it** — read `trim_enabled`, which is 0 on every fixed-offset line and 1 on an adapting one, or check the boot block the log opens with.

---

## If nothing arrives

Work the ladder in [`../link-bringup/`](../link-bringup/) rather than guessing — it isolates one hop per stage. [`../link_check.py`](../link_check.py) gives a staged verdict in one shot, and [`../link-bringup/Stage4_BridgeThroughput/stage4_rate_check.py`](../link-bringup/Stage4_BridgeThroughput/stage4_rate_check.py) proves the pipeline with no control law attached.

Two settings must match across all three devices: **1000000 baud** and **UDP port 4210**. Five more must match between the XIAO sketch and the laptop script: `WIFI_SSID`, `WIFI_PASSWORD`, `XIAO_IP`, `XIAO_GATEWAY`, `XIAO_SUBNET`.

One recurring trap worth naming: with the XIAO's USB attached and nothing reading its CDC port, its `loop()` collapses to ~0.5 iterations/s and the bridge relays almost nothing. Unplug the XIAO's USB, or set `ENABLE_LINK_STATUS 0`.
