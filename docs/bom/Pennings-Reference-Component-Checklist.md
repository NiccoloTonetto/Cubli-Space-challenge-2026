---
tags: [space-challenge, sofia, cubli, hardware, bom, checklist]
---

# Pennings Reference Build + Component Checklist

Two things: (1) assessment of the open-source Willem Pennings balancing cube as a reference, (2) the complete component checklist and exactly which datasheet numbers to extract tomorrow.

---

# Part 1 — The Pennings balancing cube

**Repo:** https://github.com/willem-pennings/balancing-cube (~1.1k stars, GPL-3.0)
**Write-up:** https://willempennings.nl/balancing-cube/

## What it is

A corner-balancing reaction-wheel cube, reverse-engineered and drawn from scratch because the earlier work (ETH Cubli and others) was largely closed-source. It balances on a corner and can simultaneously rotate about its vertical axis under control. Repo contains `bom/`, `configuration/`, `electronics/`, `mechanical/`, `measurements/`, `software/`.

**Toolchain:** Fusion 360 (mechanical), KiCad 7 (PCBs), Arduino IDE (code, C++), Maxon motors + ESCON drivers tuned via Escon Studio, ESP32 as the microcontroller.

## Yes, this is very useful — here's specifically how

1. **`measurements/` is the most valuable folder for us.** Real parameter-identification data from a working cube. It's a ground-truth cross-check on our own free-swing and spin-down identification (see [[Deriving Dynamics from CAD]] §4) and a sanity check that our inertia numbers are the right order of magnitude.
2. **`mechanical/` (Fusion 360) as a design reference, not a copy.** Look specifically at how he solved: wheel-to-motor mounting, the corner pivot tip, frame stiffness with printed parts, cable routing. These are the details that cost days if you rediscover them.
3. **`bom/`** — validates our component checklist below and reveals parts we might forget.
4. **`electronics/` (KiCad)** — motherboard schematic shows power distribution, regulation, and how the ESP32/driver/IMU interconnect. Useful even if EnduroSat hands us different parts.
5. **`software/`** — his controller structure, calibration routines, and parameter file. **Read it, don't paste it** (see license warning).
6. **He points at Bobrow et al. (University of São Paulo)**, who reduce the required IMU count from six (ETH's design) to **one**. That's a meaningful simplification for us — one good IMU instead of six mediocre ones. Worth chasing the paper tomorrow; verify the citation before using it.

## Three cautions — read before the team gets attached to it

**⚠️ 1. GPL-3.0 is copyleft.** If we incorporate his code into our firmware, our firmware must also be released under GPL-3.0. That's usually fine for a student challenge, **but EnduroSat is a company and may have expectations about IP on work produced in their programme.** Ask them tomorrow. The safe default: read his code for architecture and technique, write our own. Treat the repo as documentation, not as a library. (Mechanical CAD under GPL is legally murkier but the same principle applies — use as reference, draw our own.)

**⚠️ 2. His hardware stack is probably not our hardware stack.** He uses Maxon motors with ESCON drivers (industrial, expensive, current-mode, tuned in proprietary software) and an ESP32. EnduroSat is giving us their own motors, batteries, and a Raspberry Pi. Gains, wheel geometry, and driver interface will **not** transfer. Architecture, calibration procedure, and mechanical solutions will.

**⚠️ 3. Reproducing it is not trivial — and the known failure mode is exactly the one I warned about.** A hardware engineer who rebuilt the cube from his files reports on his site: balance holds for roughly 7–8 seconds, then falls when it attempts static balance with rotation; after parameter changes, roughly one attempt in twenty succeeds and any small disturbance topples it.

That "works for ~8 seconds then falls" signature is textbook **wheel momentum saturation from a COM offset** ([[Sizing Memo]] §5): the wheel absorbs the bias torque until it runs out of speed, then the cube has no authority left. It confirms two of our design rules are not optional — physically shim the COM, and penalise wheel momentum in the LQR cost. Also note his mention of accelerometer calibration files: **IMU calibration is a required step, not a nice-to-have.**

**Recommendation:** clone the repo tonight, have the CAD subteam study `mechanical/` and `bom/` tomorrow, and have me go through `measurements/` and `software/` architecture. Do not plan to fork it.

---

# Part 2 — Component checklist for tomorrow

Organised by subsystem. **Bold = ask EnduroSat explicitly.** For each, the datasheet numbers we actually need and why.

## 2.1 Actuation

### Motors (×3) — **provided**
| Parameter | Symbol | Why we need it |
|---|---|---|
| Torque constant | $K_t$ [N·m/A] | Converts our torque command to current; sizing memo input |
| **Max continuous torque** | $\tau_{\max}$ | **Sets max cube mass — CAD is blocked on this** |
| Peak/stall torque | $\tau_{\text{peak}}$ | Jump-up feasibility |
| **Max speed** | $\omega_{\max}$ [rpm] | Momentum capacity $h_{\max}=I_w\omega_{\max}$ |
| **Rotor inertia** | $J_{\text{rotor}}$ [g·cm²] | Adds to wheel inertia; often buried or omitted |
| Terminal resistance | $R$ [Ω] | Power/thermal budget |
| Terminal inductance | $L$ [mH] | Current-loop bandwidth |
| Pole pairs | — | Required for FOC commutation |
| Hall sensors present? | — | Wheel speed feedback without a separate encoder |
| Mass | $m_{\text{motor}}$ | COM and inertia budget |
| Nominal voltage | — | Must match battery |

*If rotor inertia isn't listed, ask Maxon-equivalent datasheets or estimate; it's usually small versus the wheel but not negligible.*

### Motor drivers (×3) — **provided?**
**The single most important question tomorrow.**
| Parameter | Why |
|---|---|
| **Control modes** | Current/torque mode is what our whole model assumes. Speed-only ESC changes the plant, the linearisation, and the gains |
| **Command interface** | Analog? PWM? UART/CAN/SPI? Determines MCU wiring and latency |
| Current loop bandwidth | Sets $\tau_{\text{lag}}$ in the model; want ≫ our 500 Hz outer loop |
| Continuous / peak current | Must cover $\tau_{\max}/K_t$ |
| Supply voltage range | Battery compatibility |
| **Regenerative handling** | Decelerating a wheel pumps energy back — see §2.4 |
| Encoder/Hall input support | Wheel speed feedback path |
| Configuration software needed? | Maxon needs ESCON Studio; adds a setup day if so |

### Reaction wheels (×3) — **we fabricate**
Not a purchased part but a design deliverable. Maximise inertia per unit mass → **rim-weighted**, not solid discs. Printed hub with a press-fit or bolted metal ring (brass/steel) is the standard student solution.
- Need: outer radius, rim mass, hub material, **balance quality** (an unbalanced wheel at 8000 rpm shakes the IMU — see §2.2)
- Need: **hub/coupling to the motor shaft** — set-screw hub, keyed, or clamping. Easy to forget; blocks assembly if missing.

## 2.2 Sensing

### IMU — **check if provided; likely we source**
Per Bobrow et al., **one good IMU** may be enough (vs. ETH's six).
| Parameter | Target / why |
|---|---|
| **Gyro range** | ≥ ±2000 °/s — jump-up spins fast; clipping ruins the estimate |
| Gyro noise density | [°/s/√Hz] — feeds Kalman tuning |
| Gyro bias stability / random walk | Dominant error source in attitude drift |
| Accel range | ±8 g typical |
| Accel noise density | Kalman tuning |
| **Max output data rate** | ≥ 1 kHz so it doesn't bottleneck the loop |
| **Interface** | **SPI strongly preferred over I²C** — I²C at 1 kHz with three devices gets marginal |
| Built-in DLPF / FIFO | Helps with wheel vibration |
| Temperature sensitivity | Bias drifts as motors heat the frame |

**Mounting matters as much as the part:** soft-mount, keep away from motors, and record its position in the body frame — the lever arm shows up in the accelerometer measurement model.

### Wheel speed feedback
Hall sensors (usually integrated in BLDC) or encoders. Need: counts per revolution, interface, and whether the driver exposes speed or we read it directly.

### Current sensing
Usually inside the driver. If exposed, it gives us actual applied torque — extremely useful for validating the torque command and measuring $\eta_{\text{brake}}$.

## 2.3 Compute — **provided (Raspberry Pi)**
Ask, in this order:
1. **Which board?** Pi Pico (MCU, fine) vs Pi 4/5 (Linux, real-time problem — see [[Revised Schedule - Provided Hardware]] §2).
2. **Is a microcontroller available alongside?** For the split architecture.
3. Available interfaces: SPI, UART, PWM, CAN?
4. Programming/debug setup — do we get a debugger, or is it USB-only?

Plus: **oscilloscope or logic analyser** for measuring loop jitter. Not optional — we need to *prove* the loop rate, not assume it.

## 2.4 Power — **provided (battery)**
| Item | What to ask |
|---|---|
| Battery | Nominal voltage, cell count, capacity [Ah], **max continuous discharge (C)** — must cover 3 motors at peak |
| | **Max charge rate** — reaction wheels regenerate on deceleration |
| | Mass and dimensions — drives COM |
| | Connector type |
| BMS / protection | Present? Does it cut out on regen overvoltage? |
| Regulators | Motor bus → 5 V → 3.3 V. Current capacity for Pi (a Pi 4/5 wants ~3 A!) |
| **Bulk capacitance** | Across the motor bus, to absorb regenerative spikes. Commonly forgotten; causes driver overvoltage faults |
| Power switch + fuse | Safety, and you *will* want a fast kill switch |
| Emergency stop | See safety below |

**Regenerative braking is a real design item here.** Every time a wheel decelerates it acts as a generator. A LiPo will normally absorb it, but if there's a protection circuit that blocks charge current, bus voltage spikes and drivers fault out — which looks like a random controller failure. Ask about it tomorrow.

## 2.5 Mechanical / structural
| Item | Note |
|---|---|
| Frame | 3D printed — **measure effective density on a coupon** ([[Deriving Dynamics from CAD]] §2.3) |
| **Corner pivot tip** | A printed corner deforms and wears within hours. Needs a **hardened insert** — steel ball, dowel pin, or ground tip. Easy to overlook, kills repeatability |
| **Balancing pad** | Needs *friction* so the corner doesn't slip. Rubber/silicone mat. Pennings uses a dedicated pad |
| Bearings | If the wheel shaft isn't fully motor-supported |
| Fasteners | Heat-set inserts for printed parts (threading plastic directly fails under vibration) |
| Cable management | Contributes mass asymmetrically → COM offset → momentum saturation |
| **Wheel guards** | Pennings explicitly recommends this: thin discs over the spinning flywheels to prevent trapped fingers. **Three wheels at 8000 rpm in a room of 15 people — do this** |
| Shim/ballast provision | Small screws or weights to trim the COM after assembly. Design it in |

## 2.6 Test equipment
- **1D pendulum jig** (arm + one wheel) — highest-value item of week 1
- Precision scale (COM and mass measurement)
- Bench power supply (debug without draining the battery)
- Multimeter, oscilloscope/logic analyser
- Soft landing surface — the cube *will* fall over, repeatedly
- Tether/soft rig for first corner attempts
- Camera on a tripod — film everything from day 1

---

## The three questions that block everyone else tomorrow
1. **Driver control mode** — torque/current, or speed only?
2. **Which Raspberry Pi**, and is an MCU available alongside?
3. **Motor $\tau_{\max}$, $\omega_{\max}$, $K_t$, rotor inertia** — the sizing script inputs; CAD is blocked until these exist.

Everything else can wait a day. These three cannot.
