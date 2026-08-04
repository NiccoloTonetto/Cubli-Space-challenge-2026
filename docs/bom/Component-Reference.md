---
tags: [space-challenge, sofia, cubli, components, hardware, reference, bom]
---

# Component Reference — full specs

One page per subsystem, every BOM item. **Verified** = manufacturer datasheet or docs. **Estimate** = flagged, needs measuring or confirming.

Motor has its own note: [[Motor - TMotor Antigravity 4006]]. Architecture: [[System Integration - Full Cube]].

---

## 1. Actuation

### 1.1 mjbots moteus-n1 — motor driver ×3

| Parameter | Value |
|---|---|
| Topology | 3-phase brushless **FOC** |
| Input voltage | **10–54 V** (≤12S) — 6S at 25.2 V full is comfortably inside |
| Phase current | **9 A uncooled**, 26 A cooled, **100 A peak** |
| Peak electrical power | 2 kW @ 36 V |
| **Control rate** | **15–30 kHz** |
| PWM switching | 15–60 kHz |
| CPU | STM32G4 |
| Comms | **5 Mbps CAN-FD**, ±58 V bus fault protection |
| Current sense noise | ~30 mA |
| Dimensions | **46 × 46 × 8 mm** |
| Mass | **14.6 g** |
| Temperature | −40 to +85 °C |
| Peripheral power out | 3.3 V and 5 V, **100 mA each** total across aux connectors |
| Firmware | open source, Apache 2.0 |
| Included | 1× XT30 connector, 1× JST-PH3 housing + crimps, 1× ¼″ diametric magnet |

**Ports:** AUX2 = JST **GH-7** (this is where the MA600 goes). Power = XT30. CAN = JST PH3, daisy-chainable.

**Headroom check:** 9 A uncooled × $K_t$ 0.02513 = **0.226 N·m continuous from the driver**, versus the motor's thermal limit near 0.12 N·m. **The motor binds, not the driver** — comfortable.

> ⚠️ **The n1 has an integrated on-axis absolute encoder** (AS5047P-class) and ships with a magnet for it. It only works if the board sits directly at the motor shaft end. On our outrunners that's awkward — which is exactly why the BOM includes MA600 breakouts. See §2.1.

Thermal: improvable with a 5 V fan or heat spreader if boards run hot.

### 1.2 TowerPro MG92B — servo ×1–3

| Parameter | Value |
|---|---|
| Type | metal-gear micro servo |
| Torque | ~3.0 kg·cm @ 4.8 V, ~3.5 kg·cm @ 6 V |
| **Speed** | **~0.08 s / 60° @ 4.8 V**, ~0.07 s @ 6 V |
| Voltage | 4.8–6 V |
| Mass | ~13.8 g |
| Dimensions | ~22.8 × 12.2 × 25.6 mm |
| Control | standard PWM, 50 Hz, 1–2 ms pulse |

**Purpose unconfirmed** — working assumption is a wheel brake for jump-up. ⚠️ **Timing concern:** at 0.08 s/60°, a clamp stroke could take 50–80 ms against a pendulum time constant of ~114 ms. The impulsive-brake model assumes the transfer is *fast* relative to the dynamics. Needs mechanical advantage and short travel; **measure actual clamp time on the jig.** See [[System Integration - Full Cube]] §8.2.

---

## 2. Sensing

### 2.1 mjbots MA600 breakout — wheel encoder ×3

| Parameter | Value |
|---|---|
| Sensor | MPS MA600, **TMR** (tunnel magnetoresistance) |
| Type | **absolute** angle, full 360° |
| Resolution | up to **16-bit** absolute angle over SPI |
| Interface | **SPI**, 3.3 V |
| Connector | GH6; ships with **GH6→GH7 cable** for moteus AUX2 |
| Included | sense magnet |

**Why MA600 rather than the moteus's built-in encoder:** lower noise and better linearity than hall-effect sensors (AS5047P, AS5600, MA702, MA732), and — the practical reason — it lets the encoder sit at the shaft while the moteus board lives anywhere within cable reach.

> ⭐ **The finding that matters for CAD: the MA600 supports off-axis mounting.** mjbots specifically developed off-axis support using a **diametrically magnetised ring magnet** (they stock 32 × 22 × 4 mm) with the sensor beside it rather than over the shaft end. The chip is placed near the board edge for exactly this.
>
> This **solves the outrunner mounting problem elegantly** — instead of needing access to a rotating shaft end, put a ring magnet around the bell and the sensor alongside. Worth designing around from v0.

> ⚠️ **Requires moteus firmware 2024-10-29 or newer.** Check and update before bring-up.

Also reported: less audible noise at the same encoder bandwidth and control gains.

### 2.2 SparkFun BMI270 — IMU ×1

| Parameter | Value |
|---|---|
| Type | Bosch 6-DoF (3-axis gyro + 3-axis accel) |
| Gyro range | ±125 to ±2000 °/s |
| Accel range | ±2 to ±16 g |
| Interfaces | I²C **and SPI** |
| Voltage | 3.3 V |

⚠️ **To verify:** (a) does the SparkFun Qwiic breakout expose the **SPI pins**? We want SPI, not I²C, at 1 kHz. (b) Configured **output data rate** — this may be the loop-rate ceiling rather than the Teensy.

Configure gyro to **±2000 °/s** — jump-up spins fast and clipping destroys the estimate.

---

## 3. Compute and communications

### 3.1 Teensy 4.1 — flight computer ×1

| Parameter | Value |
|---|---|
| MCU | NXP i.MX RT1062, **ARM Cortex-M7 @ 600 MHz**, hardware FPU |
| RAM / Flash | 1024 KB / 8 MB |
| I/O | 55 pins, 35 PWM-capable |
| Serial / SPI / I²C | 8 / 3 / 3 |
| CAN | 2× CAN 2.0B + **1× CAN-FD** |
| **Storage** | **built-in microSD socket** |
| Logic | **3.3 V, NOT 5 V tolerant** |
| Dimensions | 61 × 18 mm |
| Toolchain | Arduino IDE + Teensyduino |

Peripheral budget: 3 SPI buses covers a dedicated IMU bus with room to spare; the microSD is our high-rate logging path (see [[System Integration - Full Cube]] §2.6).

### 3.2 CAN-FD adapter ×1
Transceiver breakout for the Teensy's CAN-FD peripheral. The Teensy provides the controller; this provides the physical layer to the bus.

### 3.3 mjbots mjcanfd-usb-1x ×1
USB↔CAN-FD adapter. **Ground support only** — laptop taps the same bus for `moteus_tool` calibration and `tview` debugging. Not on the cube in the final demo.

### 3.4 Seeed XIAO ESP32-C6 + antenna ×1

| Parameter | Value |
|---|---|
| MCU | ESP32-C6, RISC-V |
| Radio | Wi-Fi 6, BLE 5, Zigbee/Thread |
| Dimensions | ~21 × 17.5 mm |
| Logic | 3.3 V |

Telemetry only, on UART as a **side branch**. Never in the control loop — a blocking Wi-Fi send that stalls the 1 kHz loop is a fall.

### 3.5 JST PH3 cables ×≥4
3-pin, 2.0 mm pitch. CAN daisy chain: CAN_H, CAN_L, GND. Each moteus ships with one housing + crimps.

---

## 4. Power

### 4.1 Turnigy Heavy Duty 6S 3600 mAh 60C

| Parameter | Value |
|---|---|
| Chemistry / cells | LiPo, **6S** (6 cells in series) |
| Nominal voltage | 22.2 V |
| **Full charge** | **25.2 V** ← the number that kills 16 V capacitors |
| Min safe | 19.8 V (3.3 V/cell); 18 V absolute floor |
| Capacity | 3600 mAh ≈ **80 Wh** |
| Discharge | 60C ≈ 216 A (vastly more than we need) |
| Length | **139 mm** ← hard structural constraint |
| Connector | XT90 |

⚠️ **Verify by weighing** — estimated ~530–600 g from energy density, which would be ~40% of the cube's mass. Also **measure the other two dimensions** (width, height) for diagonal-fit clearance.

**Worth asking the mentor:** a 1500 mAh 6S pack would weigh ~220 g and be shorter. Our draw is ~10–50 W; 80 Wh runs the cube for over an hour when we need 20–30 minutes. That single swap saves ~300 g and relaxes both constraints.

### 4.2 LM2596 buck converter ×1

| Parameter | Value |
|---|---|
| Input | 4.5–40 V |
| Output | adjustable 1.23–37 V |
| Current | 3 A peak, **~2 A continuous realistic** without heatsinking |
| Switching | 150 kHz |
| Efficiency | ~75–88 % |

⚠️ At 22 V → 5 V the step-down ratio is large and it will **get hot**. Budget the 5 V loads (Teensy, ESP32, servo stall, IMU) and check against 2 A. If tight, a modern synchronous buck would be a worthwhile upgrade.

### 4.3 XT90 connector pair
~90 A rated. ⚠️ **The BOM has XT90 only, but each moteus uses XT30** — need 3+ XT30 pairs. Also strongly recommend an **XT90-S anti-spark** variant for the main connection: connecting bulk capacitance to a charged 6S pack draws a large inrush and arcs the contacts.

### 4.4 1N5819 Schottky diode ×1–2
40 V, 1 A, forward drop ~0.45 V. Uses: reverse-polarity protection, power-ORing between bench supply and battery, or servo flyback. ⚠️ Confirm intent — 40 V is fine on a 25.2 V bus but **1 A limits it to signal-level duty**, not the motor bus.

---

## 5. Passives and hardware

### 5.1 CAN termination: 60.4 Ω ×4 + 4.7 nF ×2 — **split termination**

Standard CAN needs 120 Ω at each end of the bus. **Split termination** replaces each 120 Ω with two 60.4 Ω in series and a capacitor from the midpoint to ground:

```
CAN_H ──[60.4Ω]──┬──[60.4Ω]── CAN_L
                 │
              [4.7nF]
                 │
                GND
```

2 × 60.4 = 120.8 Ω ✓ (60.4 Ω is the standard E96 value nearest 60). The capacitor shunts **common-mode noise** to ground — meaningfully better EMC than plain termination, which matters with three switching motor drivers on the same harness.

BOM quantities confirm the intent: **4 resistors + 2 capacitors = 2 networks**, one at each physical end of the bus.

### 5.2 Capacitors

| Part | Purpose | Note |
|---|---|---|
| **100 µF 16 V** electrolytic, 105 °C | **5 V rail only** | ⚠️ **Cannot touch the motor bus** — 25.2 V full charge would destroy it. 105 °C rating is good (long life). |
| **100 nF 50 V** ×6–10 | decoupling, per IC | standard practice |
| **4.7 nF** ×2 | CAN split termination | see §5.1 |

⚠️ **Missing from the BOM:** motor-bus **bulk capacitance, 470–1000 µF, ≥50 V, low-ESR**. Decelerating wheels regenerate onto the bus; without bulk capacitance the voltage spikes and drivers fault — which looks exactly like a random controller failure. High-consequence gap.

### 5.3 Perfboard 15 × 9 cm, double-sided
Power distribution and interconnect. Double-sided helps for a ground plane.

---

## 6. Mass budget (electronics)

| Item | Qty | Unit | Total |
|---|---|---|---|
| MN4006 motor | 3 | 66 g | 198 g |
| moteus-n1 | 3 | 14.6 g | 44 g |
| MA600 breakout + magnet | 3 | ~2 g* | 6 g |
| Teensy 4.1 | 1 | ~11 g* | 11 g |
| CAN-FD adapter | 1 | ~5 g* | 5 g |
| XIAO ESP32-C6 | 1 | ~3 g* | 3 g |
| BMI270 breakout | 1 | ~3 g* | 3 g |
| MG92B servo | 1 | 13.8 g | 14 g |
| LM2596 module | 1 | ~10 g* | 10 g |
| Perfboard + passives | — | — | ~40 g* |
| Wiring, connectors | — | — | ~50 g* |
| **Subtotal (no battery)** | | | **~384 g** |
| **Battery** | 1 | ~550 g* | **~550 g** |
| **Total electronics** | | | **~934 g** |

\* estimate — **weigh everything**. Add wheels (~180–270 g) and structure (~250 g) on top.

⚠️ This lands the cube near **1.3–1.4 kg**, well above the 900 g working target. The battery alone is ~40%. This is the single strongest argument for asking about a smaller pack.

---

## 7. Open items

- [ ] **Weigh the battery**; measure width and height
- [ ] Confirm motor quantity — 3 or 4 (spare)?
- [ ] Confirm the **servo's purpose** with the mentor
- [ ] Confirm SparkFun BMI270 breakout exposes **SPI**
- [ ] Confirm BMI270 configured **ODR**
- [ ] Check **moteus firmware ≥ 2024-10-29** for MA600 support
- [ ] Confirm whether moteus reports **bus voltage** in the reply frame
- [ ] Source: **XT30 pairs ×3**, **≥50 V bulk capacitor**, XT90-S anti-spark, inline fuse
- [ ] Confirm 1N5819 intent
- [ ] Decide MA600 mounting: **off-axis with ring magnet** (recommended) vs on-axis
