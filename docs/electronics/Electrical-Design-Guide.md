---
tags: [space-challenge, sofia, cubli, electronics, schematic, kicad, wiring]
---

# Electrical Design Guide — schematic, wiring, KiCad

Complete reference for drawing the electronic (schematic) and electrical (wiring) diagrams. Every connection, pin, and passive.

Components: [[Component Reference]]. Architecture rationale: [[System Integration - Full Cube]]. Physical placement: [[Structural Constraints]].

---

## 1. Architecture in one line

**Teensy 4.1 → CAN-FD → 3× moteus-n1 → 3× MN4006**, each driver closing its own current loop with its own MA600 encoder. One BMI270 IMU on SPI. ESP32-C6 on UART for telemetry only. Everything from one 6S pack via two rails.

Four subsystems, drawn as four schematic sheets:
1. **Power** — battery, protection, motor bus, 5 V rail
2. **CAN bus** — transceiver, chain, termination
3. **Compute & sensing** — Teensy, IMU, ESP32
4. **Drivers** — 3× moteus with encoders (mostly connectors)

---

## 2. Power distribution

```
6S LiPo  (22.2 V nom / 25.2 V full / 18 V floor)
   │
   ├─ XT90-S anti-spark  ← inrush limiting on connect
   ├─ Main switch / E-STOP    ⚠ must break the MOTOR bus, not just logic
   ├─ Fuse ~30 A slow-blow
   │
   ├──► MOTOR BUS (22–25 V) ──┬── XT30 ──► moteus #1
   │         │                ├── XT30 ──► moteus #2
   │         │                └── XT30 ──► moteus #3
   │         └── BULK CAP 470–1000 µF ≥50 V low-ESR   ⚠ NOT IN BOM
   │
   └──► LM2596 (22 V → 5 V) ──► 5 V RAIL
              │                    ├──► Teensy VIN
              │                    ├──► ESP32-C6 5 V pin
              │                    └──► Servo V+  (local 100 µF)
              └── 100 µF 16 V on OUTPUT side only
                                   │
                     Teensy 3.3 V out (≤250 mA) ──► BMI270, CAN transceiver
```

### 2.1 ⚠️ Two hazards to get right

**The 100 µF 16 V capacitors go on the 5 V rail only.** A fully charged 6S pack is **25.2 V**. A 16 V electrolytic on the motor bus will fail, possibly violently. Mark this on the schematic explicitly.

**The motor-bus bulk capacitor is missing from the BOM.** Decelerating wheels regenerate onto the bus; without bulk capacitance the voltage spikes and drivers fault out — presenting as *random controller failures* that look like software bugs. Source **470–1000 µF, ≥50 V, low-ESR**, and place it physically close to the moteus power taps, not at the battery end.

### 2.2 LM2596 loading

| Load | Typical | Peak |
|---|---|---|
| Teensy 4.1 @600 MHz | ~150 mA | 200 mA |
| ESP32-C6 | ~100 mA | 300 mA (Wi-Fi TX) |
| BMI270 | ~1 mA | — |
| Servo MG92B | ~10 mA idle | ~1 A stall |
| **Total** | **~400 mA** | **~1.5 A** |

Within the LM2596's realistic 2 A continuous, but at 22→5 V it dissipates real power and will run warm. **Give the servo its own local 100 µF** so stall inrush doesn't sag the Teensy rail and reset the MCU mid-balance.

### 2.3 Wire gauge and protection

| Run | Gauge |
|---|---|
| Battery → bus | 14 AWG |
| Bus → each moteus | 18 AWG |
| 5 V rail | 22 AWG |
| Signal | 26–28 AWG |

- **Fuse:** ~30 A slow-blow. Must survive jump-up spin-up current without nuisance blowing.
- **E-stop:** must interrupt the motor bus. A logic-only kill leaves the wheels spinning.
- **1N5819 (40 V, 1 A):** signal-level only — reverse-polarity protection on the 5 V rail or servo flyback. **Cannot protect the motor bus.**

---

## 3. CAN-FD bus

### 3.1 The adapter — confirmed part

**Fusion Tech "Single CAN-FD adapter for Teensy 4.1"** (Tindie). Confirmed specs:

| Property | Value |
|---|---|
| Transceiver | **TI TCAN330G** — CAN-FD to **5 Mbps** ✓ matches moteus |
| Teensy interface | **CAN3 controller, pins 30 / 31** ✓ the only FD-capable one |
| Termination | **120 Ω onboard**, with a **cuttable trace** to disable |
| Library | **FlexCAN_T4** (bundled with Teensyduino) — `github.com/tonton81/FlexCAN_T4` |
| Form factor | backpack, solders onto the Teensy; 3-pin 0.1″ header for the bus |

Backwards compatible with CAN 2.0, so no speed compromise is needed anywhere.

### 3.2 Topology — ONE termination network, not two

Because the adapter is already terminated:

```
[adapter: 120 Ω onboard] ── moteus#1 ── moteus#2 ── moteus#3 ── [2× 60.4 Ω + 4.7 nF]
                                             │
                                  (USB adapter taps mid-chain for debug)
```

⚠️ **Build only ONE split-termination network, at moteus #3.** Fitting both ends would put ~40 Ω on the bus and communication fails outright.

⚠️ **The adapter must sit at a physical END of the chain.** It carries termination, so mid-chain placement breaks the bus. Leave its trace intact for this topology; only cut it if the adapter has to move elsewhere.

Leftover from the BOM: **2× 60.4 Ω + 1× 4.7 nF spare** — keep as rebuild stock.

**Rules:**
- Linear chain, **never a star**.
- Stubs under ~30 cm at 5 Mbps.
- CAN_H / CAN_L as a **twisted pair**, GND alongside.
- Route away from motor phase wires; cross at right angles where unavoidable.

### 3.3 Split termination (at moteus #3 only)

```
CAN_H ──[60.4 Ω]──┬──[60.4 Ω]── CAN_L
                  │
               [4.7 nF]
                  │
                 GND
```

2 × 60.4 Ω = 120.8 Ω ✓ (60.4 Ω is the E96 value nearest 60). The capacitor shunts **common-mode** noise to ground — better EMC than plain 120 Ω, which matters with three switching drivers sharing a harness.

### 3.4 ⚠️ Connector mismatch — plan this before soldering

The adapter's bus side is a **3-pin 0.1″ (2.54 mm) header**. The moteus uses **JST PH3 at 2.0 mm pitch**. **These do not mate.**

Options:
1. Re-crimp one end of a PH3 cable into a 3-way Dupont housing (cleanest).
2. Solder a female header strip to the PH3 cable's bare end.
3. Solder the cable's bare wires directly to the adapter pins, with heatshrink.

First check whether your 30 cm PH3 cables have connectors on **both** ends or one end flying — if one end is bare, option 3 is immediate.

⚠️ **Confirm pin order on both sides before soldering.** A swapped CANH/CANL is *silent* — the bus simply doesn't communicate, with no error pointing at the cause. Write the pinout on the schematic sheet.

### 3.5 Adapter connections

| Adapter | Teensy | Note |
|---|---|---|
| TXD | pin 31 | CAN3 TX |
| RXD | pin 30 | CAN3 RX |
| VCC | 3.3 V | ~35 mA, safe from Teensy regulator |
| GND | signal ground | |
| CANH / CANL / GND | 3-pin header → bus | see §3.4 |

Since it mounts directly on the Teensy, the first four are made by the header — verify they land on 30/31 and not 22/23 or 0/1.

---

## 4. Teensy 4.1 pin map

| Pin | Function | Connects to | Note |
|---|---|---|---|
| 30 | CAN3 RX | Transceiver RXD | **CAN3 is the only FD-capable peripheral** |
| 31 | CAN3 TX | Transceiver TXD | |
| 10 | SPI0 CS | BMI270 CS | |
| 11 | SPI0 MOSI | BMI270 SDI | |
| 12 | SPI0 MISO | BMI270 SDO | |
| 13 | SPI0 SCK | BMI270 SCK | also onboard LED — will flicker |
| 9 | Digital in | BMI270 INT1 | data-ready interrupt |
| 0 | Serial1 RX | ESP32 TX | |
| 1 | Serial1 TX | ESP32 RX | cross-connect |
| 2 | PWM | Servo signal | 50 Hz, 1–2 ms |
| 3 | Digital in | E-stop sense | optional, pull-up |
| 4 | Digital out | Status LED | + series resistor |
| 14 (A0) | Analog in | Battery divider | optional; moteus also reports bus V |
| VIN | 5 V in | LM2596 out | 3.6–5.5 V range |
| 3.3 V | 3.3 V out | IMU, transceiver | ≤250 mA total |
| GND | Ground | signal ground | multiple pins, use several |

**CAN1 and CAN2 are classic CAN only** — do not use them for the moteus bus. The built-in microSD uses dedicated SDIO pins and consumes none of the above.

---

## 5. Sensing and auxiliaries

### 5.1 Encoders — not on the Teensy

Each **MA600 → its own moteus AUX2** (JST GH-7), using the supplied GH6→GH7 cable. The moteus supplies 3.3 V over the same cable.

⚠️ **AUX2 SPI runs at 6 MHz.** Keep each run **under 20 cm**, and route away from that motor's phase wires. This is the constraint that forces each moteus to sit near its own motor.

Requires moteus firmware **2024-10-29 or newer** for MA600 support.

### 5.2 BMI270 IMU

**SPI, not I²C** — I²C at 1 kHz with the noise environment is marginal.

| BMI270 | Teensy |
|---|---|
| CS | 10 |
| SDI (MOSI) | 11 |
| SDO (MISO) | 12 |
| SCK | 13 |
| INT1 | 9 |
| VDD / VDDIO | 3.3 V |
| GND | signal ground |

Add **100 nF** at its supply pin. Configure gyro to **±2000 °/s** — jump-up spins fast and clipping destroys the estimate.

⚠️ Verify the SparkFun Qwiic breakout **exposes the SPI pins** — it is I²C-first.

### 5.3 Servo (MG92B)

| Servo | To |
|---|---|
| Signal (orange) | Teensy pin 2 |
| V+ (red) | 5 V rail, **not** Teensy 3.3 V |
| GND (brown) | signal ground |

Local **100 µF** at the servo connector. Optional 1N5819 across the supply for flyback.

### 5.4 ESP32-C6

| ESP32 | Teensy |
|---|---|
| TX | pin 0 (RX1) |
| RX | pin 1 (TX1) |
| 5 V | 5 V rail |
| GND | signal ground |

Telemetry only — **never in the control path**. Teensy writes must be non-blocking; a stalled Wi-Fi send that blocks the 1 kHz loop is a fall.

---

## 6. Passive placement

| Part | Qty | Where | Why |
|---|---|---|---|
| 100 µF 16 V | 2–3 | 5 V rail output; one at servo | bulk smoothing ⚠️ **never on motor bus** |
| 100 nF 50 V | 6–10 | at every IC supply pin | high-frequency decoupling |
| 60.4 Ω | 4 | 2 per CAN end | split termination |
| 4.7 nF | 2 | 1 per CAN end, midpoint→GND | common-mode shunt |
| 1N5819 | 1–2 | 5 V rail / servo | reverse polarity, flyback |
| **≥50 V bulk** | 1 | **motor bus** | **regen absorption — TO SOURCE** |

Decoupling caps go **as close to the pin as physically possible** — a 100 nF 10 mm away does much less than one at 2 mm.

---

## 7. Grounding — the star

The moteus drivers switch tens of amps. If that return current flows through signal ground, it injects noise into the IMU and CAN, and the symptom is an estimator that works on the bench and fails on the cube.

```
Battery −  ──►  POWER GROUND BUS   (heavy copper / thick wire on perfboard)
                  ├── moteus #1 GND   (short, direct)
                  ├── moteus #2 GND
                  ├── moteus #3 GND
                  ├── bulk capacitor −
                  └── LM2596 input −
                          │
                    ═══ ONE LINK ═══   ← the ONLY connection
                          │
                SIGNAL GROUND ──┬── Teensy GND
                                ├── BMI270 GND
                                ├── ESP32 GND
                                ├── CAN transceiver GND
                                ├── CAN bus GND (to all moteus)
                                └── servo GND
```

Rules:
- **One** connection between power and signal ground, near the LM2596.
- Never daisy-chain signal grounds through a driver.
- CAN GND is part of the signal ground — it is the shared reference for the differential pair.
- On the perfboard, run a wide ground trace or a copper strip; do not rely on thin jumpers.

---

## 8. KiCad resources

### 8.1 Where to get symbols, footprints, 3D models

| Source | Covers | Notes |
|---|---|---|
| **SnapMagic Search** (was SnapEDA) — snapeda.com | Teensy 4.1, most parts | Free, KiCad export incl. 3D |
| **ComponentSearchEngine** (SamacSys) | Teensy 4.1, generic parts | Free with account; symbol + footprint + 3D |
| **github.com/Seeed-Studio/OPL_Kicad_Library** | **XIAO ESP32-C6** (SMD + DIP) | Official Seeed; see gotcha below |
| **github.com/sparkfun/SparkFun-KiCad-Libraries** | SparkFun breakouts incl. BMI270 | Official; symbols + footprints + open 3D models |
| **github.com/XenGi/teensy_library** + **teensy.pretty** | Teensy symbols / footprints | Community, KiCad-native |
| **KiCad built-in libraries** | R, C, D, connectors, headers | Already installed |

### 8.2 ⚠️ The XIAO footprint gotcha

Seeed's repo ships **standalone `.kicad_mod` files** (`XIAO-ESP32C6-SMD.kicad_mod`, `XIAO-ESP32C6-DIP.kicad_mod`). KiCad 6/7/8/9 require footprints to live **inside a `.pretty` directory** to be recognised.

Fix: create a folder named e.g. `XIAO.pretty/`, move the `.kicad_mod` files into it, then add that folder via *Preferences → Manage Footprint Libraries*. Symbols come from `Seeed_Studio_XIAO_Series.kicad_sym` via *Manage Symbol Libraries*.

Use the **DIP variant** if socketing on headers (recommended — replaceable), SMD if soldering directly.

### 8.3 Parts with no ready-made library

**moteus-n1, MA600 breakout, LM2596 module, XT90/XT30** — likely need custom symbols. Since these are **modules that mount off-board or plug in**, the pragmatic approach for a perfboard build:

- Draw them as **generic connectors** in the schematic (e.g. `Conn_01x03` for a JST PH3, `Conn_01x02` for XT30) rather than full component symbols.
- The moteus is a separate board — represent it as a **connector block** with its power (XT30), CAN (PH3 in/out) and AUX2 (GH-7) pins.
- For 3D fit checking, grab STEP files where available: mjbots publishes mechanical drawings; XT connectors are on GrabCAD/Thingiverse.

For a **perfboard** build you don't strictly need footprints at all — the schematic is the deliverable, and the physical layout is hand-wired. Only invest in footprints if you move to a fabricated PCB.

### 8.4 Suggested schematic sheet structure

```
root.kicad_sch
 ├── 1_power.kicad_sch      battery, fuse, e-stop, bulk cap, LM2596, rails
 ├── 2_can.kicad_sch        transceiver, chain connectors, 2× termination
 ├── 3_compute.kicad_sch    Teensy, IMU, ESP32, servo, LED
 └── 4_drivers.kicad_sch    3× (XT30 + PH3 in/out + GH-7 encoder)
```

Use **global labels** for rails (`+5V`, `+3V3`, `VBAT`, `GND`, `GNDP`) and for `CANH` / `CANL`. Keeping power ground (`GNDP`) and signal ground (`GND`) as **separate nets joined at one symbol** makes the star grounding explicit in the schematic and prevents the layout tool from stitching them everywhere.

---

## 9. Bring-up sequence

Never power a new build from the battery. Use a **current-limited bench supply** set to ~1 A initially — the difference between a bug and a fire.

1. **Power only.** No modules fitted. Verify 5 V and 3.3 V rails, correct polarity, no shorts. Check LM2596 temperature.
2. **Teensy alone.** Blink. Confirm it enumerates over USB.
3. **IMU.** Read WHO_AM_I over SPI. Then stream raw gyro/accel.
4. **CAN, one driver.** Connect one moteus, confirm `tview` over the USB adapter sees it. Verify termination with a multimeter: **~60 Ω across CAN_H/CAN_L with power off** (two 120 Ω networks in parallel).
5. **Encoder.** Rotate the motor by hand, confirm a monotonic reading over a full revolution **with the wheel installed**.
6. **Torque mode.** Command a known torque, log acceleration — this is jig test M2 and confirms the whole architecture assumption.
7. **All three drivers.** Check bus utilisation and that arbitration is clean.
8. **Servo, ESP32** last — they are not on the critical path.

---

## 10. To verify before finalising the schematic

**Resolved:**
- ✅ CAN adapter is **CAN-FD at 5 Mbps** (TCAN330G) — confirmed from the product page
- ✅ Adapter has **120 Ω onboard termination**, cuttable trace → build only one split network
- ✅ Uses **CAN3, pins 30/31** — pin map confirmed
- ✅ Library: **FlexCAN_T4**

**Still open:**
- [ ] Does the **moteus-n1 have its own onboard termination**? If yes, reconsider the moteus #3 network.
- [ ] Does the **SparkFun BMI270 breakout expose SPI pins**? It is a Qwiic/I²C-first board.
- [ ] Exact **moteus PH3 pinout** (which pin is CANH / CANL / GND) — from mjbots docs, not assumed.
- [ ] Do the **30 cm PH3 cables have connectors both ends**, or one flying? Decides the §3.4 approach.
- [ ] **1N5819 intended role** — confirm with whoever specced it.
- [ ] Source: **≥50 V bulk capacitor**, **3× XT30 pairs**, **XT90-S anti-spark**, **inline fuse**.
- [ ] Confirm **moteus firmware ≥ 2024-10-29** for MA600 support.
