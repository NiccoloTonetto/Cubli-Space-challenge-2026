# Stage 5 — The same test, powered from the bench supply

**Power:** cube power **ON** (logic rail only — motor bus **dead**). **No USB
cable on either board.**
**Can anything move?** No. The faker still never opens CAN, and the motor bus
is off anyway.
**New code:** none. Same faker, same bridge, same script as Stage 4.

## Why this is a separate stage

Stage 4 passing on USB power does **not** prove the link works on the rail.
Everything electrical changes:

| | USB power (Stage 4) | Rail power (Stage 5) |
|---|---|---|
| 5 V source | laptop, stiff, well filtered | LM2596 at 22 → 5 V, switching |
| ground | shared through the laptop | star ground near the LM2596 |
| XIAO current | irrelevant | ~100 mA typical, **300 mA peaks on WiFi TX** |
| diagnostics | both USB monitors | UDP only — no `[status]`, no `[faker]` |

The WiFi TX peak is the one that bites. If the rail sags on every transmit
burst, the XIAO brown-out resets, rejoins over ~2–3 s, and you see telemetry
that "works but keeps cutting out" — with no serial port attached to tell you
why. That is worth ten minutes of deliberate testing now rather than during a
balance run.

## ⚠️ Power rules — the constraint you actually have

You said it plainly: with the supply connected, **the Teensy and the XIAO both
get 5 V and you cannot separate them.** That is exactly why this ladder is
built the way it is.

**Never have cube power and USB connected at the same time.** Both `Teensy
VIN` and the XIAO's `5V` pin are tied to their board's USB 5 V internally, so
plugging in USB while the rail is live puts the **laptop's 5 V directly in
parallel with the LM2596's 5 V** — two stiff sources, neither current-limited
toward the other. Whichever sits higher back-feeds the other. That can damage
the laptop's USB port, the LM2596, or a board.

**The good news: your constraint blocks nothing.** Stages 0–4 run entirely on
USB power with the rail off, and Stage 5 runs entirely on rail power with no
USB. You never need both — that is the whole design of this sequence.

### Correct order, every time

| | |
|---|---|
| **Power up** | cube power **OFF** → plug USB → flash → **unplug USB** → cube power **ON** |
| **Power down** | cube power **OFF** first → *then* USB, if you need it |
| **Wiring** | all power off before touching D6/D7/GND. Hot-plugging a signal wire back-feeds an unpowered board through its clamp diodes. GND connects first, disconnects last. |

### If you want USB *and* rail power anyway

Ranked by how much it buys you for the effort:

1. **Put a jumper (or a switch) in the XIAO's 5 V feed.** Two-pin header
   between the rail and the XIAO's `5V` pin. Ten minutes, and it turns "I
   cannot separate them" into "I pull one jumper". This is the fix worth
   making — it also lets you use `link_check.py --serial`, the only
   *conclusive* directional test in the whole toolkit, without powering the
   cube down.
2. **A data-only USB cable** (VBUS conductor cut or omitted). The board runs
   from the rail, the cable carries D+/D− only, nothing back-feeds. Verify it
   at Stage 0 before you rely on it: plug it into a *rail-powered* XIAO and
   confirm the port still enumerates. If it does not, fall back to option 1.
3. **Cut the Teensy's VUSB↔VIN pad** on the underside. PJRC documents this as
   the supported way to combine external power with USB. Consequence: the
   Teensy will then **not run from USB alone** — it needs the rail present to
   power up at all. This only fixes the Teensy; the XIAO still needs 1 or 2.

## Before powering the rail

- [ ] Motor bus **dead** (e-stop open, or XT30s unplugged). If your e-stop
      breaks the motor bus only and leaves logic live, that is exactly the
      configuration for this stage.
- [ ] Bench supply set to the pack voltage the LM2596 expects (~22 V), with a
      **current limit** set — a few hundred mA is plenty for logic-only, and it
      turns a wiring mistake into a tripped limit instead of smoke.
- [ ] `100 µF` bulk capacitor present on the 5 V rail output, per
      [`Electrical-Design-Guide.md`](../../../../../../../docs/electronics/Electrical-Design-Guide.md)
      §2.2. This is what absorbs the WiFi TX peaks.
- [ ] **No USB cable on either board.**
- [ ] D6/D7/GND jumpers still connected, untouched since Stage 1.

## Run

1. Rail on. Both boards boot with no cable attached.
2. `python stage4_rate_check.py --seconds 120`
3. `python ../../link_check.py` (network-only mode; `--serial` needs option 1
   or 2 above).

## Pass

The same numbers as Stage 4, held for **two minutes**:

```
rate 249.6/s   loss 0.2%   longest gap 2 lines
```

Same rate, same loss as on USB power. **Any difference is the rail**, not the
network — the network did not change.

## If it fails

| Symptom | Cause |
|---|---|
| Repeating gaps of **hundreds** of lines, every few seconds | The XIAO is **brown-out resetting** and rejoining WiFi. Rail sag on TX peaks: add/increase bulk capacitance, check the supply's current limit, check 22 AWG on the 5 V run and the crimps. |
| Nothing at all, ever | Measure 5 V at the XIAO's own pin, not at the LM2596. A long thin wire or a bad crimp drops it below the board's minimum. |
| Works, but loss much worse than Stage 4 | Switching noise or a ground problem. Confirm the star ground: **one** connection between power and signal ground, near the LM2596. |
| Teensy alive, XIAO dead (or the reverse) | Whichever is dead is not actually being fed 5 V. Measure at the pin. |
| Everything dead the moment the rail comes up | Stop. Check polarity and the 5 V/GND wiring before anything else. |

Then: with a scope or a multimeter on the 5 V rail, watch it during a run.
Sag below ~4.5 V on transmit bursts explains every intermittent symptom above.

→ Then [Stage 6](../Stage6_RealFirmware/) — the real firmware, motors still
dead.
