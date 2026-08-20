# Stage 4 — The whole pipeline at the real data rate, no control law

**Power:** cube power **OFF**. Both boards on their **own USB cable**, into the
same laptop. Jumpers connected. Nothing touches the LM2596 rail.
**Can anything move?** **No** — the faker never opens CAN. There is no CAN
traffic on the bus at all.

## Proves

`Teensy → Serial1 @ 1 Mbaud → XIAO → WiFi/UDP → laptop`, exercised exactly the
way `CornerBalance_WiFi.ino` will exercise it: **21 CSV fields, 250 lines/s**,
the same command grammar, the same `#` console lines — with the moteus, the
BMI270, the estimator, the control law and arming all absent.

That is the point. When this passes, the transport is a known-good constant.
The first time you run the real firmware, anything wrong is in the firmware.
Skip this stage and you debug both at once, on a cube that can throw itself
off the bench.

**The XIAO runs the real bridge from here on** —
[`../../xiao_teensy_bridge/xiao_teensy_bridge.ino`](../../xiao_teensy_bridge/xiao_teensy_bridge.ino).
Flash it now. Stage 4 is the last chance to test that sketch against a data
source that cannot hurt anyone.

## Run

1. XIAO ← `../../xiao_teensy_bridge/xiao_teensy_bridge.ino` (same five config
   values you used in Stages 2–3).
2. Teensy ← `teensy_csv_faker/teensy_csv_faker.ino`.
3. `python stage4_rate_check.py`

```bash
python stage4_rate_check.py                  # 20 s
python stage4_rate_check.py --seconds 120    # soak it
python stage4_rate_check.py --command a1     # also test laptop -> Teensy
```

Then, once it passes, point the **real plotter** at the faker:

```bash
python ../../telemetry_python_wifi_corner.py
```

The panels fill with slow sinusoids. That proves matplotlib, the CSV parsing,
the 21-column layout, the command console and the CSV-on-close — before any of
it is load-bearing. Type `a1` and watch the `armed` column flip.

## Pass

```
rate 249.6/s   loss 0.2%   longest gap 2 lines   out-of-order 0
```

Under ~1 % loss, in ones and twos, is normal UDP-over-WiFi and plots perfectly
smoothly.

### Exact loss, not an average

The faker sets `t_ms = seq*4`, so `seq = t_ms/4` is recoverable from every
line. "Telemetry looks choppy" becomes a number — and, more usefully, a
*shape*: singles mean radio noise, long runs mean the bridge stalled or the
WiFi roamed. A packets-per-second average hides both.

## If it fails

| Symptom | Cause |
|---|---|
| rate collapses to **~0.5/s** | The XIAO's USB is plugged in with nothing reading it. Its CDC blocks ~2 s per print; the bridge relays one line every two seconds. See [the CDC stall](../README.md#the-cdc-stall-the-one-that-wastes-the-most-time). |
| loss in **long runs** | WiFi roaming or the bridge falling behind. Check RSSI on the `[status]` line. |
| rate ≈ 125/s, loss ≈ 50 % | `loop()` running at half the needed rate — one line is relayed per iteration. |
| 0 lines but `#` answers arrive | Commands work, telemetry does not: the Teensy is in `t0` (USB) mode. Send `t1`. |
| 0 lines, and the Teensy says `lines_out=0 tx_skipped=250` | The Teensy never got a byte onto the wire. Serial1's TX buffer is smaller than one line, so the room check in `emitLine()` rejects all of them — Teensy 4.x defaults to **40 B** against a ~200 B line. Fixed by `Serial1.addMemoryForWrite()` in `setup()`; if you see this, you are running an older faker. |
| 0 lines, nothing at all | Back to Stage 3. If Stage 3 passes, the fault is Serial1 → back to Stage 1. |
| `--command a1` gets no `#` reply | The laptop → Teensy direction is broken: XIAO D6 → Teensy pin 0. Stage 1 tests exactly this. |

### Which side is dropping — the one cross-check worth doing

Compare the Teensy's own USB `[faker]` line against the script:

| | Meaning |
|---|---|
| `lines_out` = lines received | perfect |
| received is lower | lost **after** the Teensy — the XIAO relay or the air link |
| `tx_skipped` nonzero | lost **before** the XIAO — the Serial1 TX buffer was full, the Teensy could not even hand the line over |

From the laptop alone those are indistinguishable, which is why the faker
counts them.

→ Then [Stage 5](../Stage5_RailPower/) — the same test, on the bench supply.
