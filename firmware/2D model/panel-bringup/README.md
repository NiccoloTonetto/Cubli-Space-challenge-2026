# Panel Bring-Up — Teensy 4.1 / moteus-n1 / BMI270

Staged Arduino IDE firmware for bringing the Stage 1 planar panel rig up
from "nothing tested" to closed-loop balancing, one confidence-building
step at a time. Companion to [`../../docs/simulation/`](../../docs/simulation/)
(plant, LQR derivation, validation gates) — this is the hardware side of
that pipeline.

## Files

| File | Motor can move? | What it tests |
|---|---|---|
| `test_control.txt` | No — hard-wired off | Stage 0: estimator only (IMU sign, mount offset, complementary filter) |
| `Stage1_OpenLoopTorque/` | Yes — fixed pulse only | First real torque command; the two sign checks that matter most before anything closes the loop |
| `Stage2_DampingOnly/` | Yes — closed loop | Rate feedback only (`k2`), full safety scaffold introduced |
| `Stage3_PositionDamping/` | Yes — closed loop | Adds position feedback (`k1`), gain ramped 0.1→1.0 by hand |
| `Stage4_FullLaw/` | Yes — closed loop | Adds momentum management (`k3`) — wheel should unwind after corrections |
| `Stage5_Release/` | Yes — closed loop, unsupported | Panel released with rails + e-stop; the actual balancing attempt |

`test_control.txt` stays a loose file (not `.ino`) by design — everything
else is a self-contained Arduino IDE sketch in a folder named after itself,
so each opens directly.

## Docs

- **`Arduino Bring-Up Plan — Sections 2b and 2d.md`** — the original
  procedure/checklists these sketches implement, stage by stage.
- **`Bring-Up Stages — Implementation Notes.md`** — what each file's code
  actually does and why, including two real bugs found and fixed during
  hardware bring-up on this rig:
  - `SetPosition()`'s default wire format silently drops every `Command`
    field except `position`/`velocity` — torque commands were never
    reaching the moteus until a custom `Format` was passed explicitly.
  - The wheel's physical mounting convention was backwards relative to the
    LQR model's sign convention (confirmed via Stage 1 sign check 4) —
    fixed with a single `kWheelSign` constant at the hardware boundary,
    not by touching the derived gains.

Both are worth reading before assuming any of this "just works" on a
different rig — they compiled cleanly and looked correct; only real
hardware testing caught either one.

## Gains currently deployed

```
K = [-1.0998, -0.1232, -0.001732]   (N*m/rad, N*m/(rad/s), N*m/(rad/s))
```

From `cubli_panel_simscape_gates.m` Gate 7, LQR on the measured-mode plant
(`../../matlab/cubli_panel_params.m`). Closed-loop poles for this `K`:
≈ **-12.5, -9.7, -6.3 rad/s** (all real) — see the Implementation Notes for
where two different `-15/-9/-6`-style figures floating around elsewhere
came from and why they were wrong.

Retuning these follows [Panel Controller Workflow](../../docs/simulation/Panel-Controller-Workflow.md)
step 4: measure `Theta` from a free-swing test on the built panel, update
the plant, re-run `lqr()`, re-flash. See the Implementation Notes' closing
section on fine-tuning for the fuller version of that process.
