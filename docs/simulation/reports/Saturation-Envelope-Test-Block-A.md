---
tags:
  - space-challenge
  - sofia
  - cubli
  - simscape
  - simulation
  - panel
  - controls
  - validation
  - saturation
---

#---

tags:

- space-challenge
- sofia
- cubli
- simscape
- simulation
- panel
- controls
- validation
- saturation

---

# Saturation Envelope Test — Block A

First of the nonlinear tests from §6 of [[Simscape Panel Model — Build Guide]]. Answers one question: **with a real actuator, how far can the panel tip and still come back?**

**Result: 11–12°, and the limit is the wheel-speed cap, not the motor.**

Script: `cubli_panel_saturation.m` Depends on: `cubli_panel_params.m` (measured mode), and the LQR cell of `cubli_panel_simscape_gates.m` for `p` and `K_lqr`.

---

## 1 — Why this test exists

Gates 1–6 validated the _plant_. Closing the loop with LQR showed the _control law_ is sound. Both assume an actuator that delivers whatever torque the maths asks for.

The MN4006 does not, and our own firmware imposes a tighter wheel-speed cap on purpose. So there is an angle beyond which no amount of correct control law helps, and finding it is the difference between "the controller works" and "the controller works on our hardware".

Three analytical bounds already existed. None of them is the answer.

|Bound|Formula|Value|What it assumes|
|---|---|---|---|
|Naive torque|$\tau_{cont}/K_1$|6.3°|where the _linear demand_ first exceeds the ceiling. Not a recovery limit at all — it is a property of the gain set|
|Momentum|$1-\cos\theta = h_{cap}^2/(2\Theta Sg)$|14.3°|the full momentum budget spent on one perfectly efficient recovery|
|Static|$\arcsin(\tau/Sg)$|27.2°|hold position with no momentum exchange|

---

## 2 — Plant

Built from **measured** part masses (03/08/2026), not assumed density. See [[Panel Mass Reconstruction]] / the header of `cubli_panel_params.m`.

|||
|---|---|
|m|0.2631 kg|
|ℓ|101.70 mm|
|S = mℓ|0.02675 kg·m|
|Sg|0.2623 N·m|
|Θ|3.3986e-3 kg·m²|
|I_w|1.8583e-4 kg·m²|
|λ|9.037 rad/s (τ = 111 ms)|
|K|[−1.0998, −0.1232, −0.001732]|
|τ_cont / τ_peak|0.12 / 0.40 N·m|
|ω_cap|40 rad/s → h_cap = 0.00743 N·m·s|

---

## 3 — How the code is structured

### 3.1 The model change — one block

A MATLAB Function block `ACTUATOR` between the LQR gain and the Simulink-PS Converter:

```matlab
function u = actuator(u_cmd, phidot, tau_max, w_cap)
%#codegen
u = max(min(u_cmd, tau_max), -tau_max);      % torque ceiling

s = (w_cap - abs(phidot)) / (0.1*w_cap);     % taper over the last 10 %
s = max(min(s, 1), 0);
if sign(u) == sign(phidot)                   % only when spinning UP
    u = u * s;
end
end
```

Wiring: `u_cmd` ← LQR Gain, `phidot` ← Mux element 3 via a Selector, `tau_max` ← Constant, `w_cap` ← Constant `p.omega_cap`. A **Manual Switch** bypasses the block so the linear path stays available for re-running Gate 3 — that discipline is what keeps a saturation bug distinguishable from a plant bug.

> [!warning] The taper is not cosmetic The first version hard-zeroed torque at the cap. That produced **chatter**: the actuator switched on and off at every threshold crossing, the variable-step solver drove its step toward zero, and one 8 s run took over a minute. The taper removes the discontinuity.
> 
> It is _not_ however a back-EMF model. At 40 rad/s the back-EMF is only **4.5 % of the 6S supply**, so the motor still has essentially full torque capability. The cap and its taper are **firmware policy**.

Braking torque is deliberately never restricted — only torque that would spin the wheel _faster_ is faded. Torque that slows it is always allowed, since that is exactly the recovery action needed at the cap.

### 3.2 Test structure

```
runtilt()          one simulation at a given theta_0
   |
   +-- 1. coarse sweep   8..18 deg     -> bracket the transition
   +-- 2. bisection      10 halvings   -> resolve the edge
   +-- 3. binding limit  at 98 % edge  -> WHICH constraint is active
   +-- 4. plots
```

`runtilt` returns, per run:

|Field|Meaning|
|---|---|
|`r.umax`|**demanded** torque $\max\|Kx\|$ — how hard the controller strained|
|`r.u`|**applied** torque, after ceiling and taper|
|`r.wmax`|peak wheel rate|
|`r.tsat`|seconds spent above the torque ceiling|
|`r.ok`|recovered: $\|\theta_{end}\| < 2°$ **and** all states finite|

Demanded vs applied is what makes the output readable — applied torque can never exceed the limit by construction, so only the demand carries information. The finite check matters because a diverging run produces NaNs, which would otherwise read as a spurious pass.

Bisection rather than a fine sweep, because each simulation is expensive and only one number matters.

The **binding-limit diagnostic** re-runs at 98 % of the edge and measures what fraction of the trajectory sits on each constraint. This is the part that says _what to change_, which is more useful than the envelope alone.

### 3.3 Gotchas hit during the build

- `res(i) = r` on a preallocated struct → _"Subscripted assignment between dissimilar structures"_. Fixed by collecting into a cell (`res{i} = r`) and flattening with `res = [res{:}]` after the loop.
- Solver: `'MaxStep','1e-3'` and `'ZeroCrossControl','DisableAll'` on the `sim` call, needed even with the taper.
- The binding diagnostic must compare against **the ceiling actually in use**. Left at `p.tau_cont` during the τ_peak run it reported "torque binds first" when torque never saturated at all.
- Naive-bound printout needs `abs(K_lqr(1))` — the gains are negative.

---

## 4 — Results

### Run 1 — conservative, ceiling at τ_cont = 0.12 N·m

|θ₀|ok|demand|% τ_cont|max φ̇|t_sat|
|---|---|---|---|---|---|
|8°|true|0.1536|128|33.2|0.067 s|
|10°|true|0.1919|160|39.7|0.108 s|
|12°|false|—|—|—|—|

**Edge = 11.05°.** At 98 % of edge: 1.6 % of the run on the torque limit, 1.5 % at the wheel cap → **both constraints bind**.

### Run 2 — realistic, ceiling at τ_peak = 0.40 N·m

τ_cont is a _thermal average_ over a time constant of tens of seconds. The saturation events last ~0.1 s and deposit negligible heat, so clipping at τ_cont at every instant is conservative by construction.

|θ₀|ok|demand|max φ̇|t_sat|
|---|---|---|---|---|
|8°|true|0.1536|32.7|0.184 s|
|10°|true|0.1919|39.2|0.266 s|
|12°|false|—|—|—|

**Edge = 11.96°.** At 98 % of edge: **0.0 % on the torque limit**, 1.8 % at the wheel cap → **momentum binds, alone**.

### Summary

|Actuator model|Envelope|Torque limit active|Cap active|
|---|---|---|---|
|Conservative (τ_cont)|11.05°|1.6 %|1.5 %|
|Realistic (τ_peak)|**11.96°**|**0.0 %**|1.8 %|

---

## 5 — What it means

### 5.1 The envelope is ~75 % larger than the naive bound

11° against 6.3°, because **the controller does not need to hold statically — it needs to survive transiently.** At 10° it demands 160 % of τ_cont for 0.108 s, banks momentum in the wheel, and rides that back to upright.

The torque ceiling is a **rate limit on momentum transfer**, not a wall. Exceed it briefly and momentum simply transfers more slowly than the ideal law wanted. Same structure as the sizing note that dynamic recovery beats the static bound, one level down at the actuator.

### 5.2 The motor is oversized for this panel

Relaxing the torque ceiling by **3.3×** bought **8 %** more envelope. At the recovery limit the controller asks for 0.192 N·m — **48 % of τ_peak**. Torque headroom is not what constrains performance here.

### 5.3 The wheel cap is the real design parameter

11.96° is **84 %** of the theoretical momentum bound (14.3°). The gap is real: LQR spends momentum less efficiently than the energy-optimal manoeuvre the bound assumes. That 84 % is a useful realisation factor.

Envelope scales with the cap at roughly **0.3° per rad/s**:

|ω_cap|Momentum bound|Expected realised (84 %)|
|---|---|---|
|30|10.7°|~9°|
|**40**|14.3°|**~12°** ✓|
|50|17.9°|~15°|
|60|21.5°|~18°|

> [!important] Why the cap stays at 40 despite costing envelope The motor tops out near **883 rad/s**. The cap discards 95 % of the available momentum budget _on purpose_. Without it the panel cannot exercise momentum saturation at all — and that is the failure mode that kills Cubli builds: a small COM offset slowly driving the wheel to its limit, presenting as "balances for eight seconds, then falls over for no visible reason". Better to meet it deliberately in a controlled test than accidentally in the demo. See [[Disturbance Estimation & Attitude Representation]].

### 5.4 It is robust to a 25 % mass revision

The first run, before the parts were weighed, gave **11.02°** on a plant 25 % heavier. The two bounds moved in opposite directions and cancelled:

||ρ = 845 estimate|Measured|
|---|---|---|
|Static torque bound|20.4°|27.2° (lighter panel, less gravity torque)|
|Momentum bound|15.9°|14.3° (lighter wheel, less I_w)|
|**Edge**|**11.02°**|**11.05°**|

So 11° is not an artefact of the density assumptions.

### 5.5 Failure mode

Beyond the edge, θ runs to ~330° and oscillates about 180° — the panel lying down and swinging as a pendulum. Bounded, no numerical blow-up, which is itself a good sign the model behaves outside its design envelope.

### 5.6 Translating to the bench

|Simulation says|On the real panel|
|---|---|
|Recovers from ≤ 11–12°|A firm nudge is survivable; a shove is not|
|Fails at ≥ 12°|This is not a tuning problem — beyond ~12° it _will_ go over|
|Wheel hits 39.7 of 40 rad/s at the edge|The firmware cap actually engages. It is not a theoretical guard|
|Torque demand 48 % of peak|The motor is not the bottleneck|
|Clean fall to hanging|Safe to test — the base-plate rails catch it|

---

## 6 — Caveats

**Perfect state knowledge.** No quantisation, no sensor noise, no loop delay. All three consume margin. The realistic figure comes after Blocks B and C, and will be **lower** than 11–12°.

**τ_cont = 0.12 N·m is an estimate**, not a datasheet figure — a thermal guess for a prop motor with no airflow. It matters less than it did now that Run 2 shows torque is not binding, but it still sets the conservative bracket. A thermal test would settle it.

**Released from rest.** A real disturbance is an impulse: the panel arrives at 11° already moving, which is strictly harder.

**No friction or cogging.** Both oppose wheel spin-up and eat into the budget.

---

## 7 — Open items

- [ ] Re-run after Blocks B and C for the realistic envelope
- [ ] Sweep ρ against recoverable angle — one line, informs the hardware gains
- [ ] Thermal test to pin τ_cont
- [ ] Plumb-line the COM for ℓ, and clamped-wheel swing period for Θ (fastener placement is currently assumed — see the params header)

---

## Related

- [[Simscape Panel Model — Build Guide]] — the six gates, §6 for the remaining nonlinear blocks
- [[Firmware Handoff — Panel Stage]] — the caps that must match sim and Teensy
- [[Panel Controller Workflow]] — where the gains come from
- [[Disturbance Estimation & Attitude Representation]] — why momentum saturation is the failure mode that matters