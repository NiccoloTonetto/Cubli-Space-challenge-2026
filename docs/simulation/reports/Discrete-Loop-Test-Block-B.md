---
tags:
  - space-challenge
  - sofia
  - cubli
  - simscape
  - simulation
  - panel
  - controls
  - firmware
  - validation
---

# Discrete Loop Test — Block B

Second nonlinear test from §6 of [[Simscape Panel Model — Build Guide]].
Answers: **how slowly can the Teensy run before the loop falls over?**

**Result: 35 Hz floor. The controller is not what sets the loop rate.**

Script: `cubli_panel_discrete.m`
Depends on: `cubli_panel_params.m` (measured mode) and the LQR cell of
`cubli_panel_simscape_gates.m`.

---

## 1 — Structure of the test

[[Saturation Envelope Test — Block A]] asked *how far can it tip* — a question
about actuator authority. Block B asks a genuinely independent question: *how
much phase can we throw away before the unstable pole outruns the loop?*

The plant has an unstable pole at λ = 9.037 rad/s, so disturbances grow with a
**111 ms** time constant. Every millisecond between reading a sensor and
applying a torque is time the plant spends diverging uncorrected. Two effects
produce that delay:

| Source | Delay | Modelled by |
|---|---|---|
| Sample-and-hold | ~0.5·Ts (average staleness) | Zero-Order Hold |
| Compute latency | 1.0·Ts (read → apply, one cycle later) | Unit Delay |
| **Total** | **~1.5·Ts** | |

The Unit Delay is the one that matters and the one usually omitted. It is
invisible in continuous design and is the standard reason a controller that
works in simulation fails on hardware.

**Method:** sweep the loop rate from 800 Hz down to 25 Hz, score each run as
recovered or not, then bisect the boundary. Then compare that floor against the
*other* candidate constraints on loop rate, because the binding one may not be
the controller at all.

---

## 2 — Code review

### 2.1 Model change

```
Mux ──► Zero-Order Hold (Ts) ──► Unit Delay (Ts) ──► Gain(-K_lqr) ──► ACTUATOR
```

Both blocks take **Sample time = `Ts`**, the workspace variable — not a
literal, since the script rewrites it every iteration. A Manual Switch bypasses
both so the continuous path stays available for re-running Gate 3.

### 2.2 Script flow

```
runrate()          one simulation at a given loop rate
   |
   +-- 1. rate sweep       25 .. 800 Hz     -> bracket the floor
   +-- 2. bisection        8 halvings       -> resolve it
   +-- 3. constraint table -> which limit ACTUALLY sets f_outer
   +-- 4. c2d cross-check  -> independent discrete pole check
   +-- 5. plots
```

**`runrate`** pushes the new `Ts` into the base workspace, forces a model
recompile, simulates, and scores `|θ_end| < 2°` with all states finite.

Two details worth keeping:

- The `sim` call is wrapped in **try/catch**. At very low rates the solver
  aborts rather than diverging cleanly, and a bare script would simply stop.
  A caught failure is scored as unstable, which is the physically correct
  reading.
- `assignin('base','Ts',Ts)` followed by `set_param(...,'update')` is what
  actually propagates the sample time into the blocks. Without the update the
  sweep silently runs the same rate nine times.

**Bisection** rather than a fine sweep, same reasoning as Block A: each
simulation is expensive and only one number matters.

### 2.3 The two cross-checks

**§3 constraint table** puts the measured stability floor next to the Nyquist
requirements for the wheel vibration, and prints which dominates. This is the
part that answers the open question in [[Firmware Handoff — Panel Stage]].

**§4 `c2d` check** discretises the plant analytically, augments the state with
one sample of input delay, and computes the closed-loop pole magnitudes:

```matlab
sysd = c2d(ss(p.A,p.B,eye(3),0), Ts, 'zoh');
Aa = [Ad Bd; zeros(1,3) 0];   Ba = [zeros(3,1); 1];   Ka = [K_lqr 0];
ev = eig(Aa - Ba*Ka);         % all |z| < 1 for stability
```

Independent of Simscape entirely. If the simulation diverges but `max|z| < 1`,
the fault is in the model wiring, not the physics — a useful discriminator.

---

## 3 — Results

Plant: λ = 9.037 rad/s, K = [−1.0998, −0.1232, −0.001732], θ₀ = 2.9°.

| f [Hz] | Ts [ms] | θ_end [deg] | max\|u\| [N·m] | max φ̇ | stable |
|---|---|---|---|---|---|
| 25 | 40.00 | 338.7 | 7.129 | 195.5 | **false** |
| 40 | 25.00 | 0.000 | 0.069 | 23.1 | true |
| 60 | 16.67 | 0.000 | 0.064 | 17.0 | true |
| 80 | 12.50 | 0.000 | 0.062 | 14.4 | true |
| 100 | 10.00 | 0.000 | 0.060 | 13.4 | true |
| 150 | 6.67 | 0.000 | 0.058 | 12.5 | true |
| 200 | 5.00 | 0.000 | 0.058 | 12.3 | true |
| 400 | 2.50 | 0.000 | 0.056 | 12.0 | true |
| 800 | 1.25 | 0.000 | 0.056 | 11.8 | true |

**Minimum stable loop rate = 35 Hz** (Ts = 28.4 ms).
Loop delay at the floor ≈ 42.6 ms = **38.5 % of 1/λ**.

### Constraint comparison

| Constraint | Requirement |
|---|---|
| Control stability floor | **35 Hz** |
| Nyquist, wheel fundamental at ω_cap (6.4 Hz) | 13 Hz |
| Nyquist, wheel fundamental at ω_max (140.6 Hz) | **281 Hz** |
| Same, with ~5× hardware-filter relief | 56 Hz |

→ **Anti-aliasing dominates by 8×.**

### `c2d` cross-check at 400 Hz

`|z| = 0.0757, 0.9631, 0.9766, 0.9851` — max 0.9851, inside the unit circle.
Agrees with the simulation. The 0.0757 pole is the augmented delay state; the
0.9851 is the slow wheel-unwinding mode.

---

## 4 — Discussion

### 4.1 The controller does not set the loop rate

35 Hz floor against a Teensy 4.1 that will comfortably run 1–3 kHz. That is
**28× margin at 1 kHz, 86× at 3 kHz.** The loop rate is therefore a *free
parameter* from the control side, to be spent on whatever else needs it.

This is the useful result. It was not obvious in advance — with an unstable
pole the instinct is that the loop must be fast, and the derivation says it can
afford to be startlingly slow.

**Why so tolerant:** the plant is slow. 1/λ = 111 ms, and the loop survives
until the delay reaches 38.5 % of that. Compare a quadrotor attitude loop, where
the equivalent time constant is a few milliseconds.

### 4.2 The real floor is anti-aliasing

The wheel fundamental is max_rpm/60 ≈ **141 Hz** at full speed, so Nyquist
demands **281 Hz** — 8× above the stability floor. Vibration above Nyquist folds
down into the control band as a spurious low-frequency signal the controller
will faithfully try to correct.

Note the caveat in the table: the 141 Hz figure is at ω_max, not at the 40 rad/s
firmware cap, where the fundamental is only 6.4 Hz. **Design for ω_max**, since
the cap is policy and may be relaxed, and because spin-down through high speeds
happens during any aggressive manoeuvre.

Hardware filtering in the BMI270 relaxes the requirement roughly 5× (to ~56 Hz)
by attenuating before sampling. Software filtering does not — by then the
aliasing has already happened. **This is the architecture decision, and it is
worth more than any control-side tuning.**

### 4.3 What binds at the top end — not the controller either

| f | Loop delay | % of 1/λ | φ̇ noise from encoder | Torque noise |
|---|---|---|---|---|
| 100 Hz | 15.0 ms | 13.6 % | 0.010 rad/s | 0.01 % τ |
| 400 Hz | 3.75 ms | 3.4 % | 0.038 rad/s | 0.06 % τ |
| 1 kHz | 1.50 ms | 1.4 % | 0.096 rad/s | 0.14 % τ |
| 3 kHz | 0.50 ms | 0.5 % | 0.288 rad/s | 0.42 % τ |

Encoder quantisation noise on φ̇ grows as 1/Ts, but K₃ = 0.001732 is so small
that even at 3 kHz it contributes under half a percent of τ_cont. Not a limit.

What *would* bind:

- **IMU output data rate.** A 3 kHz loop against a 400 Hz BMI270 ODR resamples
  stale data seven times in eight — no information gained, and it makes the
  loop sensitive to jitter that does not exist in the physics.
  **Match the loop rate to the IMU ODR.**
- **CAN round-trip.** Query + reply to three moteus drivers is roughly
  150–300 µs. At 3 kHz the period is 333 µs, leaving no slack for a late frame.
  At 1 kHz there is comfortable margin.

### 4.4 Where the returns stop

Looking at the recovery traces: **40 Hz rings visibly**, 60–80 Hz shows mild
overshoot, and from **100 Hz upward the traces are indistinguishable**.

So 100 Hz is where the controller stops caring. Everything above that is bought
for anti-aliasing and margin, not for control performance.

Peak torque demand also falls with rate (0.069 → 0.056 N·m) and peak wheel speed
nearly halves (23.1 → 11.8 rad/s). A slow loop wastes actuator authority
correcting its own lag — worth remembering when the Block A envelope is
re-measured with the discrete loop in place.

### 4.5 Recommendation for firmware

**Run 400–1000 Hz.**

- Above the 281 Hz anti-aliasing floor
- 10–28× clear of the 35 Hz stability floor
- Well inside CAN round-trip budget
- Does not outrun a realistic IMU ODR

The exact figure should follow the BMI270 ODR rather than being chosen
independently. And the headline for [[Firmware Handoff — Panel Stage]]:
**28× stability margin at 1 kHz** is real design freedom — spend it on jitter
tolerance, a missed-frame policy, or a slower but better-filtered estimator,
without the controller noticing.

---

## 5 — Caveats

- **Perfect state knowledge still.** No sensor noise, no quantisation in the
  loop, no estimator. Block C adds these, and they will raise the practical
  floor above 35 Hz.
- **Zero jitter.** Every sample lands exactly on Ts. Real firmware has jitter
  and occasional missed frames; the 35 Hz figure has no allowance for either.
- **θ₀ = 2.9° only.** The floor was not swept against initial angle. A larger
  disturbance near the Block A envelope may need a faster loop.
- **The Ms = 1.0000 guarantee is gone.** Full-state continuous LQR has an exact
  return-difference identity; the hold and delay destroy it, which is why the
  discrete poles must be checked directly rather than assumed.

---

## 6 — Open items

- [ ] Re-run the Block A envelope with the discrete loop at the chosen rate —
      the realistic recoverable angle will be below 11–12°
- [ ] Sweep the floor against θ₀
- [ ] Add timing jitter once Andrea measures it
- [ ] Confirm the BMI270 ODR and filter configuration, then fix f_outer

---

## Related

- [[Saturation Envelope Test — Block A]] — actuator authority, the other axis
- [[Simscape Panel Model — Build Guide]] — §6.4 for the delay rationale
- [[Firmware Handoff — Panel Stage]] — this closes the open loop-rate question
- [[Panel Controller Workflow]]
