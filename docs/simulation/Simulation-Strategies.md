---
tags: [space-challenge, sofia, cubli, simulation, planning]
---

# Cubli Simulation — Strategies & Sprint Plan

**Scope (final):** simulation only, no hardware. Goal is to arrive in Sofia on July 20 with (a) strong grasp of the state-of-the-art in Cubli control, (b) a working Simscape Multibody model, (c) at least two controllers implemented and compared, so you can immediately contribute to the team project.

**Assumed licence:** Simscape Multibody available via the university MATLAB licence — confirmed.

**Time budget:** July 18 evening → July 20 (intro day). Effectively ~2 days of concentrated work. Everything below is scoped to fit that window with slack, prioritizing depth over breadth.

## Approach — chosen path

**Simscape Multibody + Simulink (Option C from the earlier analysis).** Skips Option A (pure MATLAB) as a target because the goal is not a teaching exercise — the goal is the most accurate simulation you can bring to a team, and that's Simscape.

However, the Simscape workflow *does* start with hand-derived equations of motion for the 1D case. This isn't overhead — it's the sanity check that catches Simscape model errors (misaligned joint frames, wrong inertia, dropped mass) that would otherwise silently poison every controller you tune against it. So the plan below starts analytical then goes Simscape, not because we're doing both as products, but because the analytical model is the validation reference for the Simscape model.

## Companion notes
- [[References]] — curated Cubli research papers
- [[State of the Art Controllers]] — survey of controllers applied to Cubli-family systems (PID → LQR → backstepping → MPC → learning-based → hybrid/jump-up). Read this in parallel with the sprint plan below.

## Theoretical prep — what you actually need before touching code

Ordered by importance. Everything in bold is required before the Simscape model is meaningful.

1. **Rigid-body dynamics with multiple bodies** — cube + 3 wheel rotors; inertia tensors in body vs. inertial frame; angular momentum decomposition.
2. **Lagrangian mechanics for constrained systems** — pivot (edge/corner) as holonomic constraint.
3. **Attitude representation** — Euler angles fine for 1D; **quaternions** strongly preferred for 3D. Ribas Martins & da Silva (arXiv:2009.14625) is the cleanest derivation.
4. **Linearization about unstable equilibrium** — Jacobians of f(x,u), A/B matrices, controllability check via `ctrb`. Understand *why* the pole is unstable (positive real part from gravity term).
5. **LQR** — algebraic Riccati equation via `lqr`. Understand Q, R selection: penalize attitude error and wheel speed together, or LQR happily lets wheels drift toward saturation.
6. **Kalman filter** — for IMU + encoder fusion. Same as CELAB Luenberger observer work, extended to stochastic setting.
7. **Actuator modeling** — BLDC as first-order torque source with saturation. Motor time constant, torque-speed curve if you want to be thorough.
8. *For state-of-the-art depth (parallel with sprint):* backstepping (Muehlebach & D'Andrea) and constrained MPC (any recent tutorial on MATLAB's MPC toolbox). See [[State of the Art Controllers]] for the map.

## 2-day sprint plan

Deliverables prioritized so that if you get pulled away partway through, what you have is still usable. Each phase produces a saveable artifact.

### Day 1 (July 18 evening + July 19 full day)

**Morning block 1: theory intake (~3 hours)**
- Watch ETH Cubli videos (~10 min).
- Read IROS 2012 in full (mechanism, 1D dynamics, jump-up strategy).
- Read ECC 2013 sections II (dynamics) and IV (control) in detail; skim III (parameter ID).
- Skim Muehlebach & D'Andrea nonlinear paper — enough to know what backstepping does structurally, don't need to internalize the Lyapunov proofs yet.

**Morning block 2: analytical 1D derivation (~2 hours)**
- Hand-derive 1D edge-balancing equations of motion via Lagrangian.
- State vector: `[θ, θ̇, ω_w]` (pendulum angle, its rate, wheel angular velocity).
- Linearize about upright.
- Verify controllability.
- **Deliverable:** clean derivation in Obsidian at `Cubli Simulation/Derivations/1D Edge Dynamics.md`.

**Afternoon block 1: MATLAB script sanity check (~2 hours)**
- Implement the nonlinear ODE and linearized system in a MATLAB script.
- Design LQR on linearized model.
- Simulate closed-loop response from θ₀ = 5°, verify settling behavior matches ECC 2013 order-of-magnitude (~0.5–1 s).
- Save the LQR gain `K_lqr_1d` — you'll reuse it to sanity-check the Simscape model.
- **Deliverable:** working `cubli_1d_lqr.m`, plot showing controlled settling.

**Afternoon block 2: Simscape model, 1D (~3 hours)**
- Build a Simscape Multibody model of the 1D Cubli (single body + one revolute joint at pivot + one reaction wheel on a second revolute joint).
- Use ECC 2013 nominal parameters as a starting point (or reasonable estimates if some are missing — a 15 cm × 15 cm × 15 cm aluminum cube with a 5 cm reaction wheel is a fine placeholder).
- Confirm the model swings freely under gravity, matching pendulum period from the analytical model.
- Extract linearized state-space via `linmod` or Simulink's Model Linearizer.
- Compare `A`, `B` matrices to your analytical derivation — this is the critical validation step. Small differences are OK (numerical linearization); order-of-magnitude differences mean the Simscape model is wrong.
- **Deliverable:** `cubli_1d.slx` Simscape model that passes the free-swing and linearization sanity checks.

**Evening block: LQR on Simscape 1D (~1.5 hours)**
- Wrap the Simscape plant with an LQR controller.
- Reproduce the analytical settling behavior. If they diverge, diagnose (usually joint frame alignment, inertia bookkeeping, or forgetting to include the wheel's rotor inertia).
- **Deliverable:** closed-loop 1D Simscape sim, animation working.

### Day 2 (July 20 morning, before evening intro)

**Morning block 1: 3D dynamics (~2 hours)**
- Derive 3D equations of motion for corner balancing. Use quaternions (Ribas Martins & da Silva as reference).
- Linearize about the corner-upright equilibrium.
- Design LQR on the linearized 3D plant.
- **Deliverable:** derivation in `Derivations/3D Corner Dynamics.md`, LQR gain `K_lqr_3d`.

**Morning block 2: Simscape 3D model (~2 hours)**
- Extend the 1D Simscape model to a full 3D cube with three orthogonal reaction wheels.
- Add a 6-DOF joint at the corner-pivot (or 3-DOF spherical joint if you want to constrain translation).
- Sanity-check free-swing dynamics against a simple 3D pendulum model.
- **Deliverable:** `cubli_3d.slx` model.

**Midday block: LQR on 3D Simscape + noise (~2 hours)**
- Deploy the 3D LQR controller in Simscape.
- Add IMU noise blocks (gyro + accelerometer).
- Add a Kalman filter for state estimation.
- Verify closed-loop behavior degrades gracefully with noise (rather than going unstable — if it does, the Kalman filter is mistuned).
- **Deliverable:** `cubli_3d_lqg.slx`, plot of corner balancing under noise.

**Afternoon block (if time allows before intro): controller comparison (~1–2 hours)**
- If Day 1 and Day 2 morning went cleanly, add either:
  - A PID controller as a baseline (fast, ~30 min) → gives you a "here's what the naive approach does" comparison figure for team discussions.
  - Or a first pass at a backstepping controller — much more involved, only start this if you're ahead of schedule.
- **Deliverable:** side-by-side plot of controllers on identical Simscape plant. This is the artifact most valuable to bring to Sofia.

## What to bring to Sofia (deliverable checklist)

- [ ] The two Obsidian derivation notes (1D + 3D).
- [ ] The three Simscape model files (`cubli_1d.slx`, `cubli_3d.slx`, `cubli_3d_lqg.slx`).
- [ ] A short "Cubli Simulation — What I Built" summary note for teammates: architecture, parameters, controllers implemented, known open items.
- [ ] The [[References]] and [[State of the Art Controllers]] notes as offline reading material.
- [ ] The ECC 2013 and TCST 2016 PDFs downloaded locally in case hotel wifi is bad.

## Realistic grade of results

Assumes the sprint plan goes reasonably well (some Simscape debugging pain is normal).

- **1D LQR balancing in Simscape:** ~90% fidelity to ECC 2013 for the 1D case. Very achievable in Day 1.
- **3D LQR corner balancing in Simscape:** ~75–85% fidelity. The gap comes from parameter estimates (real Cubli parameters were identified from hardware; ours will be nominal). But the *structure* of the response — settling time, wheel speed evolution, controller effort — will match qualitatively.
- **3D LQG with IMU noise + Kalman:** matches published academic Cubli simulations. Getting the Kalman tuning right on Day 2 morning is the main risk.
- **Backstepping controller:** stretch goal. Unlikely to fit in 2 days if you also want it verified against LQR. Reasonable to arrive in Sofia with the theoretical understanding and the LQR/LQG working, and implement backstepping *during* the challenge with team support.
- **MPC comparison:** stretch stretch goal. Realistically a Day 3–4 task, so on the team's schedule not yours.
- **Jump-up maneuver:** out of scope for the sprint. Great topic for the team project once corner balancing is solid.

## What this gets you in Sofia

Walking in on July 20 with a working 3D Simscape LQR/LQG cube and a mental map of the SOTA controllers means:

- If assigned to Cubli: you can lead the simulation subteam from Day 1 rather than spending the first week catching up to the ETH baseline.
- If assigned to CubeSat: same underlying skills (attitude control, reaction wheels, LQR/LQG, Kalman) transfer directly, and you have a portfolio piece.
- If assigned to AI (unlikely per your preference ordering, but possible): still valuable as personal work, and the [[State of the Art Controllers]] survey is exam-useful for space-robotics topics.

## Open items to resolve as you go
- Choice of nominal parameters (ECC 2013 vs. estimates for a 15 cm cube). If ECC 2013 doesn't publish the full parameter table, use their identified frequency-domain estimates.
- Simscape solver settings (typically variable-step, ode15s or ode23t for stiff mechanical systems).
- Whether to model the reaction wheel motor with a BLDC block (higher fidelity, more setup) or a torque source with a first-order lag (fast, adequate for controller design).
