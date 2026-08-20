// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 4: FULL LAW + MODE-AUGMENTED KALMAN FILTER + FIXED OFFSET
// ============================================================================
// EXPERIMENTAL / UNVALIDATED FILTER. Read this whole header before flashing.
//
// Copy of Stage4_AutoTrim_RateFilter.ino with THREE changes: (1) fixed
// offset + no arm gate, same as Stage4_FixedOffset_RateFilter.ino -- see
// that file's header for the reasoning, not repeated in full here; (2)
// the first-order rate low-pass (w_filt, SECTION 2b-2) is REPLACED by a
// 3-state Kalman filter per axis that explicitly models the ~30-40 Hz
// structural mode as its own state, rather than just attenuating
// everything above a corner frequency; (3) three new telemetry columns
// (mode_x/y/z_dps) expose the filter's internal estimate of the mode's
// own contribution to the raw rate, for bench validation.
//
// ---------------------------- WHY THIS EXISTS -------------------------------
// docs/testing/Kalman-Filter-Rate-Estimator-Evaluation-2026-08-20.md
// (Section 3, "Option 2") is the design doc this file implements. Short
// version: hw-run-analysis.md's fix 4.1 (the first-order filter in
// Stage4_AutoTrim_RateFilter.ino) only gets ~6 dB at the real mode
// frequency -- a first-order filter structurally cannot do much better
// without also destroying the 1.3 Hz phase margin it's built to protect.
// A Kalman filter that carries an explicit STATE for the mode (a 2nd-
// order oscillator, not just a low-pass) can in principle separate the
// mode's own contribution from the genuine rigid-body rate even where
// their frequency content overlaps, because it discriminates by
// consistency with a dynamics model rather than by frequency alone --
// the same reason ghat/phi is already clean while raw w_b isn't (see
// that doc's Section 1).
//
// THIS WAS VALIDATED ON REAL TELEMETRY BEFORE BEING PORTED HERE (that
// doc's Section 2), not just derived on paper -- and the validation
// process itself found a naive/kinematic version of this idea LOSES to
// the plain first-order filter (less attenuation of the mode AND less of
// the real signal preserved, at matched overall smoothing). It only wins
// once the mode itself gets an explicit state, which is what this file
// implements. Don't mistake "it's a Kalman filter" for automatic
// superiority -- it earns it here, but the naive version genuinely didn't.
//
// ---------------------------- REAL, UNRESOLVED RISK -------------------------
// The mode's natural frequency has NOT been pinned down by a proper tap
// test (still outstanding per both that doc and the Notch Filter doc it
// follows up on) -- observed peaks across three different sessions were
// 29.7-30.5 Hz, 38.7 Hz, and 41.0 Hz. That spread is itself evidence the
// mode may not be stable session-to-session (mounting, temperature,
// battery position). gModeHz/gModeZeta below are LIVE-SETTABLE ("n<Hz>"/"z<value>")
// specifically because of this uncertainty -- do not treat the compiled-
// in defaults as validated, they're a starting point for bench sweeping,
// not a result. A wrong mode model is NOT neutral the way a wrong notch
// frequency is (a notch just stops helping off-frequency) -- this filter
// actively trusts its own model, so a bad wn/zeta can inject a worse
// estimate than doing nothing. Watch mode_x/y/z_dps in telemetry: it
// should look like a genuine ~30-40 Hz oscillation, not noise or a flat
// line -- either of those is a sign the model doesn't match what's
// actually happening and gModeHz/gModeZeta need adjusting or this file needs to
// wait for the tap test.
//
// SAFETY CHECKS ARE UNAFFECTED regardless of how wrong gModeHz/gModeZeta are:
// raw w_b still drives ghat propagation and every isfinite()/kMaxTilt
// trip check, exactly like Stage4_AutoTrim_RateFilter.ino's w_filt --
// this filter's output (om_true) only replaces w_b in commandWheels()'s
// xVec om block, nowhere else. A bad Kalman estimate can make the
// CONTROL response worse; it cannot suppress a real trip.
//
// ---------------------------- FIXED OFFSET AND ARM GATE ---------------------
// Same as Stage4_FixedOffset_RateFilter.ino: kPhiOffset (perfect_
// equilibrium_2.log, mean over 132.41s, std<0.0012 deg/axis) replaces
// live trim adaptation, and the arm gate is removed -- "a1" arms from any
// tilt. The Kp control law is only validated for small angles (recovery
// envelope ~2.7-3.1 deg worst case, Cube-Performance-Envelope-Results.md);
// the ONLY remaining backstop once armed is kMaxTilt (25 deg auto-trip,
// unchanged). Given this file ALSO carries the unresolved Kalman-filter
// risk above, treat this as the highest-caution file in the whole
// corner-bringup family -- stop/catch and a hand on the e-stop path,
// more than any other Stage 4 variant here.
//
// UPDATE 2026-08-19: corner [-1,-1,-1]'s Kp/ell/theta_eq below are the
// re-derived values for the new plant (mass 1.633 kg + strut). The other
// seven corners are still the OLD (1.5668 kg, no strut) table -- see the
// TODO above kCorners.
//
// STAGE 4 CHECKLIST — cube held by hand, gGainScale = 1.0:
//   Still place near the resolved corner's equilibrium before "a1".
//   [ ] mode_x/y/z_dps in telemetry looks like a real ~30-40 Hz
//       oscillation (see REAL, UNRESOLVED RISK above) -- if it looks like
//       noise or stays flat, gModeHz/gModeZeta don't match this session and need
//       adjusting via "n<Hz>"/"z<value>" before trusting anything else.
//   [ ] om_x/y/z_filt_dps (the Kalman om_true estimate) should track
//       smoother than raw om_x/y/z_dps -- compare against a
//       Stage4_FixedOffset_RateFilter.ino run on the same corner if the
//       improvement isn't obvious.
//
// Velocity cap loosened (kMaxOmega, not literally removed) same as every
// other Stage 4 file in this family. Trips here are still LOOSE
// (hand-held) -- Stage 5 tightens to the real DISARM/OMEGA_CAP policy.
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

Moteus moteusX(canBus, []() { Moteus::Options options; options.id = 2; return options; }());
Moteus moteusY(canBus, []() { Moteus::Options options; options.id = 3; return options; }());
Moteus moteusZ(canBus, []() { Moteus::Options options; options.id = 1; return options; }());

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from Stage 1-3)
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };

float theta1_deg = -30.0f;
float theta2_deg = 54.74f;
float theta3_deg = 45.0f;
float gMountDCM[3][3];

void updateMountingDCM() {
  const float k  = (float)DEG_TO_RAD;
  const float c1 = cosf(theta1_deg * k), s1 = sinf(theta1_deg * k);
  const float c2 = cosf(theta2_deg * k), s2 = sinf(theta2_deg * k);
  const float c3 = cosf(theta3_deg * k), s3 = sinf(theta3_deg * k);
  gMountDCM[0][0] =  c1*c3 - c2*s1*s3;
  gMountDCM[0][1] =  c3*s1 + c1*c2*s3;
  gMountDCM[0][2] =  s2*s3;
  gMountDCM[1][0] = -c1*s3 - c2*c3*s1;
  gMountDCM[1][1] =  c1*c2*c3 - s1*s3;
  gMountDCM[1][2] =  c3*s2;
  gMountDCM[2][0] =  s1*s2;
  gMountDCM[2][1] = -c1*s2;
  gMountDCM[2][2] =  c2;
}

void checkMountingDCMValid() {
  const float (&C)[3][3] = gMountDCM;
  const float det = C[0][0]*(C[1][1]*C[2][2] - C[1][2]*C[2][1])
                   - C[0][1]*(C[1][0]*C[2][2] - C[1][2]*C[2][0])
                   + C[0][2]*(C[1][0]*C[2][1] - C[1][1]*C[2][0]);
  Serial.print("# mount DCM check: det="); Serial.print(det, 4);
  for (int i = 0; i < 3; ++i) {
    const float len = sqrtf(C[i][0]*C[i][0] + C[i][1]*C[i][1] + C[i][2]*C[i][2]);
    Serial.print("  |row"); Serial.print(i); Serial.print("|="); Serial.print(len, 4);
  }
  Serial.println();
}

void readIMURaw(BMI270& sensor, float aImu[3], float wImu[3]) {
  aImu[0] = (sensor.data.accelX * kG0 - kAccelOffset[0]) / kAccelScale[0];
  aImu[1] = (sensor.data.accelY * kG0 - kAccelOffset[1]) / kAccelScale[1];
  aImu[2] = (sensor.data.accelZ * kG0 - kAccelOffset[2]) / kAccelScale[2];
  wImu[0] = sensor.data.gyroX * (float)DEG_TO_RAD - kGyroBias[0];
  wImu[1] = sensor.data.gyroY * (float)DEG_TO_RAD - kGyroBias[1];
  wImu[2] = sensor.data.gyroZ * (float)DEG_TO_RAD - kGyroBias[2];
}

void rotateToBodyFrame(const float aImu[3], const float wImu[3],
                       float aBody[3], float wBody[3]) {
  for (int i = 0; i < 3; ++i) {
    wBody[i] = gMountDCM[i][0]*wImu[0] + gMountDCM[i][1]*wImu[1] + gMountDCM[i][2]*wImu[2];
    aBody[i] = gMountDCM[i][0]*aImu[0] + gMountDCM[i][1]*aImu[1] + gMountDCM[i][2]*aImu[2];
  }
}

static inline float norm3(const float v[3]) { return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
static inline void normalize3(float v[3]) {
  const float n = norm3(v);
  if (n > 1e-9f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}
static inline void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}
static inline float dot3(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float ghat[3] = { 0.0f, 0.0f, 1.0f };
float w_b[3]  = { 0.0f, 0.0f, 0.0f };
static float gGyroBiasBody[3] = { 0.0f, 0.0f, 0.0f };

void calibrateGyroBias(uint16_t n_samples) {
  float sum[3] = { 0.0f, 0.0f, 0.0f };
  for (uint16_t i = 0; i < n_samples; ++i) {
    imu.getSensorData();
    float aImu[3], wImu[3], aBody[3], wBody[3];
    readIMURaw(imu, aImu, wImu);
    rotateToBodyFrame(aImu, wImu, aBody, wBody);
    sum[0] += wBody[0]; sum[1] += wBody[1]; sum[2] += wBody[2];
    delay(2);
  }
  gGyroBiasBody[0] = sum[0]/n_samples;
  gGyroBiasBody[1] = sum[1]/n_samples;
  gGyroBiasBody[2] = sum[2]/n_samples;
}

static const float kP_FILT = 4.0f;   // HARD CLIFF between 6 and 7 -- do not raise
static const float kI_FILT = 0.5f;
static float bhat[3] = { 0.0f, 0.0f, 0.0f };
static bool gEstimatorInit = false;

void attitudeUpdate(const float a_imu[3], const float w_imu[3], float dt) {
  float aBody[3], wBodyRaw[3];
  rotateToBodyFrame(a_imu, w_imu, aBody, wBodyRaw);
  const float wRaw[3] = {
    wBodyRaw[0] - gGyroBiasBody[0],
    wBodyRaw[1] - gGyroBiasBody[1],
    wBodyRaw[2] - gGyroBiasBody[2],
  };

  float ga[3] = { aBody[0], aBody[1], aBody[2] };
  normalize3(ga);

  if (!gEstimatorInit) {
    ghat[0] = ga[0]; ghat[1] = ga[1]; ghat[2] = ga[2];
    gEstimatorInit = true;
    return;
  }

  float e[3];
  cross3(ghat, ga, e);
  const float wc[3] = { wRaw[0]-bhat[0], wRaw[1]-bhat[1], wRaw[2]-bhat[2] };
  float wxg[3]; cross3(wc, ghat, wxg);
  float exg[3]; cross3(e, ghat, exg);
  for (int i = 0; i < 3; ++i) { ghat[i] += dt * (-wxg[i] + kP_FILT * exg[i]); }
  normalize3(ghat);
  for (int i = 0; i < 3; ++i) { bhat[i] += kI_FILT * e[i] * dt; }   // PLUS
  for (int i = 0; i < 3; ++i) { w_b[i] = wRaw[i] - bhat[i]; }

  static int badCount = 0;
  if (dot3(ghat, ga) < 0.0f) {
    badCount++;
    if (badCount > 20) {
      ghat[0] = ga[0]; ghat[1] = ga[1]; ghat[2] = ga[2];
      bhat[0] = bhat[1] = bhat[2] = 0.0f;
      badCount = 0;
    }
  } else {
    badCount = 0;
  }
}


// ----------------------------------------------------------------------------
// SECTION 2b-2: MODE-AUGMENTED KALMAN FILTER -- Kalman-Filter-Rate-
// Estimator-Evaluation-2026-08-20.md Option 2
// ----------------------------------------------------------------------------
// w_b (above) stays raw -- this is a SEPARATE signal, used only in
// commandWheels()'s om block (same scope as the first-order filter this
// replaces). See the header's "REAL, UNRESOLVED RISK" note before
// trusting the defaults below.
//
// Per-axis 3-state Kalman filter, state x = [om_true; eta; eta_dot]:
//   om_true  -- the rate estimate that feeds the control law (random-walk
//               process model -- no rigid-body dynamics term; that's a
//               SEPARATE, not-yet-built upgrade, the design doc's Option 1)
//   eta      -- the structural mode's own modeled angular deflection
//   eta_dot  -- its rate; the mode's estimated CONTRIBUTION to the
//               measured gyro rate is eta_dot, not eta itself
// Measurement: z = om_true + eta_dot + noise -- the raw gyro reading is
// the SUM of real rate and the mode's contribution; the filter's whole
// job is splitting that sum back into its two parts.
//
// Discrete-time (forward Euler, matching this file's dt):
//   om_true_{k+1} = om_true_k + w1
//   eta_{k+1}     = eta_k + eta_dot_k * dt
//   eta_dot_{k+1} = eta_dot_k + (-wn^2*eta_k - 2*zeta*wn*eta_dot_k)*dt + w2
// i.e. F = [[1,0,0],[0,1,dt],[0,-wn^2*dt,1-2*zeta*wn*dt]], H = [1,0,1].

// wn (mode natural frequency) and zeta (damping ratio) are LIVE-SETTABLE
// ("n<Hz>"/"z<value>") -- see the header's REAL, UNRESOLVED RISK note for
// why: the observed mode frequency varied 29.7-41.0 Hz across three
// sessions, and this default (35 Hz) is the midpoint of that spread, NOT
// a validated number. zeta=0.05 is a generic light-damping guess for an
// aluminum/printed structural assembly -- there is no measurement behind
// it yet at all. Sweep both on the bench; watch mode_x/y/z_dps in
// telemetry for a plausible-looking ~30-40 Hz oscillation as the signal
// that the current setting is at least in the right neighborhood.
static float gModeHz   = 35.0f;
static float gModeZeta = 0.05f;

// Process/measurement noise -- validated in Python against real telemetry
// (design doc Section 2/3) at the DECIMATED ~125 Hz logging rate, NOT
// this file's real 500 Hz loop rate. Discrete process noise scales with
// dt, so these are a starting point to re-tune on hardware, not a result
// to trust at face value -- see the design doc's Section 4 for the dt
// discrepancy this inherits. Not made live-settable (unlike wn/zeta) to
// keep the command surface manageable -- revisit if bench data says
// these need bench sweeping as badly as the mode parameters do.
static const float kQOmTrue = 0.05f;   // (rad/s)^2 per step, om_true process noise
static const float kQEtaDot = 5.0f;    // (rad/s)^2 per step, mode forcing noise
// BMI270's datasheet-informed gyro noise (2.4 mrad/s -- the same figure
// the pre-hardware simulation study used, Cube-Performance-Envelope-
// Results.md), squared for variance. Comparatively well-characterized
// vs Q above.
static const float kRGyro = 0.0024f * 0.0024f;   // (rad/s)^2

// Per-axis filter state: gKfX[axis] = [om_true, eta, eta_dot], gKfP[axis]
// is its 3x3 covariance. om_true (what commandWheels() actually reads)
// and modeContrib (telemetry-only, does NOT feed the control law) are
// exposed as their own arrays for direct printState() access, same
// pattern w_filt used in the file this replaces.
static float gKfX[3][3]    = {{0.0f}};        // [axis][state: om_true,eta,eta_dot]
static float gKfP[3][3][3] = {{{0.0f}}};      // [axis][row][col]
float om_true[3]     = { 0.0f, 0.0f, 0.0f };
float modeContrib[3] = { 0.0f, 0.0f, 0.0f };

void initModeKalman() {
  for (int a = 0; a < 3; ++a) {
    gKfX[a][0] = gKfX[a][1] = gKfX[a][2] = 0.0f;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) { gKfP[a][r][c] = (r == c) ? 1.0e-2f : 0.0f; }
    }
  }
}

// BUG FOUND 2026-08-20 on real hardware (via the WiFi port -- a byte-for-
// byte copy of this math, so this was a latent defect here in the
// ORIGINAL, not a porting error): telemetry showed om_x/y/z_filt_dps and
// mode_x/y/z_dps stuck at "nan" from within ~10s of boot, on every arm
// attempt, silently (isfinite(u) in commandWheels() correctly caught the
// resulting NaN torque and disarmed, but with nothing printed and no way
// to recover without a reboot -- indistinguishable from "arming does
// nothing" at the terminal). Root cause: f22 = 1 - 2*zeta*wn*dt is this
// filter's eta_dot pole, and it leaves the stable unit disk for large dt
// at realistic wn/zeta (e.g. wn=2*pi*45, zeta=0.15, dt=0.01s already
// gives |f22|>1) -- a single cycle with an anomalously large dt (a CAN/
// SPI hiccup) can blow up x[2]/P in one step, and nothing here previously
// detected or recovered from that. Two fixes, both defense-in-depth:
//   1. Clamp dt tighter than loop()'s general 0.5s stall-fallback,
//      specifically for the stability this pole needs.
//   2. Detect non-finite state/covariance per axis and reset JUST that
//      axis -- same "detect divergence, reset" pattern attitudeUpdate()
//      already uses for ghat/bhat (see its badCount threshold above) --
//      so a bad cycle costs one axis's convergence, not a permanent
//      NaN-lock requiring a power cycle.
// BUG FOUND 2026-08-20 on real hardware, PART 2 (same day, after the
// part-1 fix above shipped and a field report came back: correct corner
// [-1,-1,-1], sub-1-degree placement, still "goes absolutely bananas").
// Part 1 fixed a SYMPTOM (NaN-lock on an anomalous dt spike); this is
// the actual disease. F's eta/eta_dot rows above were forward-Euler --
// eta_{k+1}=eta_k+eta_dot_k*dt, eta_dot_{k+1}=eta_dot_k+(-wn^2*eta_k
// -2*zeta*wn*eta_dot_k)*dt -- and forward Euler applied to a lightly
// damped oscillator is only stable for dt < 2*zeta/wn. At this file's
// OWN compiled-in defaults (gModeHz=35, gModeZeta=0.05) that threshold
// is 0.46 ms -- 4.3x SMALLER than the loop's ordinary 2 ms period. Direct
// eigenvalue check of the old F confirms it: |eigenvalue| = 1.072 at
// (35 Hz, zeta=0.05, dt=2ms), i.e. the (eta,eta_dot) state amplitude grew
// ~7.2% EVERY control cycle, doubling roughly every 20 ms -- present at
// the ordinary nominal dt, not just under a timing spike, and true across
// the ENTIRE documented n<Hz>/z<value> sweep range (25-45 Hz x 0.02-0.15,
// all unstable at dt=2ms, checked numerically, not assumed). Because the
// filter runs continuously from boot regardless of arm state, it can
// already be mid-blowup the moment "a1" is sent, dumping a large wrong
// eta_dot straight into om_true's innovation right as the law engages --
// exactly "goes bananas immediately on arm" with correct placement and
// the correct corner. Part 1's self-heal (below) kept catching this once
// it went non-finite, but the instability regrew every cycle after each
// reset, so the wrong correction kept recurring rather than actually
// going away.
//
// FIX: replace forward Euler for the (eta,eta_dot) block with the EXACT
// (zero-order-hold) discretization of a damped harmonic oscillator --
// the closed-form matrix exponential of [[0,1],[-wn^2,-2*zeta*wn]].
// That block's eigenvalues are exp(-zeta*wn*dt) exactly: strictly inside
// the unit disk for ANY zeta>0 and ANY dt>0, by construction -- not a
// tuned-for-these-defaults fix, stable across the whole sweep range.
// om_true's row (still a pure random walk, F row [1,0,0]) is untouched.
void updateModeKalman(float dt) {
  // Still clamp dt -- no longer needed for THIS block's stability (the
  // exact discretization below is unconditionally stable for any dt), but
  // an extreme dt still means the fixed per-step Qdiag below and the
  // gyro-innovation timing assumption stop being a sane approximation, so
  // keep the guard. IN ADDITION TO loop()'s existing 0.5s clamp, not a
  // replacement for it -- that one guards every other estimator here too.
  if (dt > 0.01f) { dt = 0.01f; }

  const float wn   = 2.0f * (float)PI * gModeHz;
  const float zeta = gModeZeta;
  // wd = damped natural frequency. gModeZeta is refused outside (0,1) by
  // the 'z' command handler, so zeta<1 always holds here -- the
  // underdamped closed form below is always the right branch.
  const float wd    = wn * sqrtf(fmaxf(1.0f - zeta * zeta, 1.0e-6f));
  const float decay = expf(-zeta * wn * dt);
  const float cwd   = cosf(wd * dt);
  const float swd   = sinf(wd * dt);
  const float zwwd  = (zeta * wn) / wd;
  const float f11 = decay * (cwd + zwwd * swd);
  const float f12 = decay * (swd / wd);
  const float f21 = decay * (-(wn * wn / wd) * swd);
  const float f22 = decay * (cwd - zwwd * swd);
  const float F[3][3] = { { 1.0f, 0.0f, 0.0f },
                           { 0.0f, f11,  f12  },
                           { 0.0f, f21,  f22  } };
  const float Qdiag[3] = { kQOmTrue, 0.0f, kQEtaDot };

  for (int a = 0; a < 3; ++a) {
    float* x = gKfX[a];

    // ---- predict state: x = F*x ----
    const float x0 = x[0], x1 = x[1], x2 = x[2];
    x[0] = x0;
    x[1] = f11 * x1 + f12 * x2;
    x[2] = f21 * x1 + f22 * x2;

    // ---- predict covariance: P = F*P*F' + Q ----
    float FP[3][3];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        FP[i][j] = F[i][0]*gKfP[a][0][j] + F[i][1]*gKfP[a][1][j] + F[i][2]*gKfP[a][2][j];
      }
    }
    float Ppred[3][3];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        Ppred[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1] + FP[i][2]*F[j][2];
      }
      Ppred[i][i] += Qdiag[i];
    }

    // ---- update against this axis's raw gyro reading, H = [1,0,1] ----
    // Hph = Ppred*H' (a column here, but by Ppred's symmetry it equals
    // H*Ppred too, a row -- reused below for the covariance update so
    // this is computed once, not twice).
    const float Hph[3] = { Ppred[0][0]+Ppred[0][2],
                            Ppred[1][0]+Ppred[1][2],
                            Ppred[2][0]+Ppred[2][2] };
    const float S = Hph[0] + Hph[2] + kRGyro;   // H*Ppred*H' + R
    const float K[3] = { Hph[0]/S, Hph[1]/S, Hph[2]/S };

    const float yInnov = w_b[a] - (x[0] + x[2]);   // z - H*x_pred
    x[0] += K[0] * yInnov;
    x[1] += K[1] * yInnov;
    x[2] += K[2] * yInnov;

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) { gKfP[a][i][j] = Ppred[i][j] - K[i]*Hph[j]; }
    }

    // Self-healing: reset JUST this axis if the update produced a
    // non-finite state or covariance, instead of staying NaN-locked
    // until reboot (see the BUG FOUND note above).
    bool bad = !isfinite(x[0]) || !isfinite(x[1]) || !isfinite(x[2]);
    for (int r = 0; r < 3 && !bad; ++r) {
      for (int c = 0; c < 3; ++c) {
        if (!isfinite(gKfP[a][r][c])) { bad = true; break; }
      }
    }
    if (bad) {
      x[0] = x[1] = x[2] = 0.0f;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) { gKfP[a][r][c] = (r == c) ? 1.0e-2f : 0.0f; }
      }
      Serial.print("# KF axis "); Serial.print(a == 0 ? "X" : a == 1 ? "Y" : "Z");
      Serial.println(" reset (state/covariance went non-finite)");
    }

    om_true[a]     = x[0];
    modeContrib[a] = x[2];
  }
}


// ----------------------------------------------------------------------------
// SECTION 2c: CORNER CANDIDATE (resolved fresh here too)
// ----------------------------------------------------------------------------

struct CornerCandidate {
  const char* name;
  float gB[3];
  float Kp[3][9];   // rows = wheel X,Y,Z; cols = [phi(3) om(3) rho(3)]
  float placeOffsetDeg;
  float ellM;       // m, contact-to-COM lever arm -- NEW vs Stage4_FullLaw.ino,
                     // used only to convert a converged trim into an
                     // equivalent COM offset in mm for telemetry (Automatic
                     // Trim.md S4's "log trim continuously" guard).
};

// >>> TODO: MIXED-GENERATION PLANT. Corner [-1,-1,-1]'s Kp/ell/theta_eq
// below were UPDATED 2026-08-19 for mass 1.633 kg + the new corner-to-
// housing strut (user-supplied); the OTHER SEVEN corners are still the
// 1.5668 kg / no-strut export from cubli_gains.h (2026-08-14). This
// table is NOT internally consistent across corners -- fine for
// continued single-corner [-1,-1,-1] bring-up (this file's rate-filter/
// k_a fixes were verified against the old table, a legitimate baseline
// to tune the new Kp against), but do not resolve onto or trust any
// OTHER corner's numbers until they're re-derived the same way.
// [-1,-1,-1]'s own gB (corner-resolution direction) was NOT part of
// this update -- see that entry's own comment for why this is low-risk.
// Swap in the remaining seven corners' gB/Kp[3][9]/ell/Sg/lambda/theta_eq
// the moment they're available, same 8-entry shape as below.
static const CornerCandidate kCorners[8] = {
  { "[-1,-1,-1]"  // UPDATED 2026-08-19 for mass 1.6330 kg (+4.2%) + the new
                  // corner-to-housing strut (mounted 4.49 deg off the
                  // balancing diagonal, near-axial -- "barely hurts" per
                  // the source note). theta_eq 0.894 deg vs body diagonal
                  // (was 0.797), ell 121.08 mm (was 122.84), Sg 1.9390
                  // (was 1.8875), lambda 8.3688 s^-1 (was 8.2572).
                  // Recovery 2.89 deg (was 3.14), margin +2.00 deg -- this
                  // corner and its diagonal twin [+1,+1,+1] are still
                  // safely inside their recovery envelope; the OTHER SIX
                  // corners are NOT with this same pole (see Cube-
                  // Performance-Envelope-Results.md) -- corner balancing
                  // (this file's test) is unaffected, multi-corner
                  // locomotion is not, until the pole is rebalanced or a
                  // counterweight is added.
                  // Robustness (source diagnostics): Ms=0.999999, slowest
                  // mode -7.93 s^-1 (126.1 ms), |K1| spread 1.022, discrete
                  // max|z|=0.9813, robust +/-30% worst -1.947, momentum-
                  // limited bound 4.81 deg vs a 5.02 deg torque bound.
                  // gB below is now ALSO the confirmed new direction
                  // (2026-08-19, second update) -- this corner's full set
                  // (gB/Kp/ell/theta_eq) is complete and matched. The
                  // other seven corners' gB/ell/Sg/lambda/theta_eq are now
                  // known too (see Cube-Performance-Envelope-Results.md)
                  // but their Kp[3][9] are NOT -- do not extend this
                  // update to them. Per that doc: each corner needs its
                  // OWN gains, not a preference -- applying THIS corner's
                  // Kp elsewhere diverges at essentially the open-loop
                  // rate on six of eight corners, and the antipodal
                  // corner [+1,+1,+1] becomes a 13s slow fall a <15s test
                  // would score as a false pass.
    , { -0.57067251f, -0.59002078f, -0.57114653f }
    , { { -7.0685f, 3.4801f, 3.4675f, -0.8446f, 0.4235f, 0.4244f, -0.000998f, 0.006001f, 0.005931f },  // wheel X
        { 3.5743f, -6.9184f, 3.5758f, 0.4173f, -0.8435f, 0.4177f, 0.004252f, -0.002662f, 0.004252f },  // wheel Y
        { 3.4703f, 3.4844f, -7.0670f, 0.4242f, 0.4237f, -0.8452f, 0.005879f, 0.005948f, -0.001049f } } // wheel Z
    , 0.894f, 0.12108f },
  { "[-1,-1,+1]"  // lean 3.170 deg vs body diagonal, ell 128.54 mm, Sg 1.9750, lambda 8.1212
    , { -0.546220303f, -0.562558711f, 0.620621562f }
    , { { -7.58488846f, 3.33574939f, -3.65192771f, -0.921932399f, 0.430031687f, -0.480414748f, 0.000473975349f, 0.00748626003f, -0.00814931281f },  // wheel X
        { 3.43748641f, -7.47500134f, -3.7502768f, 0.421860248f, -0.928083122f, -0.467502952f, 0.00511831883f, -0.00179392833f, -0.00555475708f },  // wheel Y
        { -3.84645557f, -3.83494377f, -6.8614974f, -0.464526713f, -0.460435808f, -0.881195128f, -0.00357620185f, -0.00353527209f, -0.00314604398f } } // wheel Z
    , 3.170f, 0.128537506f },
  { "[-1,+1,-1]"  // lean 2.773 deg vs body diagonal, ell 126.08 mm, Sg 1.9373, lambda 8.1591
    , { -0.556852818f, 0.616181135f, -0.55698812f }
    , { { -7.26747227f, -3.47609496f, 3.4201951f, -0.874630272f, -0.451983064f, 0.442295492f, 0.000827456824f, -0.00813719723f, 0.00776857371f },  // wheel X
        { -3.79863954f, -6.87362671f, -3.8063941f, -0.426038593f, -0.899968863f, -0.426554501f, -0.000622705789f, -0.00668813102f, -0.000633999531f },  // wheel Y
        { 3.41229653f, -3.47500682f, -7.25577497f, 0.442907125f, -0.45318532f, -0.871017814f, 0.00794851314f, -0.0083494084f, 0.00103329937f } } // wheel Z
    , 2.773f, 0.126083225f },
  { "[-1,+1,+1]"  // lean 3.097 deg vs body diagonal, ell 131.64 mm, Sg 2.0226, lambda 8.0480
    , { -0.533349514f, 0.590173781f, 0.605997622f }
    , { { -8.23398495f, -3.56334782f, -3.77657819f, -1.06839609f, -0.416523129f, -0.442939252f, -0.00582132814f, -0.00140620384f, -0.00158187468f },  // wheel X
        { -3.41282964f, -7.33259392f, 4.13743114f, -0.429089457f, -0.917372108f, 0.514714897f, -0.00497917458f, -0.00120868487f, 0.00603242731f },  // wheel Y
        { -3.50851822f, 4.01196194f, -6.99511194f, -0.465059072f, 0.524895251f, -0.847355127f, -0.00789372995f, 0.00895721f, 0.00243827817f } } // wheel Z
    , 3.097f, 0.131639361f },
  { "[+1,-1,-1]"  // lean 3.171 deg vs body diagonal, ell 128.56 mm, Sg 1.9753, lambda 8.1180
    , { 0.620658159f, -0.562471628f, -0.546268404f }
    , { { -6.85864925f, -3.8295908f, -3.8494637f, -0.880151272f, -0.460793495f, -0.465634525f, -0.00304374471f, -0.00361368596f, -0.00366454106f },  // wheel X
        { -3.75562048f, -7.4855423f, 3.44052219f, -0.467026591f, -0.931139171f, 0.421110034f, -0.00539576123f, -0.00195194408f, 0.0049761422f },  // wheel Y
        { -3.65655398f, 3.33088136f, -7.58417654f, -0.481462985f, 0.429957539f, -0.921488523f, -0.00823030341f, 0.00754544372f, 0.000543692615f } } // wheel Z
    , 3.171f, 0.128557414f },
  { "[+1,-1,+1]"  // lean 2.609 deg vs body diagonal, ell 134.01 mm, Sg 2.0591, lambda 7.9804
    , { 0.595400333f, -0.539581716f, 0.595273018f }
    , { { -7.30901575f, -3.4263885f, 4.20474958f, -0.895961821f, -0.451509207f, 0.553766072f, 0.00180089788f, -0.00741348462f, 0.00873730052f },  // wheel X
        { -3.81644511f, -8.42617893f, -3.82059836f, -0.41949293f, -1.11799276f, -0.419535488f, 0.00164802792f, -0.00882489327f, 0.00163950643f },  // wheel Y
        { 4.19695139f, -3.42353535f, -7.30109262f, 0.554382324f, -0.452106625f, -0.892860353f, 0.00891439989f, -0.00758054852f, 0.00199437374f } } // wheel Z
    , 2.609f, 0.134011015f },
  { "[+1,+1,-1]"  // lean 3.095 deg vs body diagonal, ell 131.66 mm, Sg 2.0229, lambda 8.0501
    , { 0.606037736f, 0.590086639f, -0.533400357f }
    , { { -6.99898767f, 4.01700401f, -3.50819087f, -0.848652959f, 0.524742544f, -0.464503765f, 0.00232795905f, 0.00885933265f, -0.00779828522f },  // wheel X
        { 4.13386154f, -7.32801533f, -3.40998602f, 0.515303075f, -0.915354431f, -0.429458082f, 0.00615369575f, -0.00108131079f, -0.00508674001f },  // wheel Y
        { -3.77546f, -3.56726503f, -8.23596478f, -0.442555219f, -0.416409701f, -1.06887817f, -0.00154884392f, -0.00138305803f, -0.00584927434f } } // wheel Z
    , 3.095f, 0.131658807f },
  { "[+1,+1,+1]"  // lean 0.714 deg vs body diagonal, ell 136.99 mm, Sg 2.1048, lambda 7.9341
    , { 0.582457066f, 0.567126632f, 0.582332492f }
    , { { -7.76383209f, 3.81748652f, 4.04768848f, -0.975789487f, 0.490672857f, 0.522951066f, -0.000524852891f, 0.00608501304f, 0.00639130361f },  // wheel X
        { 3.93991017f, -8.08397388f, 3.93213129f, 0.481777757f, -1.03972197f, 0.481087863f, 0.00368249253f, -0.00348382187f, 0.00367164146f },  // wheel Y
        { 4.05641413f, 3.81822968f, -7.77580976f, 0.522317231f, 0.489363909f, -0.97946316f, 0.00622009346f, 0.00590694463f, -0.000720091164f } } // wheel Z
    , 0.714f, 0.136988997f },
};

int gCornerIdx = 0;

void resolveCornerCandidate() {
  int bestIdx = 0, secondIdx = 0;
  float bestDot = -2.0f, secondDot = -2.0f;
  for (int i = 0; i < 8; ++i) {
    const float d = dot3(ghat, kCorners[i].gB);
    if (d > bestDot) { secondDot = bestDot; secondIdx = bestIdx; bestDot = d; bestIdx = i; }
    else if (d > secondDot) { secondDot = d; secondIdx = i; }
  }
  gCornerIdx = bestIdx;
  Serial.print("# corner resolved: "); Serial.print(kCorners[gCornerIdx].name);
  Serial.print("  place_offset="); Serial.print(kCorners[gCornerIdx].placeOffsetDeg, 3);
  Serial.print(" deg  (best_dot="); Serial.print(bestDot, 4);
  Serial.print(" runner_up="); Serial.print(kCorners[secondIdx].name);
  Serial.print(" dot="); Serial.print(secondDot, 4); Serial.println(")");
  if (bestDot - secondDot < 0.2f) {
    Serial.println("# WARNING: best and runner-up corners are close -- verify before arming.");
  }
}

float phi[3] = { 0.0f, 0.0f, 0.0f };

// FIXED OFFSET -- see the header's FIXED OFFSET AND ARM GATE note for
// provenance (perfect_equilibrium_2.log, mean over 132.41s, std<0.0012
// deg/axis). ADDED to the raw measurement below, same sign/convention
// gTrim used (verified on the bench, cos(trim,wheels)=-1.0000 -- see
// Stage4_AutoTrim.ino's header) -- this is that same verified value,
// just no longer recomputed live.
static const float kPhiOffset[3] = {
  0.004580f,   // rad =  0.2624 deg
  0.003952f,   // rad =  0.2265 deg
  -0.008430f,  // rad = -0.4830 deg
};

void updateCornerProjection() {
  float t[3];
  cross3(kCorners[gCornerIdx].gB, ghat, t);
  phi[0] = -t[0] + kPhiOffset[0];
  phi[1] = -t[1] + kPhiOffset[1];
  phi[2] = -t[2] + kPhiOffset[2];
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------

void printState(uint32_t t_ms, const float rho[3], const float rhoLp[3],
                const float tau[3], const float tauCmd[3], bool armed,
                float gainScale) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(phi[0] * (float)RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(phi[1] * (float)RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(phi[2] * (float)RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(w_b[0] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(w_b[1] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(w_b[2] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(rho[0], 3);
  Serial.print('\t'); Serial.print(rho[1], 3);
  Serial.print('\t'); Serial.print(rho[2], 3);
  Serial.print('\t'); Serial.print(rhoLp[0], 3);
  Serial.print('\t'); Serial.print(rhoLp[1], 3);
  Serial.print('\t'); Serial.print(rhoLp[2], 3);
  Serial.print('\t'); Serial.print(tau[0], 4);
  Serial.print('\t'); Serial.print(tau[1], 4);
  Serial.print('\t'); Serial.print(tau[2], 4);
  Serial.print('\t'); Serial.print(tauCmd[0], 4);
  Serial.print('\t'); Serial.print(tauCmd[1], 4);
  Serial.print('\t'); Serial.print(tauCmd[2], 4);
  Serial.print('\t'); Serial.print(armed ? 1 : 0);
  Serial.print('\t'); Serial.print(gainScale, 2);
  // Same 5 columns the AutoTrim-family telemetry has, kept for format
  // compatibility -- but these are now CONSTANTS, not a live readout.
  // trim_enabled=0 always, signaling "fixed, not adapting."
  Serial.print('\t'); Serial.print(kPhiOffset[0] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(kPhiOffset[1] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(kPhiOffset[2] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(norm3(kPhiOffset) * kCorners[gCornerIdx].ellM * 1000.0f, 3);
  Serial.print('\t'); Serial.print(0);
  // Kalman om_true readout (same column names Stage4_AutoTrim_RateFilter
  // .ino's w_filt used -- compare against raw om_x/y/z_dps above) --
  // compare against mode_x/y/z_dps below to see whether the filter's
  // rejection is coming from a genuine mode estimate or not.
  Serial.print('\t'); Serial.print(om_true[0] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(om_true[1] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(om_true[2] * (float)RAD_TO_DEG, 2);
  // Mode-contribution readout, NEW vs the plain rate-filter files --
  // see the header's STAGE 4 CHECKLIST: this should look like a genuine
  // ~30-40 Hz oscillation, not noise or a flat line.
  Serial.print('\t'); Serial.print(modeContrib[0] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(modeContrib[1] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.println(modeContrib[2] * (float)RAD_TO_DEG, 2);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL — full law: phi + om + rho, plus friction FF
// ----------------------------------------------------------------------------

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // already validated through Stage 3's ramp

// Loose this stage -- hand-held, watching combined-term behavior. Stage 5
// tightens to the real DISARM/OMEGA_CAP policy from cubli_gains.h.
static const float kMaxTilt    = 0.4363f;   // rad, 25 deg, vs norm3(phi)
// Velocity cap loosened from the 40 rad/s policy value (see header note
// above), but not literally removed -- set to 2000 RPM, the motor's real
// mechanical speed rating, so there's still a genuine hardware ceiling
// behind it rather than "effectively infinite." TAU_MAX below is
// untouched and is still the real physical torque saturation.
static const float kMaxOmega   = 209.43951f;    // rad/s (2000 RPM, motor rating)
static const float kTauMax     = 0.12f;     // N*m, TAU_MAX
static const float kTaperStart = 36.0f;     // rad/s -- irrelevant now: with
                                              // kMaxOmega this large, the
                                              // taper's fade factor stays
                                              // ~1.0 for any real wheel speed

// Friction feedforward -- Firmware Lessons: "not optional". PLACEHOLDER
// values from cubli_gains.h pending the real spin-down/breakaway test.
// Same values applied to all three wheels (no reason to expect them to
// differ between wheels of the same motor/mount design, but this is a
// PLACEHOLDER assumption too -- flag if one wheel's standing speed clearly
// misbehaves relative to the other two).
static const float kTauCw  = 0.008f;   // N*m, PLACEHOLDER
static const float kBw     = 0.0f;     // N*m*s, PLACEHOLDER
static const float kEpsFf  = 0.05f;    // rad/s, tanh width

// NO ARM GATE HERE -- see the header's FIXED OFFSET AND ARM GATE note.
// kArmGate existed to refuse "a1" unless already near the resolved
// corner's equilibrium; that check is gone from the 'a' command below,
// deliberately.

static const float kAxisWheelSign[3] = {
  1.0f,   // X -- CONFIRMED
  1.0f,   // Y -- CONFIRMED
  1.0f,   // Z -- CONFIRMED
};

static Moteus::PositionMode::Format kTorqueFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque    = Moteus::kFloat;
  f.kp_scale               = Moteus::kFloat;
  f.kd_scale                = Moteus::kFloat;
  f.maximum_torque          = Moteus::kFloat;
  f.watchdog_timeout        = Moteus::kFloat;
  f.ignore_position_bounds  = Moteus::kFloat;
  return f;
}();

static float gLastTau[3]      = { 0.0f, 0.0f, 0.0f };
static float gLastTauCmd[3]   = { 0.0f, 0.0f, 0.0f };
static float gRhoLp[3]        = { 0.0f, 0.0f, 0.0f };   // standing speed, tau = 5 s

Moteus& wheelObj(int i) {
  return i == 0 ? moteusX : (i == 1 ? moteusY : moteusZ);
}

void commandWheels(const float rho[3]) {
  // x = [phi(3); om(3); rho(3)] -- full state, all nine columns of each
  // wheel's Kp row now contribute. om uses om_true (the mode-augmented
  // Kalman filter's rate estimate), NOT raw w_b -- see SECTION 2b-2 and
  // the header. Everything else in the law (Kp itself, phi, rho) is
  // untouched, same scope hw-run-analysis.md's fix 4.1 had.
  const float xVec[9] = {
    phi[0], phi[1], phi[2],
    om_true[0], om_true[1], om_true[2],
    rho[0], rho[1], rho[2],
  };

  float tau[3];
  for (int i = 0; i < 3; ++i) {
    const float* row = kCorners[gCornerIdx].Kp[i];
    float u = 0.0f;
    for (int j = 0; j < 9; ++j) { u -= row[j] * xVec[j]; }
    u *= gGainScale;
    u += kTauCw * tanhf(rho[i] / kEpsFf) + kBw * rho[i];

    const bool spinning_up = (u >= 0.0f) == (rho[i] >= 0.0f);
    if (spinning_up) {
      float s = (kMaxOmega - fabsf(rho[i])) / (kMaxOmega - kTaperStart);
      s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
      u *= s;
    }

    if (!isfinite(u)) { u = 0.0f; gArmed = false; }
    u = u >  kTauMax ?  kTauMax : u;
    u = u < -kTauMax ? -kTauMax : u;
    tau[i] = u;
  }

  if (norm3(phi) > kMaxTilt) { gArmed = false; }
  for (int i = 0; i < 3; ++i) {
    if (fabsf(rho[i]) > kMaxOmega) { gArmed = false; }
  }
  if (!isfinite(phi[0]) || !isfinite(phi[1]) || !isfinite(phi[2]) ||
      !isfinite(w_b[0]) || !isfinite(w_b[1]) || !isfinite(w_b[2])) {
    gArmed = false;
  }

  static const float kLpTau = 5.0f;
  const float dtNom = kPeriodMs * 1e-3f;
  const float alphaLp = dtNom / (kLpTau + dtNom);

  for (int i = 0; i < 3; ++i) {
    const float tau_cmd = gArmed ? tau[i] : 0.0f;
    Moteus::PositionMode::Command cmd;
    cmd.position               = NaN;
    cmd.velocity               = 0.0f;
    cmd.kp_scale                = 0.0f;
    cmd.kd_scale                 = 0.0f;
    cmd.feedforward_torque       = kAxisWheelSign[i] * tau_cmd;
    cmd.maximum_torque           = kTauMax;
    cmd.watchdog_timeout          = 0.10f;
    cmd.ignore_position_bounds    = 1.0f;
    wheelObj(i).SetPosition(cmd, &kTorqueFormat);

    gRhoLp[i] += alphaLp * (rho[i] - gRhoLp[i]);
    gLastTau[i]    = tau[i];
    gLastTauCmd[i] = tau_cmd;
  }
}


// SECTION 2e-2 (AUTOMATIC TRIM ADAPTATION) removed -- see the header's
// FIXED OFFSET AND ARM GATE note. gTrim/gKAdapt/updateTrim()/
// applyTrimGuards() are gone; kPhiOffset above is a compile-time
// replacement, not a live-adapting one. SECTION 2b-2 (the mode-augmented
// Kalman filter) is UNCHANGED and still runs every cycle -- only the
// trim mechanism is removed here.


// ----------------------------------------------------------------------------
// SECTION 2f: SERIAL COMMANDS
// ----------------------------------------------------------------------------

void handleSerialCommands() {
  if (!Serial.available()) { return; }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) { return; }

  const char cmd = line.charAt(0);
  const float val = line.substring(1).toFloat();

  if (cmd == 'a') {
    // NO ARM GATE -- see the header's FIXED OFFSET AND ARM GATE note.
    // Arms unconditionally on "a1", regardless of current tilt.
    if (val != 0.0f) {
      gArmed = true;
      Serial.println("# gArmed = TRUE (no arm-gate check -- operator's call)");
    } else {
      gArmed = false;
      Serial.println("# gArmed = FALSE");
    }
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
    Serial.print("# gGainScale = "); Serial.println(gGainScale, 3);
  } else if (cmd == 'c') {
    resolveCornerCandidate();
  } else if (cmd == 'n') {
    // Mode natural frequency, Hz -- see the header's REAL, UNRESOLVED
    // RISK note: observed 29.7-41.0 Hz across sessions, sweep this live
    // rather than reflashing per guess. Refuses <=0 (undefined wn^2 sign
    // makes no physical sense) instead of silently producing garbage.
    if (val > 0.0f) {
      gModeHz = val;
      Serial.print("# gModeHz = "); Serial.print(gModeHz, 1);
      Serial.println(" Hz");
    } else {
      Serial.println("# REFUSED: n<Hz> needs a positive value (try 25-45).");
    }
  } else if (cmd == 'z') {
    // Mode damping ratio zeta -- pure guess by default (0.05), no
    // measurement behind it at all. Refuses <=0 (undamped/growing mode
    // model) and >=1 (overdamped, not a resonance anymore -- if the real
    // structure needs that, this isn't the right filter shape for it).
    if (val > 0.0f && val < 1.0f) {
      gModeZeta = val;
      Serial.print("# gModeZeta = "); Serial.println(gModeZeta, 4);
    } else {
      Serial.println("# REFUSED: z<value> needs 0 < zeta < 1 (try 0.02-0.15).");
    }
  } else if (cmd == 'h') {
    gHalted = (val != 0.0f);
    if (gHalted && gArmed) {
      gArmed = false;
      Serial.println("# disarmed by halt");
    }
    Serial.print("# gHalted = ");
    Serial.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                            : "FALSE (resumed, still DISARMED -- send a1)");
  } else {
    Serial.println("# unknown. use: a<0/1> (arm/disarm, no gate)  "
                    "g<0..1> (gain scale)  c (re-resolve)  n<Hz> (mode "
                    "natural freq)  z<0-1> (mode damping zeta)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 4: MODE-AUGMENTED KALMAN FILTER + FIXED OFFSET, NO ARM GATE (cube held by hand) -- EXPERIMENTAL, read the header");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  moteusX.SetStop();
  moteusY.SetStop();
  moteusZ.SetStop();
  Serial.println("all stopped");

  updateMountingDCM();
  checkMountingDCMValid();

  pinMode(imuChipSelectPin, OUTPUT);
  digitalWrite(imuChipSelectPin, HIGH);
  SPI.begin();

  while (imu.beginSPI(imuChipSelectPin, imuClockFrequency) != BMI2_OK) {
    Serial.println("Error: BMI270 not connected, check wiring and CS pin!");
    delay(1000);
  }
  Serial.println("BMI270 connected!");

  // 800 Hz, not 400 -- cube-bringup/Stage0c_IMUJitter.ino already found and
  // documented this exact problem (Phase 0.4): this loop runs at 500 Hz
  // (kPeriodMs=2 above), but every OTHER file in this progression still
  // configured the IMU's own output rate at 400 Hz, meaning the loop polls
  // FASTER than the sensor produces new samples -- roughly 1 in 5 reads is
  // a stale repeat of the previous one, not new information. Stage0c
  // tested and validated 800 Hz specifically to fix this (bwp left at
  // default there too -- see that file's own TODO on re-deriving group
  // delay for the real (ODR, bwp) pair if it ever matters). Found via the
  // Fine-Tuning Test Plan's Test 2 (estimator lag audit) code-read.
  if (imu.setAccelODR(BMI2_ACC_ODR_800HZ) != BMI2_OK ||
      imu.setGyroODR(BMI2_GYR_ODR_800HZ)  != BMI2_OK) {
    Serial.println("Warning: could not raise BMI270 ODR to 800 Hz");
  }

  Serial.println("# calibrating gyro bias -- keep the cube PERFECTLY STILL (~2s)");
  calibrateGyroBias(1000);

  for (int i = 0; i < 50; ++i) {
    float aImu[3], wImu[3];
    imu.getSensorData();
    readIMURaw(imu, aImu, wImu);
    attitudeUpdate(aImu, wImu, 0.002f);
    delay(2);
  }
  resolveCornerCandidate();
  initModeKalman();

  Serial.println("t_ms\tphi_x_deg\tphi_y_deg\tphi_z_deg\t"
                  "om_x_dps\tom_y_dps\tom_z_dps\trho_x\trho_y\trho_z\t"
                  "rho_x_lp\trho_y_lp\trho_z_lp\t"
                  "tau_x\ttau_y\ttau_z\ttau_cmd_x\ttau_cmd_y\ttau_cmd_z\t"
                  "armed\tgain_scale\t"
                  "trim_x_deg\ttrim_y_deg\ttrim_z_deg\ttrim_com_mm\ttrim_enabled\t"
                  "om_x_filt_dps\tom_y_filt_dps\tom_z_filt_dps\t"
                  "mode_x_dps\tmode_y_dps\tmode_z_dps");
  Serial.println("# STARTS DISARMED. NO ARM GATE -- a1 arms unconditionally,");
  Serial.println("# regardless of current tilt. Get close to the resolved corner");
  Serial.println("# before arming anyway -- see header 'FIXED OFFSET AND ARM GATE' note.");
  Serial.print("# Fixed offset (not adapting): "); Serial.print(kPhiOffset[0]*(float)RAD_TO_DEG, 4);
  Serial.print(", "); Serial.print(kPhiOffset[1]*(float)RAD_TO_DEG, 4);
  Serial.print(", "); Serial.print(kPhiOffset[2]*(float)RAD_TO_DEG, 4);
  Serial.println(" deg (from perfect_equilibrium_2.log).");
  Serial.print("# Mode-augmented Kalman filter: gModeHz="); Serial.print(gModeHz, 1);
  Serial.print(" gModeZeta="); Serial.print(gModeZeta, 3);
  Serial.println(" -- UNVALIDATED, see header 'REAL, UNRESOLVED RISK'.");
  Serial.println("# n<Hz>/z<0-1> sweep them live. Watch mode_x/y/z_dps in telemetry");
  Serial.println("# for a plausible ~30-40 Hz oscillation before trusting this file.");
  Serial.println("# Velocity cap is loosened this stage (kMaxOmega ~2000 RPM, not");
  Serial.println("# the 40 rad/s policy value) -- a0 (disarm) is the real safety net,");
  Serial.println("# along with kMaxTilt (25 deg auto-trip) since the arm gate is gone.");
  Serial.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");

  gNextSendMillis = millis();
}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  handleSerialCommands();
  if (gHalted) { return; }

  if (static_cast<int32_t>(millis() - gNextSendMillis) < 0) { return; }
  gNextSendMillis += kPeriodMs;

  const uint32_t time = millis();

  static uint32_t lastMicros = 0;
  static bool dtInitialized = false;
  const uint32_t nowMicros = micros();
  float dt = dtInitialized ? (nowMicros - lastMicros) * 1e-6f : kPeriodMs * 1e-3f;
  lastMicros = nowMicros;
  dtInitialized = true;
  if (dt <= 0.0f || dt > 0.5f) { dt = kPeriodMs * 1e-3f; }

  float aImu[3], wImu[3];
  imu.getSensorData();
  readIMURaw(imu, aImu, wImu);
  attitudeUpdate(aImu, wImu, dt);
  updateCornerProjection();
  updateModeKalman(dt);   // before commandWheels() -- this cycle's om_true
                           // must be fresh; commandWheels() reads it directly

  const float rho[3] = {
    kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI,
  };

  commandWheels(rho);
  // no updateTrim() call -- kPhiOffset is fixed, nothing to adapt each cycle

  printState(time, rho, gRhoLp, gLastTau, gLastTauCmd, gArmed, gGainScale);
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// This file vs Stage4_FixedOffset_RateFilter.ino: identical fixed-offset/
// no-arm-gate treatment and identical corner candidate table -- the
// difference is entirely in SECTION 2b-2, a mode-augmented Kalman filter
// (om_true/modeContrib) replacing the first-order low-pass (w_filt). If
// something looks wrong here that isn't about the Kalman filter
// specifically, check whether Stage4_FixedOffset_RateFilter.ino has the
// same problem before assuming this file's own filter caused it -- and if
// something looks wrong that IS about the filter, remember the fallback
// is switching commandWheels()'s xVec back to w_b (or reflashing
// Stage4_FixedOffset_RateFilter.ino), not debugging the Kalman math live
// on a physical corner.
//
// Next steps, in order:
//   - The tap test (still outstanding, on every doc's list this file
//     traces back to) -- strike the frame, log the gyro at max rate, FFT.
//     Confirms the mode's real frequency/damping and whether it's stable
//     enough to trust an explicit model of it at all. Everything below
//     this item is provisional until it's done.
//   - Bench-sweep gModeHz/gModeZeta ("n<Hz>"/"z<value>") watching
//     mode_x/y/z_dps -- this is the practical stand-in for the tap test
//     if it hasn't happened yet, but a real tap test is still the better
//     answer. FINAL/kalman_mode_hint.py automates the starting guess: point
//     it at ANY corner-format capture (this file's own, or a plain
//     AutoTrim/RateFilter one) and it Welch-PSDs om_x/y/z, finds the peak,
//     and prints ready-to-paste "n<Hz>"/"z<value>" commands -- same
//     approach used to validate this filter's design in the first place,
//     not a new/different method.
//   - kQOmTrue/kQEtaDot/kRGyro are compile-time constants, not live-
//     settable -- if bench sweeping wn/zeta alone doesn't get a
//     believable mode_x/y/z_dps trace, these need attention too (see the
//     design doc's Section 4 on why the Python-validated values may not
//     transfer directly to this file's real 500 Hz loop rate).
//   - Once trusted: Option 1 from the design doc (a full rigid-body-model
//     KF reusing cubli_corner_plant.m's (A,B), the dual of Kp's own LQR
//     derivation) is the next upgrade, orthogonal to this file's mode-
//     specific one -- it improves the om_true estimate for everything
//     the rigid-body model captures, this file's addition improves it
//     specifically at the structural mode.
//   - Drop in the real Kp[3][9]/gB/ell/Sg/lambda/theta_eq for the
//     remaining SEVEN corners once available -- see the TODO above
//     kCorners, same open item every file in this family has.
// ============================================================================
