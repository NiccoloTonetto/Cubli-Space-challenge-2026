// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 4: FULL LAW + AUTOMATIC TRIM + RATE LOW-PASS FILTER
// ============================================================================
// Copy of Stage4_AutoTrim.ino with ONE mechanism ADDED: a first-order
// low-pass on the body-rate signal feeding the control law's om block,
// per hw-run-analysis.md's fix 4.1. Read Stage4_AutoTrim.ino's header
// first for the trim mechanism (unchanged here) -- this header only
// covers what's new.
//
// ---------------------------- WHAT AND WHY ---------------------------------
// hw-run-analysis.md (373.5 s continuous corner balance, auto-trim on,
// "corner-balance-373s-2026-08-20.log") found a real problem the sim envelope didn't
// predict: torque saturated 71.8% of the time, and the rate term (K2*om)
// alone demanded 1.5x the available torque (0.180 N*m rms against a
// 0.12 N*m clamp) -- NOT because the cube was fighting a real disturbance
// (saturated vs unsaturated mean tilt were statistically identical,
// 0.365 vs 0.317 deg), but because a ~35 Hz structural mode was riding on
// the gyro signal, in phase across phi, om AND u (a resonant shape white
// noise cannot produce). The mechanism is a closed loop the controller
// cannot break out of:
//
//   35 Hz structural ring -> gyro measures it (real motion, not noise)
//   -> K2 amplifies it into torque -> torque clamps, injecting a square
//   wave -> the square wave re-excites the mode -> repeat
//
// The closed-loop bandwidth here is ~1.3 Hz (the plant's own lambda/2pi).
// A controller with 1.3 Hz bandwidth structurally CANNOT damp a 35 Hz
// mode -- it can only feed it energy. The fix is not a gain change, it's
// keeping that energy out of the gain in the first place: a first-order
// low-pass on the rate signal, corner frequency 15-25 Hz, is invisible to
// a 1.3 Hz control loop (at 20 Hz corner the added phase lag AT 1.3 Hz is
// ~3.7 deg, negligible against a 70 deg phase margin). Nothing else about
// the control law changes -- Kp stays exactly as shipped.
//
// CORRECTION 2026-08-19 (see docs/testing/Corner-RateFilter-and-Edge-
// Hardware-Tests-2026-08-19.md Section 2): an earlier version of this
// comment claimed "20-30 dB of attenuation at 35 Hz" for the 20 Hz corner
// above. That's wrong -- measured attenuation of the real ~30 Hz mode on
// hardware is only ~6 dB (Welch PSD, matches the first-order formula
// sqrt(1+(f/fc)^2) exactly: ~1.8x amplitude at f=30Hz, fc=20Hz). A
// first-order filter CANNOT give both "negligible lag at 1.3 Hz" and
// "20-30 dB at 30-35 Hz" -- those two targets are less than 5 octaves
// apart and a first order only rolls off 6 dB/octave; reaching 20-30 dB
// at 30 Hz would need fc of order 1-3 Hz, which puts 40-60 deg of lag
// back at 1.3 Hz. The filter IS real (measured ~45% RMS reduction on
// |om|, visibly smoother wheel behavior) -- it just takes the edge off
// the mode rather than killing it. A steeper filter (2nd-order, or a
// notch at the measured peak) would be needed to do better without this
// trade-off; the corner-frequency knob alone can't get there.
//
// IMPLEMENTATION: a separate w_filt[3], NOT a change to w_b[3] itself.
// w_b (raw, from attitudeUpdate()) still drives that propagation and every
// safety check (arm gate via phi, the isfinite() trip checks) unfiltered
// and un-delayed -- filtering those would add lag to exactly the signals
// that need to react fastest. w_filt only replaces w_b in commandWheels()'s
// xVec om block (the K2 path hw-run-analysis.md identified), matching the
// note's own scope: "nothing else in the design changes."
//
// ONE DELIBERATE EXTRA CHANGE beyond hw-run-analysis.md's literal 4.1: the
// trim adaptation's quiescence gate (updateTrim()'s kOmegaQuiet check) now
// reads w_filt instead of raw w_b. Justification: the same log showed phi
// (79.7% of energy below 5 Hz) is clean even while om/u are dominated by
// the 35 Hz mode -- the CUBE is genuinely holding still, only the raw rate
// reading is noisy. Gating trim adaptation on the raw, noisy rate would
// block it from ever running during exactly this condition. If this turns
// out to be wrong (trim adapting when it shouldn't), the fix is reverting
// this one line back to w_b, not the whole file.
//
// gKAdapt's default is also raised here, from tau_a=60s (2.922e-6) to
// **1e-4** -- hw-run-analysis.md 4.5 computed the adaptation's stability
// limit on the actual 15-state augmented model (9 base + 3 trim + 3
// filtered-rho-for-trim, matching this codebase's real architecture
// exactly): k_a < 0.00214 at tau_filt=5s (the existing gRhoLp time
// constant, unchanged -- the same analysis found the limit nearly
// independent of tau_filt above 2s but collapsing to 0.00043 at 1s, i.e.
// keep 5s). k_a=1e-4 sits at ~21x margin below that limit and converges
// in ~10s per that analysis (vs ~4min at the old default) -- still
// live-settable with "k<value>" if the bench disagrees. NOTE the source
// data: response was measured NON-monotonic (1e-4 converges faster than
// 1e-3), so don't assume "higher k_a = faster" and tune past this value
// by feel without re-measuring.
//
// UPDATE 2026-08-19: corner [-1,-1,-1]'s Kp/ell/theta_eq below are now the
// re-derived values for the new plant (mass 1.633 kg, plus a new corner-
// to-housing strut) -- user-supplied. The OTHER SEVEN corners are still
// the OLD (1.5668 kg, no strut) table from cubli_gains.h, same one hw-
// run-analysis.md's own 373.5s run used successfully -- so this table is
// a MIX of two plant generations, not internally consistent corner-to-
// corner. [-1,-1,-1]'s own gB (the corner-resolution direction) was NOT
// part of this update -- see that entry's own comment for why this is a
// low-risk gap, not the trim-clamp concern this comment used to flag.
// Also see that comment (and Cube-Performance-Envelope-Results.md) for
// the multi-corner finding: this pole leaves only [-1,-1,-1] and
// [+1,+1,+1] with positive recovery margin. Replace the remaining seven
// corners' Kp[3][9]/gB/ell/Sg/lambda/theta_eq once available -- see the
// TODO at the table below.
//
// ---------------------------- WHAT AND WHY ---------------------------------
// The manual tare worked, but it's a SNAPSHOT: whatever offset the cube
// happened to have at the moment you sent "z1" is what gets subtracted,
// forever, until you tare again by hand. It can't track drift (thermal
// effects on the IMU move the true offset during a run), and re-taring
// after every hardware change (battery mounted, a cable moved, a bolt
// tightened) is exactly the "gradient descent by eye" this note argues
// against.
//
// The fix uses a fact that's easy to miss: THE CUBE ALREADY MEASURES ITS
// OWN REFERENCE ERROR, continuously, for free. If the reference (gB) is
// wrong by an angle delta, the controller drives its MEASURED error to
// zero -- so the cube physically balances at a true tilt of delta. Gravity
// then exerts a constant torque proportional to delta, which the
// controller must constantly cancel. A constant torque would ramp a wheel
// to saturation, except the rho/K3 term (already in this file's full law)
// trades wheel speed against tilt instead, so the wheel settles at a fixed
// STANDING SPEED rather than accelerating forever:
//
//     standing_wheel_speed_ss = -(K1 * delta) / K3
//
// That standing speed is a signed, calibrated readout of delta. No new
// sensor needed -- just low-pass the wheel speed (this file already does,
// 5 s tau, for telemetry) and feed it back into the reference:
//
//     trim  +=  +k_a * dt * standing_wheel_speed          (PLUS, verified)
//
// As trim approaches delta, the residual tilt the controller sees shrinks,
// the standing speed shrinks, and the correction slows down and stops --
// a self-consistent fixed point, no external calibration step. Stage4_
// AutoTrim.ino's original default was tau_a = 60 s (k_a = 2.922e-6), about
// 200x slower than anything the control loop itself does so the two cannot
// interact or fight each other -- THIS FILE raises the default to k_a =
// 1e-4 (still ~21x below the stability limit re-derived for this file's
// architecture; see the "WHAT AND WHY" note above), converging in ~10s
// instead of ~4min.
//
// THIS IS NOT DISTURBANCE FEEDFORWARD -- that would estimate a torque and
// cancel it with more torque, which ramps the wheel to saturation forever
// because the WHEEL keeps supplying the cancelling torque. Trim instead
// MOVES THE REFERENCE so gravity supplies zero net torque at the new
// equilibrium -- bounded, and it drives its own error to zero.
//
// >>> THE SIGN IS VERIFIED, DO NOT RE-DERIVE. <<< A projected trim was
// injected on the bench and the wheel response measured: cos(trim, wheels)
// = -1.0000. The wheels spin OPPOSITE the trim. The opposite adaptation
// sign is UNSTABLE -- it runs away with time constant K3/(k_a*K1) in the
// wrong direction. Same class of "checked, not a guess" constant as this
// file's own kI_FILT gyro-bias integration a few sections down (also a
// verified PLUS).
//
// GUARDS, all of which matter (see updateTrim()/applyTrimGuards() below):
//   - Adapts ONLY while quiescent (tilt AND rate both small, "x1" enabled,
//     armed) -- during a recovery the wheel speeds are large and say
//     nothing about the equilibrium; adapting then would let one
//     disturbance corrupt the trim.
//   - Projected perpendicular to gB every update -- the component of trim
//     ALONG gB is yaw, which is unobservable and uncontrollable (Attitude
//     representation for the firmware.md S1); left alone it drifts without
//     bound instead of converging.
//   - Clamped to +/-2 deg -- an error beyond that is mechanical (shim it,
//     don't trim it). Hitting the clamp is itself a useful readout.
//   - Freezes automatically on any trip, because gArmed goes false the
//     same cycle a trip fires and updateTrim() is gated on gArmed -- "do
//     not learn from a fall" falls out for free, no extra state needed.
//   - Logged every cycle in telemetry, both in degrees and converted to an
//     equivalent COM offset in mm via this corner's ell -- per the note,
//     the converged value IS a live COM-error readout, not just an
//     internal correction term.
//
// NOT implemented here (deliberately, see the note's own scope): EEPROM
// persistence across boots ("save once converged"). "x0" freezes the
// current value for a session; there's no flash write yet, so trim starts
// at zero (or wherever "z1" seeds it) every power-up. Add EEPROM/LittleFS
// persistence separately once the trim's behavior on real hardware is
// trusted enough to want it surviving a reboot.
//
// "z1" still exists, repurposed as a FAST-START: it seeds trim so the
// corrected phi reads ~0 immediately, instead of waiting several tau_a for
// automatic adaptation to walk there from zero. The automatic trim then
// takes over from that seed and tracks any further drift on its own --
// "z1" is a shortcut to the same fixed point, not a competing mechanism.
// "z0" clears trim to zero. "x0"/"x1" freezes/resumes adaptation without
// touching the current value or disarming.
//
// STAGE 4 CHECKLIST — cube held by hand, gGainScale = 1.0:
//   Position near the resolved corner's equilibrium BEFORE sending "a1" --
//   if norm3(phi) doesn't settle under ARM_GATE (0.5 deg) even holding the
//   cube still at its natural rest point, either send "z1" once to fast-
//   start (recommended for this bring-up run), or arm anyway once close
//   enough and let automatic adaptation walk it in over the next ~4 time
//   constants (~10 s at this file's default k_a = 1e-4).
//   [ ] trim_x/y/z_deg in telemetry moves smoothly toward a steady value
//       while armed and quiescent, then stops changing -- that's
//       convergence. Note the value: per the guard above, it's your
//       equivalent COM-error readout (also printed as trim_com_mm).
//   [ ] All three wheels unwind after each correction (watch the standing-
//       speed low-pass columns -- rho through a ~5s low-pass) once trim has
//       converged -- persistent nonzero standing speed after convergence
//       means something is still wrong (a stale trim, a corner that got
//       re-resolved mid-run, or a genuinely large/mechanical offset pinned
//       at the 2 deg clamp).
//
// Velocity cap loosened (kMaxOmega, not literally removed) same as
// Stage4_FullLaw.ino -- see that file's header for the full reasoning.
// Trips here are still LOOSE (hand-held) -- Stage 5 tightens to the real
// DISARM/OMEGA_CAP policy from cubli_gains.h.
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>
#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// THIS RIG, confirmed on bench via Stage 1's corner-ID + pulse checklist
// (2026-08-21): id 1 -> X, id 3 -> Y, id 2 -> Z. NOT the same mapping as the
// old cube (Gam/Skeleton_3Axis.ino / edge-bringup had id 2 -> X, id 3 -> Y,
// id 1 -> Z) -- X and Z are swapped here. Corner balance needs all three
// wheels live at once, so all three are always instantiated.
Moteus moteusX(canBus, []() { Moteus::Options options; options.id = 1; return options; }());
Moteus moteusY(canBus, []() { Moteus::Options options; options.id = 3; return options; }());
Moteus moteusZ(canBus, []() { Moteus::Options options; options.id = 2; return options; }());

BMI270 imu;
// I2C, not SPI, on this rig: BMI2_I2C_PRIM_ADDR (0x68) is the SDO-low address
// -- if SDO is tied high on this breakout it's BMI2_I2C_SEC_ADDR (0x69)
// instead. 400 kHz is the BMI270's I2C fast-mode ceiling (Bosch datasheet);
// a 12-byte accel+gyro burst read at 400 kHz is ~340 us, comfortably inside
// this loop's 2 ms (500 Hz) period, so the interface change costs nothing
// timing-wise. Carried across from Stage 1.
const uint8_t  imuI2CAddress = BMI2_I2C_PRIM_ADDR;
const uint32_t imuI2CClockHz = 400000;
const uint32_t kPeriodMs = 2;   // 500 Hz

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from Stage 1-3)
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
// IMU calibration, THIS rig / THIS mount. Kept byte-identical across every
// stage in this folder (2026-08-21) -- these had drifted apart, with stages 3-5
// carrying a different set from stages 1-2.
// NOTE: kAccelScale's third entry (0.568) is a 43% correction. That is not what
// a healthy accelerometer calibration looks like, and it distorts ghat -- and
// therefore corner resolution and phi -- whenever gravity is not aligned with a
// single IMU axis. Re-run IMU_Calibration.ino for this mount before trusting
// phi numerically.
static const float kGyroBias[3]    = { -0.001956f, -0.009486f, -0.001770f };
static const float kAccelOffset[3] = { +0.008400f, -0.090030f, +0.001695f };  // m/s^2
static const float kAccelScale[3]  = { +0.819659f, +0.706744f, +0.568125f };

// Mount rotation, sensor -> body. The IMU has been MOVED ONTO THE CUBE'S
// BALANCING AXIS: its z axis now points along the body diagonal it balances
// on, not along a face normal. Sensor -> body is a THREE-step sequence:
//   1st: about +Y by  54.73561031718623 deg  (= acos(1/sqrt(3)), the angle
//        between a cube's body diagonal and a face normal -- the "magic
//        angle"; tilts the sensor z off the face normal onto the diagonal's
//        cone)
//   2nd: about  Z by -45 deg (swings it around that cone onto the diagonal)
//   3rd: about +Y by  90 deg (the sensor's clocking on the mount -- this one
//        is a quarter turn of the package itself, not part of the diagonal
//        geometry, so it lands AFTER the alignment above)
// All three are ACTIVE rotations about the FIXED body axes, applied in that
// order, so the matrix is the product written in REVERSE order:
//     C = Ry(thetaY2) * Rz(thetaZ) * Ry(thetaY1),   v_body = C * v_imu
// Composed factor-by-factor below rather than as one closed-form expression:
// the sequence has already changed twice, and a product of three named
// elementary matrices stays checkable by eye where nine expanded trig terms
// would not. (A Y rotation also has no direct slot in the old Z-X-Z Euler
// triple this replaced.)
// (0/90/-90 was this rig's PREVIOUS face-normal mount; -30/54.74/45 was the
// even older Skeleton_3Axis panel mount. Do not carry either over.)
float thetaY1_deg = 54.73561031718623f;   // applied FIRST,  about +Y
float thetaZ_deg  = -45.0f;               // applied SECOND, about  Z
float thetaY2_deg = 90.0f;                // applied THIRD,  about +Y
float gMountDCM[3][3];

// out = A * B. `out` must not alias A or B.
static void mat3mul(const float A[3][3], const float B[3][3], float out[3][3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
    }
  }
}

// Standard right-handed ACTIVE rotations (rotate the vector, axes fixed).
static void rotY(float deg, float R[3][3]) {
  const float c = cosf(deg * (float)DEG_TO_RAD), s = sinf(deg * (float)DEG_TO_RAD);
  R[0][0] =    c; R[0][1] = 0.0f; R[0][2] =    s;
  R[1][0] = 0.0f; R[1][1] = 1.0f; R[1][2] = 0.0f;
  R[2][0] =   -s; R[2][1] = 0.0f; R[2][2] =    c;
}

static void rotZ(float deg, float R[3][3]) {
  const float c = cosf(deg * (float)DEG_TO_RAD), s = sinf(deg * (float)DEG_TO_RAD);
  R[0][0] =    c; R[0][1] =   -s; R[0][2] = 0.0f;
  R[1][0] =    s; R[1][1] =    c; R[1][2] = 0.0f;
  R[2][0] = 0.0f; R[2][1] = 0.0f; R[2][2] = 1.0f;
}

void updateMountingDCM() {
  float Ry1[3][3], Rz[3][3], Ry2[3][3], RzRy1[3][3];
  rotY(thetaY1_deg, Ry1);
  rotZ(thetaZ_deg,  Rz);
  rotY(thetaY2_deg, Ry2);
  mat3mul(Rz,  Ry1,   RzRy1);        // Ry1 first, then Rz
  mat3mul(Ry2, RzRy1, gMountDCM);    // ... then Ry2
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
  // Expect, for this rig's Y(+54.7356) -> Z(-45) -> Y(+90) balancing-axis
  // mount:
  //   [[-0.8165,  0.0000,  0.5774],
  //    [-0.4082,  0.7071, -0.5774],
  //    [-0.4082, -0.7071, -0.5774]]
  // Eyeball this before trusting phi/om. The strongest single check is the
  // THIRD COLUMN -- that's where the IMU's +z axis lands in body coordinates,
  // and since +z is now the balancing axis it must come out a unit body
  // diagonal, (+1,-1,-1)/sqrt(3) = (0.5774, -0.5774, -0.5774). If it isn't a
  // permutation of +-0.5774 in all three slots, the mount angles are wrong.
  const float zb[3] = { C[0][2], C[1][2], C[2][2] };
  Serial.print("# IMU +z in body (must be a unit body diagonal, +-0.5774 x3): ");
  Serial.print(zb[0], 4); Serial.print(", ");
  Serial.print(zb[1], 4); Serial.print(", ");
  Serial.println(zb[2], 4);
  Serial.println("# mount DCM:");
  for (int i = 0; i < 3; ++i) {
    Serial.print("#   [");
    Serial.print(C[i][0], 4); Serial.print(", ");
    Serial.print(C[i][1], 4); Serial.print(", ");
    Serial.print(C[i][2], 4);
    Serial.println("]");
  }
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
// SECTION 2b-2: RATE LOW-PASS FILTER -- hw-run-analysis.md fix 4.1
// ----------------------------------------------------------------------------
// w_b (above) stays raw -- this is a SEPARATE signal, used only in
// commandWheels()'s om block. See the header note for the full reasoning
// on why only that one use point is filtered.

// 15-25 Hz per the source note; 20 Hz is the value its own phase-lag
// number (3.7 deg at 1.3 Hz) was computed for. Live-settable with
// "f<Hz>" to sweep the range on the bench without reflashing.
static float gRateFilterHz = 20.0f;
float w_filt[3] = { 0.0f, 0.0f, 0.0f };

void updateRateFilter(float dt) {
  const float tau = 1.0f / (2.0f * (float)PI * gRateFilterHz);
  const float alpha = dt / (tau + dt);
  for (int i = 0; i < 3; ++i) { w_filt[i] += alpha * (w_b[i] - w_filt[i]); }
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
// >>> UPDATED 2026-08-21 -- NEW COM (IMU moved onto the balancing axis). <<<
// What in this table is new, and what is NOT:
//   gB[3]          -- ALL EIGHT replaced with the newly measured corner
//                     directions for the new COM. Trustworthy.
//   Kp[3][9]       -- derived from ONE supplied reference 3x9 (the same-sign
//                     corner) by the exact sign rule Kp[i][blk*3+j] *= s_i*s_j.
//                     For [-1,-1,-1] and [+1,+1,+1] that factor is +1 for every
//                     pair, so those two carry the reference matrix VERBATIM.
//                     For the six MIXED corners the SIGNS are exact but the
//                     MAGNITUDES are inherited from the same-sign corner rather
//                     than derived for that corner's own lean/ell -- usable, not
//                     final. Replace per corner as real gains are computed.
//                     (Rule verified against the previous table: 144/144 sign
//                     agreement across the phi and om blocks; it mispredicts a
//                     few ~1e-3 rho diagonal terms, which are three orders of
//                     magnitude below the phi gains.)
//   placeOffsetDeg -- recomputed from the new gB (angle to the body diagonal).
//   ellM, and the per-corner prose comments below (ell / Sg / lambda /
//                     theta_eq / recovery margins) -- STALE, still the OLD
//                     plant. Do not trust those numbers until re-derived.
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
    , { -0.583713f, -0.584069f, -0.564042f }
    , { { -4.835f, 2.4877f, 2.4276f, -0.6198f, 0.3174f, 0.3096f, -0.0009f, 0.0021f, 0.0021f },  // wheel X
        { 2.4904f, -4.8347f, 2.4291f, 0.3174f, -0.6202f, 0.3095f, 0.0021f, -0.0009f, 0.0021f },  // wheel Y
        { 2.4045f, 2.4033f, -4.977f, 0.31f, 0.3098f, -0.6346f, 0.0023f, 0.0023f, -0.0008f } } // wheel Z
    , 0.928724393f, 0.12108f },
  { "[-1,-1,+1]"  // lean 3.170 deg vs body diagonal, ell 128.54 mm, Sg 1.9750, lambda 8.1212
    , { -0.537196f, -0.537524f, 0.649991f }
    , { { -4.835f, 2.4877f, -2.4276f, -0.6198f, 0.3174f, -0.3096f, -0.0009f, 0.0021f, -0.0021f },  // wheel X
        { 2.4904f, -4.8347f, -2.4291f, 0.3174f, -0.6202f, -0.3095f, 0.0021f, -0.0009f, -0.0021f },  // wheel Y
        { -2.4045f, -2.4033f, -4.977f, -0.31f, -0.3098f, -0.6346f, -0.0023f, -0.0023f, -0.0008f } } // wheel Z
    , 5.27655426f, 0.128537506f },
  { "[-1,+1,-1]"  // lean 2.773 deg vs body diagonal, ell 126.08 mm, Sg 1.9373, lambda 8.1591
    , { -0.549159f, 0.645625f, -0.530653f }
    , { { -4.835f, -2.4877f, 2.4276f, -0.6198f, -0.3174f, 0.3096f, -0.0009f, -0.0021f, 0.0021f },  // wheel X
        { -2.4904f, -4.8347f, -2.4291f, -0.3174f, -0.6202f, -0.3095f, -0.0021f, -0.0009f, -0.0021f },  // wheel Y
        { 2.4045f, -2.4033f, -4.977f, 0.31f, -0.3098f, -0.6346f, 0.0023f, -0.0023f, -0.0008f } } // wheel Z
    , 5.00861047f, 0.126083225f },
  { "[-1,+1,+1]"  // lean 3.097 deg vs body diagonal, ell 131.64 mm, Sg 2.0226, lambda 8.0480
    , { -0.509899f, 0.599468f, 0.616962f }
    , { { -4.835f, -2.4877f, -2.4276f, -0.6198f, -0.3174f, -0.3096f, -0.0009f, -0.0021f, -0.0021f },  // wheel X
        { -2.4904f, -4.8347f, 2.4291f, -0.3174f, -0.6202f, 0.3095f, -0.0021f, -0.0009f, 0.0021f },  // wheel Y
        { -2.4045f, 2.4033f, -4.977f, -0.31f, 0.3098f, -0.6346f, -0.0023f, 0.0023f, -0.0008f } } // wheel Z
    , 4.65881833f, 0.131639361f },
  { "[+1,-1,-1]"  // lean 3.171 deg vs body diagonal, ell 128.56 mm, Sg 1.9753, lambda 8.1180
    , { 0.645701f, -0.549274f, -0.530441f }
    , { { -4.835f, -2.4877f, -2.4276f, -0.6198f, -0.3174f, -0.3096f, -0.0009f, -0.0021f, -0.0021f },  // wheel X
        { -2.4904f, -4.8347f, 2.4291f, -0.3174f, -0.6202f, 0.3095f, -0.0021f, -0.0009f, 0.0021f },  // wheel Y
        { -2.4045f, 2.4033f, -4.977f, -0.31f, 0.3098f, -0.6346f, -0.0023f, 0.0023f, -0.0008f } } // wheel Z
    , 5.01640064f, 0.128557414f },
  { "[+1,-1,+1]"  // lean 2.609 deg vs body diagonal, ell 134.01 mm, Sg 2.0591, lambda 7.9804
    , { 0.599572f, -0.510034f, 0.616749f }
    , { { -4.835f, -2.4877f, 2.4276f, -0.6198f, -0.3174f, 0.3096f, -0.0009f, -0.0021f, 0.0021f },  // wheel X
        { -2.4904f, -4.8347f, -2.4291f, -0.3174f, -0.6202f, -0.3095f, -0.0021f, -0.0009f, -0.0021f },  // wheel Y
        { 2.4045f, -2.4033f, -4.977f, 0.31f, -0.3098f, -0.6346f, 0.0023f, -0.0023f, -0.0008f } } // wheel Z
    , 4.64807997f, 0.134011015f },
  { "[+1,+1,-1]"  // lean 3.095 deg vs body diagonal, ell 131.66 mm, Sg 2.0229, lambda 8.0501
    , { 0.611553f, 0.611236f, -0.502388f }
    , { { -4.835f, 2.4877f, -2.4276f, -0.6198f, 0.3174f, -0.3096f, -0.0009f, 0.0021f, -0.0021f },  // wheel X
        { 2.4904f, -4.8347f, -2.4291f, 0.3174f, -0.6202f, -0.3095f, 0.0021f, -0.0009f, -0.0021f },  // wheel Y
        { -2.4045f, -2.4033f, -4.977f, -0.31f, -0.3098f, -0.6346f, -0.0023f, -0.0023f, -0.0008f } } // wheel Z
    , 5.10629368f, 0.131658807f },
  { "[+1,+1,+1]"  // lean 0.714 deg vs body diagonal, ell 136.99 mm, Sg 2.1048, lambda 7.9341
    , { 0.571935f, 0.571638f, 0.58832f }
    , { { -4.835f, 2.4877f, 2.4276f, -0.6198f, 0.3174f, 0.3096f, -0.0009f, 0.0021f, 0.0021f },  // wheel X
        { 2.4904f, -4.8347f, 2.4291f, 0.3174f, -0.6202f, 0.3095f, 0.0021f, -0.0009f, 0.0021f },  // wheel Y
        { 2.4045f, 2.4033f, -4.977f, 0.31f, 0.3098f, -0.6346f, 0.0023f, 0.0023f, -0.0008f } } // wheel Z
    , 0.77358409f, 0.136988997f },
};

int gCornerIdx = 0;

// ---------------------- ACTIVE EQUILIBRIUM (gB) -----------------------------
// gB is the body-frame gravity direction at balance -- i.e. THE equilibrium.
// kCorners[].gB holds the CAD/table value; gActiveGB is what the controller
// actually uses, so a measured equilibrium can replace the tabulated one.
//
// Why this exists: gTrim can only nudge phi, and applyTrimGuards() clamps it to
// gTrimMax. Once gTrim saturates, re-seeding it is a FIXED POINT -- the clamp
// rescales the new value straight back to the same vector and nothing changes.
// Capturing gB itself has no such bound and sets phi to exactly zero.
static float gActiveGB[3]  = { 0.0f, 0.0f, 1.0f };
static bool  gGBFromBench  = false;   // true once z1 has overridden the table

static void setActiveGBFromTable() {
  const float* t = kCorners[gCornerIdx].gB;
  gActiveGB[0] = t[0]; gActiveGB[1] = t[1]; gActiveGB[2] = t[2];
  normalize3(gActiveGB);
  gGBFromBench = false;
}

// Capture the current attitude as the equilibrium: gB := ghat. phi is then
// -(gB x ghat) = 0 exactly, by construction, at this pose.
static void captureEquilibrium() {
  const float* tab = kCorners[gCornerIdx].gB;
  float old[3] = { gActiveGB[0], gActiveGB[1], gActiveGB[2] };

  gActiveGB[0] = ghat[0]; gActiveGB[1] = ghat[1]; gActiveGB[2] = ghat[2];
  normalize3(gActiveGB);
  gGBFromBench = true;

  // The trim exists to correct a gB that is slightly wrong. gB is now measured,
  // so any accumulated trim is stale -- keep it and it would be applied twice.
  gTrim[0] = gTrim[1] = gTrim[2] = 0.0f;

  const float dMoved = dot3(old, gActiveGB);
  const float dTable = dot3(tab, gActiveGB);
  Serial.print("# EQUILIBRIUM CAPTURED: gB = ");
  Serial.print(gActiveGB[0], 6); Serial.print(", ");
  Serial.print(gActiveGB[1], 6); Serial.print(", ");
  Serial.print(gActiveGB[2], 6); Serial.println("  (gTrim reset to 0)");
  Serial.print("#   moved ");
  Serial.print(acosf(dMoved > 1.0f ? 1.0f : (dMoved < -1.0f ? -1.0f : dMoved)) * (float)RAD_TO_DEG, 3);
  Serial.print(" deg from the previous equilibrium, ");
  Serial.print(acosf(dTable > 1.0f ? 1.0f : (dTable < -1.0f ? -1.0f : dTable)) * (float)RAD_TO_DEG, 3);
  Serial.println(" deg from the CAD table value.");
  Serial.println("#   phi now reads ~0 at this pose. NOTE Kp was derived for the");
  Serial.println("#   TABLE geometry -- a large move here means the gains are");
  Serial.println("#   matched to a corner the cube no longer has. Re-derive them");
  Serial.println("#   if this angle is big. Send c to go back to the table value.");
}


void resolveCornerCandidate() {
  int bestIdx = 0, secondIdx = 0;
  float bestDot = -2.0f, secondDot = -2.0f;
  for (int i = 0; i < 8; ++i) {
    const float d = dot3(ghat, kCorners[i].gB);
    if (d > bestDot) { secondDot = bestDot; secondIdx = bestIdx; bestDot = d; bestIdx = i; }
    else if (d > secondDot) { secondDot = d; secondIdx = i; }
  }
  gCornerIdx = bestIdx;
  setActiveGBFromTable();
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

// AUTOMATIC TRIM state -- see the header note above for the full
// derivation. ADDED to the raw measurement below (note the sign: this is
// deliberately the opposite convention from Stage4_FullLaw.ino's
// gPhiOffset, which was SUBTRACTED -- gTrim follows "Automatic Trim...md"
// section 4's own phi_err = raw + trim verbatim, because the adaptation
// law's sign is the one thing in that note explicitly flagged as
// VERIFIED, DO NOT RE-DERIVE, and re-deriving what the equivalent
// subtraction-convention sign would be is exactly the kind of "should be
// obviously equivalent" reasoning that gets signs wrong in this codebase.
// Follow the source exactly rather than reconciling conventions by hand.
static float gTrim[3]     = { 0.0f, 0.0f, 0.0f };   // rad, added to raw phi
static bool  gTrimEnabled = true;                    // "x0" freezes, "x1" resumes

void updateCornerProjection() {
  float t[3];
  cross3(gActiveGB, ghat, t);
  phi[0] = -t[0] + gTrim[0];
  phi[1] = -t[1] + gTrim[1];
  phi[2] = -t[2] + gTrim[2];
}

// Strip the yaw component (unobservable/uncontrollable, Attitude
// representation for the firmware.md S1) and clamp to TRIM_MAX. Applied
// after BOTH the automatic adaptation step and a manual "z1" seed, so
// neither path can leave gTrim carrying a yaw component or an
// unreasonably large magnitude.
static float gTrimMax = 0.0349065850f;   // rad, 2 deg  // live, see "t<deg>"

void applyTrimGuards() {
  const float* gB = gActiveGB;
  const float along = dot3(gTrim, gB);
  gTrim[0] -= along * gB[0];
  gTrim[1] -= along * gB[1];
  gTrim[2] -= along * gB[2];

  const float n = norm3(gTrim);
  if (n > gTrimMax) {
    const float s = gTrimMax / n;
    gTrim[0] *= s; gTrim[1] *= s; gTrim[2] *= s;
  }
}

// Seed the trim so corrected phi reads ~0 right now, and REPORT HONESTLY
// whether the clamp swallowed it.
//
// The old inline version tested norm3(gTrim) > 10 deg AFTER applyTrimGuards()
// had already clamped to gTrimMax (2 deg) -- a post-clamp value can never
// exceed a threshold five times the clamp, so that warning was unreachable and
// a clamped seed looked identical to a successful one.
//
// Why it matters: arming requires |phi| < kArmGate (0.5 deg). If the seed the
// cube actually needs is larger than gTrimMax, the clamp caps it, phi never
// gets under the gate, "a1" keeps refusing, and NO torque is ever commanded --
// which looks exactly like the firmware ignoring your commands.
void seedTrimFromPhi() {
  const float want[3] = {
    gTrim[0] - phi[0],
    gTrim[1] - phi[1],
    gTrim[2] - phi[2],
  };
  const float wantMag = norm3(want);

  gTrim[0] = want[0]; gTrim[1] = want[1]; gTrim[2] = want[2];
  applyTrimGuards();

  Serial.print("# gTrim SEEDED to: ");
  Serial.print(gTrim[0] * (float)RAD_TO_DEG, 3); Serial.print(",");
  Serial.print(gTrim[1] * (float)RAD_TO_DEG, 3); Serial.print(",");
  Serial.print(gTrim[2] * (float)RAD_TO_DEG, 3); Serial.print(" deg  (|trim|=");
  Serial.print(norm3(gTrim) * (float)RAD_TO_DEG, 3); Serial.println(" deg)");

  if (wantMag > gTrimMax) {
    Serial.print("# >>> CLAMPED: the seed needed ");
    Serial.print(wantMag * (float)RAD_TO_DEG, 3);
    Serial.print(" deg but gTrimMax is ");
    Serial.print(gTrimMax * (float)RAD_TO_DEG, 3);
    Serial.println(" deg. <<<");
    Serial.println("#   phi will NOT reach the 0.5 deg arm gate, so a1 will keep");
    Serial.println("#   refusing and no torque will fire. Either shim the cube");
    Serial.println("#   mechanically, or raise the clamp with t<deg> if you accept");
    Serial.println("#   that a trim this large is a real COM offset (expected after");
    Serial.println("#   moving the IMU onto the balancing axis) and not a bad pose.");
  }
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------

static void printPhiOffset(const char* prefix) {
  Serial.print(prefix);
  Serial.print(gTrim[0] * (float)RAD_TO_DEG, 4); Serial.print(", ");
  Serial.print(gTrim[1] * (float)RAD_TO_DEG, 4); Serial.print(", ");
  Serial.print(gTrim[2] * (float)RAD_TO_DEG, 4);
  Serial.println(" deg");
}

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
  // Automatic trim readout -- per-axis in degrees, plus the equivalent COM
  // offset in mm (norm3(gTrim) * ell, small-angle) per Automatic Trim.md
  // S4's "log trim continuously" guard: this is a live COM-error readout,
  // not just an internal correction term, so it's worth watching converge.
  Serial.print('\t'); Serial.print(gTrim[0] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(gTrim[1] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(gTrim[2] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(norm3(gTrim) * kCorners[gCornerIdx].ellM * 1000.0f, 3);
  Serial.print('\t'); Serial.print(gTrimEnabled ? 1 : 0);
  // Filtered rate readout -- compare against om_x/y/z_dps above to see the
  // fix working directly: hw-run-analysis.md's 35 Hz mode should show up
  // as a much bigger raw-vs-filtered gap than ordinary sensor noise would.
  Serial.print('\t'); Serial.print(w_filt[0] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(w_filt[1] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.println(w_filt[2] * (float)RAD_TO_DEG, 2);
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

// Arm gate (cubli_gains.h's contract): refuse "a1" unless already near the
// resolved corner's equilibrium. Prevents an accidental full-authority
// command from a badly-off-vertical starting position -- checked at the
// moment of arming, not continuously (a trip mid-run is the DISARM trips'
// job, not this one). Compared against norm3(phi), the 3-vector analogue
// of edge-bringup's |phi_edge|.
static const float kArmGate = 0.00872664619f;   // rad, 0.5 deg -- kept at the
                                                  // real value; "z1" (tare)
                                                  // is the fix for a COM
                                                  // offset, not a wider gate.

// >>> WHEEL SIGN: ALL THREE +1.0f. Do not change without redoing the
// >>> eigenvalue check described below. <<<
//
// Established 2026-08-21 by closed-loop analysis, after all -1.0f and then
// (+1,-1,-1) both produced a runaway at Stage 2 on the bench.
//
// The loop is  w_dot ~ diag(s) * Kp_om * w  (the firmware commands wheel i
// with s_i * u_i, and the body feels the reaction). Sweeping all eight sign
// combinations against this corner table gives exactly ONE stable answer:
//   s = (+1,+1,+1)  ->  max Re = -0.0002   STABLE
//   s = (+1,-1,-1)  ->  max Re = +0.9371   runaway, ~1 s time constant
//   s = (-1,-1,-1)  ->  max Re = +0.9374   runaway
// Every mixed combination is unstable too: Kp is a COUPLED 3x3, so negating
// one wheel's actuation is not a local change -- it breaks the whole design.
//
// WHY STAGE 1'S PULSE CHECK CANNOT SETTLE THIS: it commands s*tau and reports
// rho = s*v ~ k*s*s*tau = k*tau. s*s is +1 for BOTH signs, so "rho goes
// positive for a positive pulse" passes no matter what is in this array. That
// checklist item is blind here; the only Stage 1 check that discriminates is
// the physical one -- does the cube push toward DECREASING |phi|.
//
// Fix wiring-convention sign problems HERE, NEVER by flipping a sign inside
// Kp -- Kp comes verbatim from the corner table.
static const float kAxisWheelSign[3] = {
  -1.0f,   // X (this rig, moteus id 1)
  -1.0f,   // Y (this rig, moteus id 3)
  -1.0f,   // Z (this rig, moteus id 2)
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

// ---------------------------- YAW PROJECTION --------------------------------
// Rotation about gB (the balancing axis) is the corner controller's null mode:
// the reduced-attitude estimator cannot OBSERVE it -- phi = -(gB x ghat) is
// perpendicular to gB by construction, so the phi block already spans only two
// dimensions and the state is effectively 8, not 9 -- and reaction wheels
// cannot hold sustained torque about it anyway.
//
// The om block is supposed to inherit that null. At the corner the reference
// gains were derived for it very nearly does; at the six MIXED corners it does
// NOT, because the Kp magnitudes are inherited from a same-sign corner (see the
// table header). Measured null-vs-gB misalignment, and the residual yaw torque
// it leaves as a fraction of a full Kp row:
//   [-1,-1,-1] 0.3 deg off,  0.7%      <- a derived corner, fine
//   [+1,+1,+1] 1.4 deg off,  3.0%      <- a derived corner, fine
//   the six MIXED corners: 4.5-5.9 deg off, 9.6-12.6%
// That residual PUSHES on yaw instead of ignoring it.
//
// Both phi and om are projected onto the plane perpendicular to gB before Kp.
// For a raw phi that is a no-op (it is already perpendicular); what it removes
// is any gB component carried by the OFFSET/TRIM added to phi, which would
// otherwise inject a tilt the estimator can never produce or correct.
// Toggle with "y0"/"y1" to A/B it on the bench.
static bool gYawProject = true;

static void projectOutYaw(const float in[3], float out[3]) {
  const float* gB = gActiveGB;
  const float n2 = dot3(gB, gB);
  if (!gYawProject || n2 < 1e-12f) {
    out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
    return;
  }
  const float k = dot3(in, gB) / n2;
  out[0] = in[0] - k * gB[0];
  out[1] = in[1] - k * gB[1];
  out[2] = in[2] - k * gB[2];
}

void commandWheels(const float rho[3]) {
  // x = [phi(3); om(3); rho(3)] -- full state, all nine columns of each
  // wheel's Kp row now contribute. om uses w_filt, NOT raw w_b -- this is
  // the one line hw-run-analysis.md's fix 4.1 changes. Everything else in
  // the law (Kp itself, phi, rho) is untouched.
  // phi and om are yaw-projected before Kp (see projectOutYaw above);
  // telemetry still reports the unprojected values.
  float phiUsed[3], omUsed[3];
  projectOutYaw(phi, phiUsed);
  projectOutYaw(w_filt, omUsed);
  const float xVec[9] = {
    phiUsed[0], phiUsed[1], phiUsed[2],
    omUsed[0], omUsed[1], omUsed[2],
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


// ----------------------------------------------------------------------------
// SECTION 2e-2: AUTOMATIC TRIM ADAPTATION
// ----------------------------------------------------------------------------
// Call once per cycle AFTER commandWheels() has updated gRhoLp for this
// cycle -- gRhoLp (5 s low-pass of wheel speed) IS the "standing wheel
// speed" the header note's derivation is built on; no separate filter is
// kept here, reusing the one this file already computes for telemetry.

// hw-run-analysis.md 4.5: stability limit k_a < 0.00214 (15-state
// augmented model, tau_filt=5s -- see header note). 1e-4 sits at ~21x
// margin and converges in ~10s per that analysis, replacing the original
// tau_a=60s (2.922e-6) default. Live-settable with "k<value>" --
// NON-MONOTONIC per the source measurement (1e-4 converges faster than
// 1e-3), so re-measure before assuming "higher = faster" past this point.
static float gKAdapt = 1.0e-4f;

static const float kTiltQuiet  = 0.00872664619f;   // rad, 0.5 deg -- same
                                                      // value as kArmGate,
                                                      // not a coincidence:
                                                      // both mean "close
                                                      // enough to trust".
static const float kOmegaQuiet = 0.10f;              // rad/s

void updateTrim(float dt) {
  // Freezes on disarm/any trip for free: gArmed goes false the same cycle
  // a trip fires (see the checks above in commandWheels()), and this is
  // gated on it -- "do not learn from a fall" without extra state.
  if (!gTrimEnabled || !gArmed) { return; }
  if (norm3(phi) >= kTiltQuiet)  { return; }   // recovering, not quiescent
  // w_filt, not raw w_b -- see header note. hw-run-analysis.md's own data
  // showed phi clean (genuinely quiescent) while raw om was dominated by
  // the 35 Hz mode; gating on the raw signal would block adaptation
  // during exactly the condition this fix targets.
  if (norm3(w_filt) >= kOmegaQuiet) { return; }   // still moving, not quiescent

  for (int i = 0; i < 3; ++i) {
    gTrim[i] += gKAdapt * dt * gRhoLp[i];   // PLUS -- VERIFIED, see header
  }
  applyTrimGuards();
}


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
    if (val != 0.0f) {
      if (norm3(phi) < kArmGate) {
        gArmed = true;
        Serial.println("# gArmed = TRUE");
      } else {
        gArmed = false;
        Serial.print("# ARM REFUSED: |phi|="); Serial.print(norm3(phi) * (float)RAD_TO_DEG, 3);
        Serial.print(" deg exceeds ARM_GATE="); Serial.print(kArmGate * (float)RAD_TO_DEG, 2);
        Serial.println(" deg. Get closer to the resolved equilibrium and retry.");
      }
    } else {
      gArmed = false;
      Serial.println("# gArmed = FALSE");
    }
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
    Serial.print("# gGainScale = "); Serial.println(gGainScale, 3);
  } else if (cmd == 'c') {
    resolveCornerCandidate();
  } else if (cmd == 'z') {
    if (val != 0.0f) {
      captureEquilibrium();
    } else {
      gTrim[0] = gTrim[1] = gTrim[2] = 0.0f;
      setActiveGBFromTable();
      Serial.println("# gTrim cleared to 0,0,0 and gB restored to the table value");
    }
  } else if (cmd == 'x') {
    gTrimEnabled = (val != 0.0f);
    Serial.print("# gTrimEnabled = ");
    Serial.println(gTrimEnabled ? "TRUE (adapting)" : "FALSE (frozen at current value)");
  } else if (cmd == 'k') {
    gKAdapt = val;
    Serial.print("# gKAdapt = "); Serial.println(gKAdapt, 8);
    Serial.println("#   (note's table: 20s->8.765e-6 30s->5.843e-6 "
                    "60s->2.922e-6 120s->1.461e-6)");
  } else if (cmd == 'f') {
    // hw-run-analysis.md 4.1's range is 15-25 Hz -- sweep it live rather
    // than reflashing per step. Refuses <=0 (would divide by zero / give
    // a negative tau) instead of silently producing garbage.
    if (val > 0.0f) {
      gRateFilterHz = val;
      Serial.print("# gRateFilterHz = "); Serial.print(gRateFilterHz, 1);
      Serial.println(" Hz");
    } else {
      Serial.println("# REFUSED: f<Hz> needs a positive value (try 15-25).");
    }
  } else if (cmd == 'y') {
    gYawProject = (val != 0.0f);
    Serial.print("# gYawProject = ");
    Serial.println(gYawProject ? "TRUE (phi and om projected onto the plane perp to gB)"
                                : "FALSE (raw phi/om into Kp -- yaw leaks ~10% of a row here)");
  } else if (cmd == 'o') {
    // o            -- report the current offset
    // oc           -- CAPTURE: make corrected phi read ~0 right now
    // oz           -- zero it
    // o<x> <y> <z> -- set gTrim explicitly, in DEGREES (comma or space
    //                 separated). NOTE this file stores the offset with the
    //                 '+' convention (phi = raw + gTrim), so an explicit
    //                 set is in THAT frame; oc/oz/report mean the same thing
    //                 in every stage regardless.
    String arg = line.substring(1);
    arg.trim();
    if (arg.length() == 0) {
      printPhiOffset("# offset = ");
    } else if (arg.charAt(0) == 'c') {
      seedTrimFromPhi();
      printPhiOffset("# offset CAPTURED from current attitude = ");
      Serial.println("# hold the cube where it actually balances before capturing");
    } else if (arg.charAt(0) == 'z') {
      gTrim[0] = gTrim[1] = gTrim[2] = 0.0f;
      printPhiOffset("# offset ZEROED = ");
    } else {
      arg.replace(',', ' ');
      float v[3] = { 0.0f, 0.0f, 0.0f };
      int n = 0, from = 0;
      while (n < 3 && from < (int)arg.length()) {
        while (from < (int)arg.length() && arg.charAt(from) == ' ') { from++; }
        if (from >= (int)arg.length()) { break; }
        int to = arg.indexOf(' ', from);
        if (to < 0) { to = arg.length(); }
        v[n++] = arg.substring(from, to).toFloat() * (float)DEG_TO_RAD;
        from = to;
      }
      if (n != 3) {
        Serial.println("# need 3 values in deg, e.g. o0.42 -0.15 0.08  (or oc / oz)");
      } else {
        gTrim[0] = v[0]; gTrim[1] = v[1]; gTrim[2] = v[2];
        applyTrimGuards();
        printPhiOffset("# offset = ");
      }
    }
  } else if (cmd == 't') {
    gTrimMax = fabsf(val) * (float)DEG_TO_RAD;
    applyTrimGuards();
    Serial.print("# gTrimMax = "); Serial.print(gTrimMax * (float)RAD_TO_DEG, 3);
    Serial.println(" deg (trim re-clamped to the new limit)");
  } else if (cmd == 'b') {
    // Re-run the gyro bias calibration ON DEMAND. The boot run only captures
    // the bias at that instant; it drifts thermally, and the component ALONG
    // GRAVITY is invisible to the complementary filter (its innovation
    // e = ghat x ga has no component along ghat), so nothing downstream can
    // ever remove it. A fresh static calibration is the only thing that can.
    // Measured on this rig: 6.2 dps residual, 6.16 of it gravity-aligned.
    gArmed = false;
    Serial.println("# Re-calibrating gyro bias -- hold the cube PERFECTLY STILL (~2 s)");
    calibrateGyroBias((val > 0.0f) ? (uint16_t)val : 1000);
    // bhat was estimated against the OLD bias; keeping it double-corrects.
    bhat[0] = bhat[1] = bhat[2] = 0.0f;
    Serial.print("# gyro bias (body, rad/s): ");
    Serial.print(gGyroBiasBody[0], 6); Serial.print('	');
    Serial.print(gGyroBiasBody[1], 6); Serial.print('	');
    Serial.println(gGyroBiasBody[2], 6);
    Serial.println("# om_* should now read ~0 while static. If it does not,");
    Serial.println("#   the cube moved during the 2 s window -- redo it.");
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
    Serial.println("# unknown. use: a<0/1>  g<0..1>  c (re-resolve)  "
                    "z<0/1> (clear/seed trim)  x<0/1> (freeze/resume "
                    "adaptation)  k<value> (set gKAdapt)  f<Hz> (rate "
                    "filter corner freq)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 4: FULL LAW + AUTO TRIM + RATE FILTER (cube held by hand)");

  // RETRY, don't spin on a stale value: errorCode used to be `const` and was
  // evaluated once, so a failed CAN init became an un-exitable loop that
  // printed forever and never re-attempted -- setup() never finished and the
  // serial command handler in loop() was therefore never reached.
  uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.print(errorCode, HEX);
    Serial.println(" -- retrying in 1 s (check CAN3 wiring / termination)");
    delay(1000);
    errorCode = ACAN_T4::can3.beginFD(canSettings);
  }

  moteusX.SetStop();
  moteusY.SetStop();
  moteusZ.SetStop();
  Serial.println("all stopped");

  updateMountingDCM();
  checkMountingDCMValid();

  Wire.begin();
  Wire.setClock(imuI2CClockHz);

  while (imu.beginI2C(imuI2CAddress, Wire) != BMI2_OK) {
    Serial.println("Error: BMI270 not connected, check wiring and I2C address!");
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

  Serial.println("t_ms\tphi_x_deg\tphi_y_deg\tphi_z_deg\t"
                  "om_x_dps\tom_y_dps\tom_z_dps\trho_x\trho_y\trho_z\t"
                  "rho_x_lp\trho_y_lp\trho_z_lp\t"
                  "tau_x\ttau_y\ttau_z\ttau_cmd_x\ttau_cmd_y\ttau_cmd_z\t"
                  "armed\tgain_scale\t"
                  "trim_x_deg\ttrim_y_deg\ttrim_z_deg\ttrim_com_mm\ttrim_enabled\t"
                  "om_x_filt_dps\tom_y_filt_dps\tom_z_filt_dps");
  Serial.println("# STARTS DISARMED. a1 refused unless norm3(phi) < ARM_GATE");
  Serial.println("# (0.5 deg) -- get close to the resolved corner first, or send");
  Serial.println("# z1 to fast-start (seeds trim so corrected phi reads ~0 now).");
  Serial.println("# Trim then adapts AUTOMATICALLY while armed+quiescent -- watch");
  Serial.println("# trim_x/y/z_deg converge in telemetry, no further z1 needed.");
  Serial.println("# z0 clears trim, x0/x1 freezes/resumes adaptation without");
  Serial.println("# disarming, k<value> sets the adaptation gain (default k_a=1e-4).");
  Serial.print("# Rate low-pass at "); Serial.print(gRateFilterHz, 1);
  Serial.println(" Hz feeds the control law (om_x/y/z_filt_dps in");
  Serial.println("# telemetry) -- f<Hz> to sweep 15-25 Hz live, see hw-run-analysis.md.");
  Serial.println("# Velocity cap is loosened this stage (kMaxOmega ~2000 RPM, not");
  Serial.println("# the 40 rad/s policy value) -- a0 (disarm) is the real safety net.");
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
  updateRateFilter(dt);   // before commandWheels() -- this cycle's w_filt
                           // must be fresh, unlike gTrim's one-cycle lag

  const float rho[3] = {
    kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI,
  };

  commandWheels(rho);
  updateTrim(dt);   // after commandWheels() so this cycle's gRhoLp is fresh

  printState(time, rho, gRhoLp, gLastTau, gLastTauCmd, gArmed, gGainScale);
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// This file vs Stage4_AutoTrim.ino: identical control law, identical corner
// candidate table, identical trim mechanism (guards, sign, clamp) -- the
// ONLY behavioral differences are (1) w_filt replacing w_b in commandWheels()'s
// om block and in updateTrim()'s quiescence gate, per hw-run-analysis.md's
// fix 4.1, and (2) gKAdapt's default raised from 2.922e-6 to 1e-4 per fix
// 4.5. If something looks wrong here that isn't about the 35 Hz mode or
// trim-adaptation speed specifically, it's worth checking whether
// Stage4_AutoTrim.ino has the same problem before assuming this file's
// changes caused it.
//
// Next steps once this is trusted on real hardware:
//   - Port the SAME rate-filter + k_a replacement into
//     Stage5_AutoTrim_RateFilter.ino -- done, same mechanism (Stage 5
//     additionally has the real DISARM/OMEGA_CAP trip policy and
//     gMaxOmega/gTaperStart; w_filt is kept OUT of those trip checks,
//     same as it's kept out of this file's isfinite()/arm-gate checks).
//   - Drop in the real Kp[3][9]/gB/ell/Sg/lambda/theta_eq for the
//     remaining SEVEN corners once available for the new plant (mass
//     1.633 kg + strut) -- corner [-1,-1,-1]'s Kp is done (2026-08-19),
//     see the TODO above kCorners. Fix 4.2 ("reduce qr") needs the
//     re-derived gain matrix itself, not something this file can compute.
//   - EEPROM/LittleFS persistence -- save gTrim once converged (see the
//     header note's guard table), load it back at boot instead of
//     starting at zero every power-up.
//   - The 180-deg flip test (Automatic Trim.md S5): let trim converge,
//     rotate the cube 180 deg about the vertical on the SAME corner, let
//     it converge again. A COM offset reverses sign; an IMU mounting
//     misalignment does not -- this is the only way to tell the two
//     apart, and it matters once multiple corners are being trimmed
//     (a mounting misalignment is the same on all eight, a COM offset
//     changes direction per corner).
// ============================================================================
