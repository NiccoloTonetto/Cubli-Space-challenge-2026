// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 5: RELEASE + AUTOMATIC TRIM + FINE-TUNING INSTRUMENTATION
// ============================================================================
// Copy of Stage5_Release.ino carrying the same three additions
// Stage4_AutoTrim.ino made to Stage4_FullLaw.ino, PLUS what Cube
// Fine-Tuning -- Test Plan.md's Session 2/3 (Tests 5, 6, 7) need that
// Stage 4 didn't: a live-settable wheel-speed cap and endurance-run
// instrumentation. If this is the first AutoTrim variant you're looking
// at, read Stage4_AutoTrim.ino's header first -- the trim mechanism itself
// isn't re-explained in full here.
//
// PHYSICAL STOP/CATCH IN PLACE. E-STOP IN HAND. This is the stage where the
// cube is NOT held by hand. The control law is identical to Stage 4 (this
// one, with trim); nothing new about the law itself, only about the full
// closed loop running unsupported, on a corner, for real.
//
// ---------------------------- WHAT'S NEW HERE ---------------------------
// vs Stage5_Release.ino:
//   - Automatic trim (gTrim), replacing nothing -- Stage5_Release never had
//     a manual offset to replace (edge-bringup's kPhiOffset/corner's old
//     Stage4 gPhiOffset were never ported to Stage 5). This is trim's
//     first appearance at the release stage. "z1"/"z0" seed/clear,
//     "x1"/"x0" freeze/resume, "k<value>" adaptation gain -- same as
//     Stage4_AutoTrim.
//   - Test 6 (wheel-speed cap): kMaxOmega/kTaperStart are now LIVE-
//     SETTABLE ("o<rad/s>"/"p<rad/s>"), not compiled-in constants. The
//     test's own procedure is iterative -- set omega_cap to ~3x the
//     post-trim standing speed, verify, tighten toward 40 rad/s in
//     steps -- and that is painful if every step needs a recompile+
//     reflash. Real policy value (40) is still the DEFAULT and the
//     documented target; live-setting it doesn't change what "done"
//     looks like, it just lets you get there without reflashing between
//     every step. The taper is already a fade (not a switch) and already
//     only touches spin-up torque, never braking -- see commandWheels()
//     below, unchanged from Stage5_Release -- so Test 6's two structural
//     requirements were already satisfied before this file existed.
//   - Test 7 (endurance run) instrumentation: a loop-overrun counter
//     (gLoopOverrunCount, any cycle whose measured dt exceeds 1.5x
//     nominal), per-wheel controller temperature (moteus's own
//     Query::Result.temperature -- requested by DEFAULT resolution, no
//     Format change needed, was already arriving in every reply unused),
//     and per-wheel saturation duty (gSatDuty, a 5s-low-pass fraction of
//     time each wheel's commanded torque sits at/near kTauMax). All three
//     are exactly the four signals Test 7 says to plot against time
//     (standing wheel speed and trim were already there).
//   - Telemetry decimated 10x (kTelemetryDecim) -- NOT primarily for the
//     bandwidth-blocking reason the WiFi builds decimate for (this file
//     uses Serial/USB, not a baud-rate-limited UART link), but because a
//     30-minute run at the full 500 Hz loop rate is ~900k telemetry
//     lines -- unwieldy to log and plot for no benefit, when nothing
//     Test 7 watches (temperature, standing speed, trim, saturation duty)
//     changes meaningfully faster than a few Hz anyway. 50 Hz is still
//     far more resolution than any of those four signals needs.
//   - BMI270 ODR raised to 800 Hz (was 400, against a 500 Hz loop) --
//     Test 2 finding, see cube-bringup/Stage0c_IMUJitter.ino, which
//     validated this exact fix and was never carried forward into any
//     balance-stage file until now.
//
// STAGE 5 PROCEDURE (unchanged from Stage5_Release.ino):
//   1. Cube resting on the corner Stage 1 identified, near the resolved
//      equilibrium (arm gate needs this, or send "z1" to fast-start trim).
//   2. Send "a1" to arm -- refused if not close enough.
//   3. Let go / give it a small push. Watch it recover, or watch it trip.
//   4. If it trips: check trip_reason in telemetry before re-arming. A
//      trip is not a bug to route around by disarming and re-arming
//      quickly -- read the number first, and check WHICH wheel's rho or
//      which axis of phi drove it before assuming it's the same failure
//      mode you saw last time.
//
// Still using the bench power supply, not the flight battery -- Sg/lambda
// (and therefore how well THIS Kp performs) are still whatever they are
// for the current, not final, mass distribution. Re-derive gains once the
// final mass is on, same measure -> derive -> re-tune workflow as
// everywhere else in this project.
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
// Deliberately does NOT touch gTripReason -- halting is an operator pause,
// not a trip, so if a real trip happened before you halted, that record
// stays visible until you actually re-arm. Same as edge-bringup's Stage 5.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from Stage 1-4)
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
// IMU calibration, THIS rig / THIS mount. Kept byte-identical across every
// stage in this folder (2026-08-21).
// NOTE: kAccelScale's third entry (0.568) is a 43% correction. That is not what
// a healthy accelerometer calibration looks like, and it distorts ghat -- and
// therefore corner resolution and phi -- whenever gravity is not aligned with a
// single IMU axis. Re-run IMU_Calibration.ino for this mount.
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
// SECTION 2c: CORNER CANDIDATE (resolved fresh here too)
// ----------------------------------------------------------------------------

struct CornerCandidate {
  const char* name;
  float gB[3];
  float Kp[3][9];   // rows = wheel X,Y,Z; cols = [phi(3) om(3) rho(3)]
  float placeOffsetDeg;
  float ellM;       // m, contact-to-COM lever arm -- NEW vs Stage5_Release.ino,
                     // used only to convert a converged trim into an
                     // equivalent COM offset in mm for telemetry.
};

// TODO: corner [-1,-1,-1]'s Kp below was UPDATED 2026-08-19 for the new
// plant (mass 1.633 kg + corner-to-housing strut); the other 7 corners are
// still the old 1.5668 kg/no-strut values -- do not treat this table as
// internally consistent across corners until they're all re-derived.
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
// actually uses, so a MEASURED equilibrium can replace the tabulated one.
//
// Why: a trim/offset can only nudge phi and is clamped, so once it saturates,
// re-seeding it is a fixed point that changes nothing. Capturing gB itself has
// no such bound and sets phi to exactly zero at the current pose.
static float gActiveGB[3] = { 0.0f, 0.0f, 1.0f };
static bool  gGBFromBench = false;

static void setActiveGBFromTable() {
  const float* t = kCorners[gCornerIdx].gB;
  gActiveGB[0] = t[0]; gActiveGB[1] = t[1]; gActiveGB[2] = t[2];
  normalize3(gActiveGB);
  gGBFromBench = false;
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

// AUTOMATIC TRIM state -- see Stage4_AutoTrim.ino's header for the full
// derivation. ADDED to the raw measurement below (phi_err = raw + trim,
// the note's own verified convention -- do not subtract).
static float gTrim[3]     = { 0.0f, 0.0f, 0.0f };   // rad, added to raw phi
static bool  gTrimEnabled = true;                    // "x0" freezes, "x1" resumes

// gB := ghat, so phi = -(gB x ghat) = 0 exactly at this pose.
static void captureEquilibrium() {
  const float* tab = kCorners[gCornerIdx].gB;
  const float old[3] = { gActiveGB[0], gActiveGB[1], gActiveGB[2] };

  gActiveGB[0] = ghat[0]; gActiveGB[1] = ghat[1]; gActiveGB[2] = ghat[2];
  normalize3(gActiveGB);
  gGBFromBench = true;
  gTrim[0] = gTrim[1] = gTrim[2] = 0.0f;   // stale against the new gB

  const float dM = dot3(old, gActiveGB), dT = dot3(tab, gActiveGB);
  Serial.print("# EQUILIBRIUM CAPTURED: gB = ");
  Serial.print(gActiveGB[0], 6); Serial.print(", ");
  Serial.print(gActiveGB[1], 6); Serial.print(", ");
  Serial.println(gActiveGB[2], 6);
  Serial.print("#   moved ");
  Serial.print(acosf(dM > 1.0f ? 1.0f : (dM < -1.0f ? -1.0f : dM)) * (float)RAD_TO_DEG, 3);
  Serial.print(" deg from the previous equilibrium, ");
  Serial.print(acosf(dT > 1.0f ? 1.0f : (dT < -1.0f ? -1.0f : dT)) * (float)RAD_TO_DEG, 3);
  Serial.println(" deg from the CAD table value.");
  Serial.println("#   phi now reads ~0 here. Kp is still the TABLE's gains for the");
  Serial.println("#   TABLE's geometry -- a large move means they no longer match");
  Serial.println("#   the cube. Send c to restore the table value.");
}

void updateCornerProjection() {
  float t[3];
  cross3(gActiveGB, ghat, t);
  phi[0] = -t[0] + gTrim[0];
  phi[1] = -t[1] + gTrim[1];
  phi[2] = -t[2] + gTrim[2];
}

// Strip yaw, clamp to TRIM_MAX -- see Stage4_AutoTrim.ino, identical.
static const float kTrimMax = 0.0349065850f;   // rad, 2 deg

void applyTrimGuards() {
  const float* gB = gActiveGB;
  const float along = dot3(gTrim, gB);
  gTrim[0] -= along * gB[0];
  gTrim[1] -= along * gB[1];
  gTrim[2] -= along * gB[2];

  const float n = norm3(gTrim);
  if (n > kTrimMax) {
    const float s = kTrimMax / n;
    gTrim[0] *= s; gTrim[1] *= s; gTrim[2] *= s;
  }
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------

// trip_reason: 0 none, 1 tilt (norm3(phi) > DISARM), 2 omega (any
// |rho[i]| > OMEGA_CAP), 3 nan. Latched until the next "a1" -- read it
// before re-arming, don't just re-arm and hope.
//
// temp[3]/satDuty[3]/overrunCount are Test 7's endurance-run instruments
// -- see the header note for what each one is and why. temp comes
// straight from moteus's own reply (Query::Result.temperature, requested
// at DEFAULT resolution -- no Format change needed, it was always in
// every reply already, just never read before this file).
void printState(uint32_t t_ms, const float rho[3], const float rhoLp[3],
                const float tau[3], const float tauCmd[3], bool armed,
                int tripReason, const float temp[3], const float satDuty[3],
                uint32_t overrunCount) {
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
  Serial.print('\t'); Serial.print(tripReason);
  // Automatic trim readout -- see Stage4_AutoTrim.ino for why both the
  // per-axis degrees and the mm conversion are logged.
  Serial.print('\t'); Serial.print(gTrim[0] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(gTrim[1] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(gTrim[2] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(norm3(gTrim) * kCorners[gCornerIdx].ellM * 1000.0f, 3);
  Serial.print('\t'); Serial.print(gTrimEnabled ? 1 : 0);
  // Test 7 endurance instrumentation.
  Serial.print('\t'); Serial.print(temp[0], 1);
  Serial.print('\t'); Serial.print(temp[1], 1);
  Serial.print('\t'); Serial.print(temp[2], 1);
  Serial.print('\t'); Serial.print(satDuty[0], 3);
  Serial.print('\t'); Serial.print(satDuty[1], 3);
  Serial.print('\t'); Serial.print(satDuty[2], 3);
  Serial.print('\t'); Serial.println(overrunCount);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL — full law (identical to Stage 4), real trip policy
// ----------------------------------------------------------------------------

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // already validated through Stage 4

// Real cube-wide policy from cubli_gains.h -- NOT Stage 4's loose values.
static const float kMaxTilt    = 0.261799395f;   // rad, 15 deg -- DISARM, vs norm3(phi)
static const float kTauMax     = 0.12f;          // N*m -- TAU_MAX

// Test 6 (wheel-speed cap): live-settable, NOT compiled-in constants --
// "o<rad/s>" / "p<rad/s>". Real policy DEFAULT is still 40/36 (unchanged
// from Stage5_Release.ino) -- live-setting is for the tightening
// procedure itself (set to ~3x post-trim standing speed, verify, step
// toward these defaults), not a way to ship with a different policy.
// Reset to default with "o40" "p36" if you lose track of where you left it.
static float gMaxOmega   = 40.0f;   // rad/s -- OMEGA_CAP, per wheel
static float gTaperStart = 36.0f;   // rad/s

static const float kTauCw  = 0.008f;   // N*m, PLACEHOLDER
static const float kBw     = 0.0f;     // N*m*s, PLACEHOLDER
static const float kEpsFf  = 0.05f;    // rad/s

static const float kArmGate = 0.00872664619f;   // rad, 0.5 deg

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
  +1.0f,   // X (this rig, moteus id 1)
  +1.0f,   // Y (this rig, moteus id 3)
  +1.0f,   // Z (this rig, moteus id 2)
};

enum TripReason { TRIP_NONE = 0, TRIP_TILT = 1, TRIP_OMEGA = 2, TRIP_NAN = 3 };
static int gTripReason = TRIP_NONE;

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

static float gLastTau[3]    = { 0.0f, 0.0f, 0.0f };
static float gLastTauCmd[3] = { 0.0f, 0.0f, 0.0f };
static float gRhoLp[3]      = { 0.0f, 0.0f, 0.0f };

// Test 7 (endurance run) instrumentation -- see header note. gSatDuty is a
// 5s-low-pass (same tau as gRhoLp, same alphaLp below) fraction of time
// each wheel's OUTGOING command sits at/near kTauMax -- 0.0 = never
// saturating, 1.0 = pinned at the limit continuously. gMotorTemp is read
// straight from each moteus's own reply, no extra query needed.
static float gSatDuty[3]    = { 0.0f, 0.0f, 0.0f };
static float gMotorTemp[3]  = { 0.0f, 0.0f, 0.0f };

Moteus& wheelObj(int i) {
  return i == 0 ? moteusX : (i == 1 ? moteusY : moteusZ);
}

// ---------------------------- YAW PROJECTION --------------------------------
// Rotation about gB (the balancing axis) is the controller's null mode: the
// reduced-attitude estimator cannot OBSERVE it (phi = -(gB x ghat) is
// perpendicular to gB by construction, so the state is effectively 8, not 9),
// and reaction wheels cannot hold sustained torque about it anyway.
// Measured null-vs-gB misalignment of the om block, and the residual yaw torque
// it leaves as a fraction of a full Kp row:
//   the two same-sign corners: 0.3-1.4 deg off,  0.7-3.0%
//   the six MIXED corners:     4.5-5.9 deg off,  9.6-12.6%
// Both phi and om are projected onto the plane perpendicular to gB before Kp.
// For a raw phi that is a no-op; what it removes is any gB component carried by
// the offset/trim, plus -- importantly -- the gravity-aligned gyro bias, which
// is exactly the residual the filter can never null. Toggle with "y0"/"y1".
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
  // phi and om are yaw-projected before Kp; telemetry reports raw.
  float phiUsed[3], omUsed[3];
  projectOutYaw(phi, phiUsed);
  projectOutYaw(w_b, omUsed);
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
      float s = (gMaxOmega - fabsf(rho[i])) / (gMaxOmega - gTaperStart);
      s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
      u *= s;
    }

    if (!isfinite(u)) { u = 0.0f; }
    u = u >  kTauMax ?  kTauMax : u;
    u = u < -kTauMax ? -kTauMax : u;
    tau[i] = u;
  }

  // --- latching trips: record WHY, print once, never auto-unlatch ---
  if (gArmed && norm3(phi) > kMaxTilt) {
    gArmed = false; gTripReason = TRIP_TILT;
    Serial.println("# TRIP: tilt limit");
  }
  for (int i = 0; i < 3; ++i) {
    if (gArmed && fabsf(rho[i]) > gMaxOmega) {
      gArmed = false; gTripReason = TRIP_OMEGA;
      Serial.print("# TRIP: wheel speed limit, wheel ");
      Serial.println(i == 0 ? "X" : i == 1 ? "Y" : "Z");
    }
  }
  if (gArmed && (!isfinite(phi[0]) || !isfinite(phi[1]) || !isfinite(phi[2]) ||
                 !isfinite(w_b[0]) || !isfinite(w_b[1]) || !isfinite(w_b[2]) ||
                 !isfinite(tau[0]) || !isfinite(tau[1]) || !isfinite(tau[2]))) {
    gArmed = false; gTripReason = TRIP_NAN;
    Serial.println("# TRIP: non-finite state or torque");
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

    const bool saturated = fabsf(tau_cmd) >= (kTauMax * 0.99f);
    gSatDuty[i] += alphaLp * ((saturated ? 1.0f : 0.0f) - gSatDuty[i]);
    gMotorTemp[i] = (float)wheelObj(i).last_result().values.temperature;
  }
}


// ----------------------------------------------------------------------------
// SECTION 2e-2: AUTOMATIC TRIM ADAPTATION -- identical to Stage4_AutoTrim.ino
// ----------------------------------------------------------------------------

static float gKAdapt = 2.922e-6f;   // tau_a = 60 s -- see Stage4_AutoTrim.ino

static const float kTiltQuiet  = 0.00872664619f;   // rad, 0.5 deg
static const float kOmegaQuiet = 0.10f;              // rad/s

void updateTrim(float dt) {
  if (!gTrimEnabled || !gArmed) { return; }
  if (norm3(phi) >= kTiltQuiet)  { return; }
  if (norm3(w_b)  >= kOmegaQuiet) { return; }

  for (int i = 0; i < 3; ++i) {
    gTrim[i] += gKAdapt * dt * gRhoLp[i];   // PLUS -- VERIFIED, see Stage4_AutoTrim.ino
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
        gTripReason = TRIP_NONE;   // manual re-arm clears the latch
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
  } else if (cmd == 'c') {
    resolveCornerCandidate();
  } else if (cmd == 'r') {
    // Bookmark only -- no effect on control. Same convention as edge-
    // bringup's Stage 5: send this the instant you let go, val is just a
    // log label. For real time alignment, prefer detecting where phi
    // starts moving on its own in the data over trusting this timestamp --
    // human release timing has more jitter than the control loop does.
    Serial.print("# RECORD_START t_ms="); Serial.print(millis());
    Serial.print(" marker="); Serial.println(val, 2);
  } else if (cmd == 'z') {
    if (val != 0.0f) {
      gTrim[0] -= phi[0];
      gTrim[1] -= phi[1];
      gTrim[2] -= phi[2];
      applyTrimGuards();
      Serial.print("# gTrim SEEDED to: ");
      Serial.print(gTrim[0] * (float)RAD_TO_DEG, 3); Serial.print(",");
      Serial.print(gTrim[1] * (float)RAD_TO_DEG, 3); Serial.print(",");
      Serial.print(gTrim[2] * (float)RAD_TO_DEG, 3); Serial.println(" deg");
    } else {
      gTrim[0] = gTrim[1] = gTrim[2] = 0.0f;
      Serial.println("# gTrim cleared to 0,0,0");
    }
  } else if (cmd == 'x') {
    gTrimEnabled = (val != 0.0f);
    Serial.print("# gTrimEnabled = ");
    Serial.println(gTrimEnabled ? "TRUE (adapting)" : "FALSE (frozen at current value)");
  } else if (cmd == 'k') {
    gKAdapt = val;
    Serial.print("# gKAdapt = "); Serial.println(gKAdapt, 8);
  } else if (cmd == 'o') {
    // Test 6: set omega_cap. "o40" restores the real policy default.
    gMaxOmega = val;
    Serial.print("# gMaxOmega (omega_cap) = "); Serial.print(gMaxOmega, 2);
    Serial.println(" rad/s");
    if (gTaperStart >= gMaxOmega) {
      Serial.println("# WARNING: gTaperStart >= gMaxOmega -- taper has no "
                      "room to fade in, set p below o.");
    }
  } else if (cmd == 'p') {
    // Test 6: set taper_start. "p36" restores the real policy default
    // (90% of the real 40 rad/s cap).
    gTaperStart = val;
    Serial.print("# gTaperStart = "); Serial.print(gTaperStart, 2);
    Serial.println(" rad/s");
  } else if (cmd == 'y') {
    gYawProject = (val != 0.0f);
    Serial.print("# gYawProject = ");
    Serial.println(gYawProject ? "TRUE (phi and om projected onto the plane perp to gB)"
                                : "FALSE (raw phi/om into Kp)");
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
    Serial.println("# unknown. use: a<0/1>  c (re-resolve)  r<val> (log marker)  "
                    "z<0/1> (clear/seed trim)  x<0/1> (freeze/resume trim)  "
                    "k<value> (trim gain)  o<rad/s> (omega_cap)  "
                    "p<rad/s> (taper_start)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 5: RELEASE + AUTO TRIM (stop/catch + e-stop, cube free)");

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

  // 800 Hz, not 400 -- Test 2 finding, see cube-bringup/Stage0c_IMUJitter.ino
  // (already validated this fix) and Stage4_AutoTrim.ino's header for the
  // full reasoning: this loop runs at 500 Hz, so a 400 Hz ODR means the
  // loop outruns the sensor and ~1 in 5 reads was a stale repeat.
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
                  "armed\ttrip_reason\t"
                  "trim_x_deg\ttrim_y_deg\ttrim_z_deg\ttrim_com_mm\ttrim_enabled\t"
                  "temp_x\ttemp_y\ttemp_z\tsat_duty_x\tsat_duty_y\tsat_duty_z\t"
                  "loop_overrun_count");
  Serial.println("# STARTS DISARMED. Stop/catch + e-stop ready BEFORE sending a1.");
  Serial.println("# trip_reason: 0 none  1 tilt  2 omega  3 nan");
  Serial.println("# z1/z0 seed/clear trim, x1/x0 freeze/resume adaptation,");
  Serial.println("# k<value> sets adaptation gain (default tau_a=60s).");
  Serial.println("# o<rad/s>/p<rad/s> set omega_cap/taper_start LIVE (Test 6 --");
  Serial.println("# real policy default is 40/36, o40 p36 restores it).");
  Serial.println("# Telemetry decimated 10x (~50 Hz) -- see header note.");
  Serial.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");

  gNextSendMillis = millis();
}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

// Test 7: any cycle whose measured dt exceeds 1.5x nominal (2ms -> 3ms)
// counts as an overrun -- something (a Serial write, a CAN retry, GC-style
// pause) made this cycle late. Compiled-in threshold, not live-settable --
// unlike the Test 6 constants above, there's no bench procedure that wants
// to move this around, it's a pass/fail instrument, not a tuning knob.
static const float kOverrunThresholdS = 0.003f;
static uint32_t gLoopOverrunCount = 0;

// Telemetry decimation -- see header note. NOT the WiFi builds' bandwidth
// reasoning (this is Serial/USB, not a baud-rate-limited UART); this is
// purely "a 30 min run doesn't need 900k lines to make its point."
static const uint32_t kTelemetryDecim = 10;   // 500 Hz / 10 = 50 Hz
static uint32_t gTelemetryCounter = 0;

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
  if (dt > kOverrunThresholdS) { gLoopOverrunCount++; }

  float aImu[3], wImu[3];
  imu.getSensorData();
  readIMURaw(imu, aImu, wImu);
  attitudeUpdate(aImu, wImu, dt);
  updateCornerProjection();

  const float rho[3] = {
    kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI,
  };

  commandWheels(rho);
  updateTrim(dt);   // after commandWheels() so this cycle's gRhoLp is fresh

  if (++gTelemetryCounter >= kTelemetryDecim) {
    gTelemetryCounter = 0;
    printState(time, rho, gRhoLp, gLastTau, gLastTauCmd, gArmed, gTripReason,
              gMotorTemp, gSatDuty, gLoopOverrunCount);
  }
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// This file is Cube Fine-Tuning -- Test Plan.md's Session 2/3 (Tests 5, 6,
// 7), on ONE corner. Suggested order, matching the Test Plan's own:
//   1. Test 5: arm, let trim converge (watch trim_x/y/z_deg settle, ~4
//      time constants at the default tau_a=60s is ~4 min), or "z1" to
//      fast-start.
//   2. Test 6: with trim converged, read the settled rho_x/y/z_lp
//      (standing_speed_report.py, or just watch telemetry), set
//      "o<3x that>" and "p<90% of that>", verify it still balances a
//      couple minutes, then step both toward the real policy (o40 p36)
//      as trim settles further.
//   3. Test 7: once 6 is done, arm and leave it -- 30 min, then check
//      trim/standing-speed/temperature/loop_overrun_count/sat_duty
//      against Test 7's table (plot_session_csv.py --cols trim,wheels,
//      endurance, or fine-tuning/README.md for the exact commands).
//
// Re-derive Kp once the battery cable/DC-DC (and eventually the flight
// battery itself) are actually mounted (measure -> derive -> re-tune, same
// workflow as everywhere else in this project) -- trim will re-converge to
// a different, smaller value once they are, that is not a sign anything
// here is wrong. Then repeat this file's Tests 5-7 for the OTHER SEVEN
// corners before trusting any of them -- a result on one corner says
// nothing about another (same discipline edge-bringup established per-
// axis, now per-corner). Once multiple corners are validated, corner-to-
// corner transitions are where the quaternion/MEKF estimator becomes
// load-bearing (Attitude representation for the firmware.md section 4) --
// this reduced-attitude gam estimator is deliberately NOT that, and isn't
// meant to be pushed into large-angle territory.
// ============================================================================
