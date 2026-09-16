// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (I2C) —
// CORNER STAGE 1: CORNER ID + PER-WHEEL PULSE CHECK
// ============================================================================
// CUBE HELD FIRMLY BY HAND (or braced), resting on ONE corner. No feedback
// loop yet -- all three wheels are commanded every cycle (SetPosition sent
// each 2 ms tick, watchdog-safe), but only ONE wheel at a time carries
// nonzero torque, selected with "w<0/1/2>" and fired with "p", same method
// as edge-bringup's Stage 1. Direct analogue of that file's checklist, now
// asking it of all three wheels together instead of one axis at a time.
//
// kWheelSign[3] is carried forward from edge-bringup, ALL THREE CONFIRMED
// +1.0f there (Stage1_WheelSignCheck.ino, X/Y/Z, real hardware) -- same
// wheels, same wiring, same moteus torque-mode convention, so there is no
// reason to expect it to differ here. This stage re-confirms it anyway,
// cheaply, because the STATE VECTOR framing is new (om/rho are now full
// body-frame vectors, not edge's single scalar projection) and because
// three wheels commanded simultaneously for the first time is itself worth
// watching before any gain is applied (Firmware Lessons S4/S7).
//
// >>> SIGN CHECK 4 IS THE ONE THAT KILLS HARDWARE. <<<
// If a positive torque on a wheel pushes the cube the WRONG way (phi should
// shrink, not grow, while that wheel's rho increases), the correct-looking
// negative sign in Stage 2's Kp matrix will actively drive the fall once
// the loop closes. Fix kAxisWheelSign here -- NEVER flip a sign inside Kp
// to compensate; Kp comes verbatim from cubli_gains.h and is not the place
// to patch a wiring-convention problem.
//
// STAGE 1 CHECKLIST (~20 min) — cube held/braced on one corner.
//   Send "c" to resolve which of the 8 corners is currently down -- confirm
//   it matches the corner you think you're resting on before doing anything
//   else. Then for EACH wheel (w0=X, w1=Y, w2=Z):
//   [ ] "w<i>" to select it, "p" to fire ONE pulse (0.05 N*m, 1000 ms)
//   [ ] that wheel's rho[i] goes POSITIVE for a positive pulse
//   [ ] the OTHER two wheels' rho stay ~0 (open loop -- nothing should be
//       driving them; if one drifts, that's coupling/friction worth noting,
//       not a fault by itself)
//   [ ] cube pushes toward DECREASING |phi| (back toward the resolved
//       corner's gB) while the pulsed wheel speeds up
//
// 0.05 N*m starting point, same as edge-bringup and the panel before it --
// confirm/adjust for this rig with "t<Nm>".
//
// ---------------------------- WHY gam HERE ---------------------------------
// Same reduced-attitude estimator as edge-bringup and Gam/Skeleton_3Axis.ino
// (ghat/w_b, kP=4/kI=0.5 cross-product complementary filter from Attitude
// representation for the firmware.md), unchanged. What's new in this folder
// is downstream of the estimator: phi is now the FULL 3-vector
// -cross(gB, ghat) (not projected onto one edge direction), om is w_b
// directly, and there's an eight-way corner identification instead of
// edge's two-way per-axis check -- ANY of the cube's 8 corners could be the
// one currently down, resolved by measurement against cubli_gains.h's
// CORNER table, never assumed from memory (same discipline as edge's
// candidate resolution, extended from 2 candidates to 8).
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

// THIS RIG, confirmed on bench via this stage's own corner-ID + pulse
// checklist (2026-08-21): id 1 -> X, id 3 -> Y, id 2 -> Z. NOT the same
// mapping as the old cube (Gam/Skeleton_3Axis.ino / edge-bringup had
// id 2 -> X, id 3 -> Y, id 1 -> Z) -- X and Z are swapped here. Corner
// balance needs all three wheels live at once, so unlike edge-bringup there
// is no kAxis selector here -- every stage in this folder always
// instantiates all three.
Moteus moteusX(canBus, []() { Moteus::Options options; options.id = 1; return options; }());
Moteus moteusY(canBus, []() { Moteus::Options options; options.id = 3; return options; }());
Moteus moteusZ(canBus, []() { Moteus::Options options; options.id = 2; return options; }());

BMI270 imu;
// I2C, not SPI, on this rig: BMI2_I2C_PRIM_ADDR (0x68) is the SDO-low address
// -- if SDO is tied high on this breakout it's BMI2_I2C_SEC_ADDR (0x69)
// instead. 400 kHz is the BMI270's I2C fast-mode ceiling (Bosch datasheet);
// a 12-byte accel+gyro burst read at 400 kHz is ~340 us, comfortably inside
// the 2 ms (500 Hz) loop period below, so the interface change costs nothing
// timing-wise.
const uint8_t  imuI2CAddress = BMI2_I2C_PRIM_ADDR;
const uint32_t imuI2CClockHz = 400000;
const uint32_t kPeriodMs = 2;   // 500 Hz -- matches edge-bringup's rate.
                                 // Gains here are continuous-time, fine to
                                 // differ from cubli_gains.h's 400Hz design rate.

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Same semantics as every panel/edge stage: loop() keeps running regardless
// of the Serial Monitor window; halting freezes IMU/CAN/telemetry and
// cancels any in-flight pulse.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam, identical to edge-bringup
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
// TODO: still the 2D-panel's mount constants, carried through cube-bringup
// and edge-bringup unchanged. Replace with this rig's own numbers once
// IMU_Calibration.ino has been re-run for THIS mount. Telemetry-only in
// this stage so it doesn't block the pulse test, but don't trust phi/om
// numerically until this is real.
static const float kGyroBias[3]    = { -0.001956f, -0.009486f, -0.001770f };
static const float  kAccelOffset[3] = { +0.008400f, -0.090030f, +0.001695f };  // m/s^2
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

// Reduced-attitude complementary filter, kP=4/kI=0.5 -- the validated form.
// e = ghat x ga (filter innovation, magnitude ~= sin(error)), correction is
// kP*(e x ghat), bias integral is PLUS kI*e*dt (Attitude representation for
// the firmware.md: the opposite sign converges to 5x the true bias).
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

  // Antipode reset (Attitude representation for the firmware.md's numerical
  // caution): e's magnitude is sin(error), zero again at 180 deg -- if the
  // cube gets picked up and inverted, ghat can lock onto the antipode.
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
// SECTION 2c: CORNER CANDIDATE -- resolved by measurement, not memory
// ----------------------------------------------------------------------------
// All 8 corners from cubli_gains.h's CORNER table, verbatim (gB and the
// full 3x9 Kp per corner -- Kp isn't used by this stage's open-loop pulse,
// but it's included here so every corner-bringup file carries the SAME
// literal table, copy-paste identical, same convention edge-bringup used
// for its per-axis candidates). ID_SEP_MIN in cubli_gains.h (67.2 deg) is
// the minimum angular separation between any two corner gB vectors -- a
// correct resolution's best-vs-runner-up dot product margin should be much
// larger than the ad hoc 0.2 warning threshold below; if it isn't, distrust
// the identification before distrusting the threshold.

struct CornerCandidate {
  const char* name;
  float gB[3];
  float Kp[3][9];   // rows = wheel X,Y,Z; cols = [phi(3) om(3) rho(3)]
  float placeOffsetDeg;
};

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
  { "[-1,-1,-1]"  // lean 0.797 deg vs body diagonal, ell 122.84 mm, Sg 1.8875, lambda 8.2572
    , { -0.583713f, -0.584069f, -0.564042f }
    , { { -4.835f, 2.4877f, 2.4276f, -0.6198f, 0.3174f, 0.3096f, -0.0009f, 0.0021f, 0.0021f },  // wheel X
        { 2.4904f, -4.8347f, 2.4291f, 0.3174f, -0.6202f, 0.3095f, 0.0021f, -0.0009f, 0.0021f },  // wheel Y
        { 2.4045f, 2.4033f, -4.977f, 0.31f, 0.3098f, -0.6346f, 0.0023f, 0.0023f, -0.0008f } } // wheel Z
    , 0.928724393f },
  { "[-1,-1,+1]"  // lean 3.170 deg vs body diagonal, ell 128.54 mm, Sg 1.9750, lambda 8.1212
    , { -0.537196f, -0.537524f, 0.649991f }
    , { { -4.835f, 2.4877f, -2.4276f, -0.6198f, 0.3174f, -0.3096f, -0.0009f, 0.0021f, -0.0021f },  // wheel X
        { 2.4904f, -4.8347f, -2.4291f, 0.3174f, -0.6202f, -0.3095f, 0.0021f, -0.0009f, -0.0021f },  // wheel Y
        { -2.4045f, -2.4033f, -4.977f, -0.31f, -0.3098f, -0.6346f, -0.0023f, -0.0023f, -0.0008f } } // wheel Z
    , 5.27655426f },
  { "[-1,+1,-1]"  // lean 2.773 deg vs body diagonal, ell 126.08 mm, Sg 1.9373, lambda 8.1591
    , { -0.549159f, 0.645625f, -0.530653f }
    , { { -4.835f, -2.4877f, 2.4276f, -0.6198f, -0.3174f, 0.3096f, -0.0009f, -0.0021f, 0.0021f },  // wheel X
        { -2.4904f, -4.8347f, -2.4291f, -0.3174f, -0.6202f, -0.3095f, -0.0021f, -0.0009f, -0.0021f },  // wheel Y
        { 2.4045f, -2.4033f, -4.977f, 0.31f, -0.3098f, -0.6346f, 0.0023f, -0.0023f, -0.0008f } } // wheel Z
    , 5.00861047f },
  { "[-1,+1,+1]"  // lean 3.097 deg vs body diagonal, ell 131.64 mm, Sg 2.0226, lambda 8.0480
    , { -0.509899f, 0.599468f, 0.616962f }
    , { { -4.835f, -2.4877f, -2.4276f, -0.6198f, -0.3174f, -0.3096f, -0.0009f, -0.0021f, -0.0021f },  // wheel X
        { -2.4904f, -4.8347f, 2.4291f, -0.3174f, -0.6202f, 0.3095f, -0.0021f, -0.0009f, 0.0021f },  // wheel Y
        { -2.4045f, 2.4033f, -4.977f, -0.31f, 0.3098f, -0.6346f, -0.0023f, 0.0023f, -0.0008f } } // wheel Z
    , 4.65881833f },
  { "[+1,-1,-1]"  // lean 3.171 deg vs body diagonal, ell 128.56 mm, Sg 1.9753, lambda 8.1180
    , { 0.645701f, -0.549274f, -0.530441f }
    , { { -4.835f, -2.4877f, -2.4276f, -0.6198f, -0.3174f, -0.3096f, -0.0009f, -0.0021f, -0.0021f },  // wheel X
        { -2.4904f, -4.8347f, 2.4291f, -0.3174f, -0.6202f, 0.3095f, -0.0021f, -0.0009f, 0.0021f },  // wheel Y
        { -2.4045f, 2.4033f, -4.977f, -0.31f, 0.3098f, -0.6346f, -0.0023f, 0.0023f, -0.0008f } } // wheel Z
    , 5.01640064f },
  { "[+1,-1,+1]"  // lean 2.609 deg vs body diagonal, ell 134.01 mm, Sg 2.0591, lambda 7.9804
    , { 0.599572f, -0.510034f, 0.616749f }
    , { { -4.835f, -2.4877f, 2.4276f, -0.6198f, -0.3174f, 0.3096f, -0.0009f, -0.0021f, 0.0021f },  // wheel X
        { -2.4904f, -4.8347f, -2.4291f, -0.3174f, -0.6202f, -0.3095f, -0.0021f, -0.0009f, -0.0021f },  // wheel Y
        { 2.4045f, -2.4033f, -4.977f, 0.31f, -0.3098f, -0.6346f, 0.0023f, -0.0023f, -0.0008f } } // wheel Z
    , 4.64807997f },
  { "[+1,+1,-1]"  // lean 3.095 deg vs body diagonal, ell 131.66 mm, Sg 2.0229, lambda 8.0501
    , { 0.611553f, 0.611236f, -0.502388f }
    , { { -4.835f, 2.4877f, -2.4276f, -0.6198f, 0.3174f, -0.3096f, -0.0009f, 0.0021f, -0.0021f },  // wheel X
        { 2.4904f, -4.8347f, -2.4291f, 0.3174f, -0.6202f, -0.3095f, 0.0021f, -0.0009f, -0.0021f },  // wheel Y
        { -2.4045f, -2.4033f, -4.977f, -0.31f, -0.3098f, -0.6346f, -0.0023f, -0.0023f, -0.0008f } } // wheel Z
    , 5.10629368f },
  { "[+1,+1,+1]"  // lean 0.714 deg vs body diagonal, ell 136.99 mm, Sg 2.1048, lambda 7.9341
    , { 0.571935f, 0.571638f, 0.58832f }
    , { { -4.835f, 2.4877f, 2.4276f, -0.6198f, 0.3174f, 0.3096f, -0.0009f, 0.0021f, 0.0021f },  // wheel X
        { 2.4904f, -4.8347f, 2.4291f, 0.3174f, -0.6202f, 0.3095f, 0.0021f, -0.0009f, 0.0021f },  // wheel Y
        { 2.4045f, 2.4033f, -4.977f, 0.31f, 0.3098f, -0.6346f, 0.0023f, 0.0023f, -0.0008f } } // wheel Z
    , 0.77358409f },
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
    Serial.println("# WARNING: best and runner-up corners are close -- cube may not");
    Serial.println("#   be resting stably on a single corner yet. Recheck before Stage 2.");
  }
}

float phi[3] = { 0.0f, 0.0f, 0.0f };
// om is just w_b, no separate variable needed -- read directly at use.
float rho[3] = { 0.0f, 0.0f, 0.0f };

// gB := ghat, so phi = -(gB x ghat) = 0 exactly at this pose.
static void captureEquilibrium() {
  const float* tab = kCorners[gCornerIdx].gB;
  const float old[3] = { gActiveGB[0], gActiveGB[1], gActiveGB[2] };

  gActiveGB[0] = ghat[0]; gActiveGB[1] = ghat[1]; gActiveGB[2] = ghat[2];
  normalize3(gActiveGB);
  gGBFromBench = true;

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
  phi[0] = -t[0]; phi[1] = -t[1]; phi[2] = -t[2];
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------

void printState(uint32_t t_ms, int pulseWheel, float tau, bool pulseActive) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(pulseWheel);
  Serial.print('\t'); Serial.print(tau, 4);
  Serial.print('\t'); Serial.print(pulseActive ? 1 : 0);
  Serial.print('\t'); Serial.print(phi[0] * (float)RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(phi[1] * (float)RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(phi[2] * (float)RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(w_b[0] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(w_b[1] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(w_b[2] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(rho[0], 3);
  Serial.print('\t'); Serial.print(rho[1], 3);
  Serial.print('\t'); Serial.println(rho[2], 3);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL -- single-shot open-loop pulse on ONE selected wheel,
// no gains involved. The other two wheels always get a zero-torque command
// (same Format/watchdog contract) so all three stay in torque mode together.
// ----------------------------------------------------------------------------

static float    gTauPulse       = 0.05f;   // N*m -- live-settable, see "t<Nm>".
static const uint32_t kPulseDurationMs = 1000;
static const float kTauMax      = 0.12f;   // N*m, TAU_MAX from cubli_gains.h

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

static int      gPulseWheel   = 0;       // 0=X 1=Y 2=Z -- select with "w<i>"
static bool     gPulseActive  = false;
static uint32_t gPulseStartMs = 0;
static float    gLastTau      = 0.0f;

Moteus& wheelObj(int i) {
  return i == 0 ? moteusX : (i == 1 ? moteusY : moteusZ);
}

void commandWheels() {
  float tau = 0.0f;
  if (gPulseActive) {
    if (millis() - gPulseStartMs < kPulseDurationMs) {
      tau = gTauPulse;
    } else {
      gPulseActive = false;
      Serial.println("# PULSE END");
    }
  }

  if (!isfinite(tau)) { tau = 0.0f; gPulseActive = false; }
  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  for (int i = 0; i < 3; ++i) {
    const float tau_i = (i == gPulseWheel) ? tau : 0.0f;
    Moteus::PositionMode::Command cmd;
    cmd.position               = NaN;
    cmd.velocity               = 0.0f;
    cmd.kp_scale                = 0.0f;
    cmd.kd_scale                 = 0.0f;
    cmd.feedforward_torque       = kAxisWheelSign[i] * tau_i;
    cmd.maximum_torque           = kTauMax;
    cmd.watchdog_timeout          = 0.10f;
    cmd.ignore_position_bounds    = 1.0f;
    wheelObj(i).SetPosition(cmd, &kTorqueFormat);
  }

  gLastTau = tau;
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

  if (cmd == 'w') {
    const int val = line.substring(1).toInt();
    if (val >= 0 && val <= 2) {
      gPulseWheel = val;
      Serial.print("# pulse wheel = "); Serial.println(val == 0 ? "X" : val == 1 ? "Y" : "Z");
    } else {
      Serial.println("# invalid, use w0 (X) w1 (Y) w2 (Z)");
    }
  } else if (cmd == 'p') {
    if (gPulseActive) {
      Serial.println("# pulse already running, ignored");
    } else {
      gPulseActive  = true;
      gPulseStartMs = millis();
      Serial.print("# PULSE START on wheel ");
      Serial.print(gPulseWheel == 0 ? "X" : gPulseWheel == 1 ? "Y" : "Z");
      Serial.print(": "); Serial.print(gTauPulse, 4);
      Serial.println(" N*m for 1000 ms");
    }
  } else if (cmd == 't') {
    const float val = line.substring(1).toFloat();
    gTauPulse = val >  kTauMax ?  kTauMax : val < -kTauMax ? -kTauMax : val;
    Serial.print("# gTauPulse = "); Serial.print(gTauPulse, 4); Serial.println(" N*m");
  } else if (cmd == 'c') {
    resolveCornerCandidate();
  } else if (cmd == 'b') {
    // Re-run the gyro bias calibration ON DEMAND. The boot run only captures
    // the bias at that instant; it drifts thermally, and the component ALONG
    // GRAVITY is invisible to the complementary filter (its innovation
    // e = ghat x ga has no component along ghat), so nothing downstream can
    // ever remove it. A fresh static calibration is the only thing that can.
    // Measured on this rig: 6.2 dps residual, 6.16 of it gravity-aligned.
    Serial.println("# Re-calibrating gyro bias -- hold the cube PERFECTLY STILL (~2 s)");
    const long bn = line.substring(1).toInt();
    calibrateGyroBias(bn > 0 ? (uint16_t)bn : 1000);
    // bhat was estimated against the OLD bias; keeping it double-corrects.
    bhat[0] = bhat[1] = bhat[2] = 0.0f;
    Serial.print("# gyro bias (body, rad/s): ");
    Serial.print(gGyroBiasBody[0], 6); Serial.print('	');
    Serial.print(gGyroBiasBody[1], 6); Serial.print('	');
    Serial.println(gGyroBiasBody[2], 6);
    Serial.println("# om_* should now read ~0 while static. If it does not,");
    Serial.println("#   the cube moved during the 2 s window -- redo it.");
  } else if (cmd == 'z') {
    // Capture the current attitude as the equilibrium (gB := ghat). Useful here
    // too: this is the stage where the corner is first identified, and the CAD
    // table value may not match the cube's actual COM.
    if (line.substring(1).toInt() != 0) {
      captureEquilibrium();
    } else {
      setActiveGBFromTable();
      Serial.println("# gB restored to the CAD table value");
    }
  } else if (cmd == 'h') {
    const float val = line.substring(1).toFloat();
    gHalted = (val != 0.0f);
    if (gHalted && gPulseActive) {
      gPulseActive = false;
      Serial.println("# pulse cancelled by halt");
    }
    Serial.print("# gHalted = ");
    Serial.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                            : "FALSE (resumed)");
  } else {
    Serial.println("# unknown. use: w<0/1/2> (select wheel)  p (fire pulse)  "
                    "t<Nm> (pulse size)  c (re-resolve corner)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 1: CORNER ID + PULSE CHECK (cube held/braced)");

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

  // 800 Hz, not 400 -- this loop still runs at 500 Hz (kPeriodMs = 2 below),
  // so a 400 Hz ODR means the loop polls faster than the sensor produces new
  // samples and roughly 1 in 5 reads is a stale repeat, not new information.
  // Same fix CubliBalance.ino carries (see its Stage0c_IMUJitter.ino note);
  // this stage predates that fix and still had the stale 400 Hz value.
  // Independent of the SPI->I2C change above -- ODR is a sensor-internal
  // sampling-engine setting, not an interface-speed one.
  if (imu.setAccelODR(BMI2_ACC_ODR_800HZ) != BMI2_OK ||
      imu.setGyroODR(BMI2_GYR_ODR_800HZ)  != BMI2_OK) {
    Serial.println("Warning: could not raise BMI270 ODR to 800 Hz");
  }

  Serial.println("# calibrating gyro bias -- keep the cube PERFECTLY STILL (~2s)");
  calibrateGyroBias(1000);
  Serial.print("# gyro bias (body, rad/s): ");
  Serial.print(gGyroBiasBody[0], 6); Serial.print('\t');
  Serial.print(gGyroBiasBody[1], 6); Serial.print('\t');
  Serial.println(gGyroBiasBody[2], 6);

  // Prime ghat with a few real cycles before resolving the corner candidate
  // -- the very first attitudeUpdate() call only initializes ghat from a
  // single raw sample, no filtering yet.
  for (int i = 0; i < 50; ++i) {
    float aImu[3], wImu[3];
    imu.getSensorData();
    readIMURaw(imu, aImu, wImu);
    attitudeUpdate(aImu, wImu, 0.002f);
    delay(2);
  }
  resolveCornerCandidate();

  Serial.println("t_ms\tpulse_wheel\ttau_Nm\tpulse_active\t"
                  "phi_x_deg\tphi_y_deg\tphi_z_deg\t"
                  "om_x_dps\tom_y_dps\tom_z_dps\t"
                  "rho_x\trho_y\trho_z");
  Serial.println("# send c to re-resolve the corner candidate, w<0/1/2> to pick");
  Serial.println("# a wheel (0=X 1=Y 2=Z), p to fire a pulse, t<Nm> to change its");
  Serial.println("# size (try t0.08 if 0.05 doesn't move it)");
  Serial.println("# HOLD/BRACE THE CUBE FIRMLY BEFORE SENDING p");

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

  commandWheels();

  rho[0] = kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI;
  rho[1] = kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI;
  rho[2] = kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI;

  printState(time, gPulseWheel, gLastTau, gPulseActive);
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: Stage2_RateOnly/Stage2_RateOnly.ino -- first closed-loop term (the
// om/rate block of each wheel's Kp row only, phi and rho columns masked to
// zero), still hand-held. This is also where the safety scaffold (latching
// trips, arm gate) gets introduced, same shape as edge-bringup's Stage 2.
// Do not proceed if any of this stage's three per-wheel sign checks failed,
// or if the corner resolution warning fired and wasn't resolved.
// ============================================================================
