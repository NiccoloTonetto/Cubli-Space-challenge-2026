// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 2: RATE (om) ONLY
// ============================================================================
// CUBE STILL HELD BY HAND. First closed-loop term: only the om block
// (columns 3-5) of each wheel's 9-wide Kp row is applied -- phi and rho
// columns (0-2, 6-8) are masked to zero in the state vector before the
// 3x9 dot product, same idea as edge-bringup's K[0]=K[2]=0, just done by
// zeroing the shared x-vector's entries rather than by only having a
// scalar K[1] to begin with (corner's Kp is a full coupled 3x9 matrix,
// not three independent scalars -- masking the STATE, not the gain, is
// what keeps all five corner stages sharing byte-identical Kp data).
//
// No position feedback yet, so there is no term that can reinforce a sign
// error into a runaway -- cheapest closed-loop confidence available, same
// reasoning as edge-bringup's Stage 2 and the panel's before it.
//
// This file also introduces the full safety scaffold (latching-by-cycle
// disarm on tilt/rate/NaN) -- everything from here onward in this folder
// runs with it active, same shape as edge-bringup's progression. Unlike
// edge, there is a SINGLE gArmed for the whole corner controller (all
// three wheels arm/disarm together) -- it doesn't make sense to arm one
// wheel's contribution to a coupled 3-wheel law and not the others'.
//
// STAGE 2 CHECKLIST (~20 min) — cube held by hand, resting on the SAME
// corner Stage 1 identified.
//   Send "a1" to arm (gGainScale fixed at 1.0 this stage).
//   [ ] The corner feels VISCOUS in every direction you tip it -- resists
//       rotation about ALL THREE axes, no tendency to hold a position yet.
//   [ ] If any direction fights back the WRONG way (feels like it's
//       helping the fall rather than resisting it), STOP -- a sign is
//       wrong somewhere (wheel sign or Kp row/column order). Go back to
//       Stage 1, do not proceed to Stage 3.
//   [ ] Park the cube where it ACTUALLY balances, let phi settle, then send
//       "oc" to capture that attitude as the phi offset. Write the printed
//       number down -- it is the measured trim to carry into Stage 3/4.
//
// TWO CHANGES MADE AT THIS STAGE, both bench-driven:
//   1. kAxisWheelSign is ALL +1.0f, in every stage of this folder. The bench
//      caught X (moteus id 1) spinning the wrong way; the eigenvalue sweep
//      that followed showed the same flip was needed on Y and Z, and that
//      (+1,+1,+1) is the ONLY stable choice of the eight. See the long note
//      at that array -- including why Stage 1's pulse check cannot decide it.
//   2. The phi offset is live-settable from the serial console ("o" family)
//      rather than the compile-time constant Stage4_FixedOffset.ino uses.
//      It does not affect torque in this stage -- the rate-only law masks the
//      phi columns of Kp to zero -- it moves reported phi and the tilt trip so
//      the trim can be MEASURED here before any stage acts on it.
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
// Same semantics as every panel/edge stage from Stage 2 onward: halting
// force-disarms (strict superset of "a0"), resuming does NOT re-arm.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from Stage 1)
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
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
    Serial.println("# WARNING: best and runner-up corners are close -- verify before arming.");
  }
}

float phi[3] = { 0.0f, 0.0f, 0.0f };        // reported tilt = raw + offset
static float gPhiRaw[3] = { 0.0f, 0.0f, 0.0f };   // before the offset is applied

// ---------------------------- PHI OFFSET ------------------------------------
// Same quantity as Stage4_FixedOffset.ino's kPhiOffset, but LIVE-SETTABLE here
// (see the "o" command) instead of a compile-time constant. It shifts where the
// controller believes "upright" is, which is how a COM that does not sit exactly
// over the contact corner gets trimmed out.
//
// WHAT IT DOES AND DOES NOT DO IN *THIS* STAGE: the rate-only law masks the phi
// columns of Kp to zero, so the offset does NOT change commanded torque here.
// It changes the reported phi and therefore the norm3(phi) > kMaxTilt disarm
// check. That is the point at this stage -- park the cube where it actually
// balances, read the steady phi, and capture it with "oc" so you carry a
// measured number into Stage 3/4 rather than guessing one.
//
// Clamped per component to kPhiOffsetMax: a trim is a fraction of a degree in
// practice, and a fat-fingered large value would otherwise move the tilt trip
// far enough to defeat it.
static const float kPhiOffsetMax = 0.261799388f;    // rad, 15 deg -- same clamp
                                                     // the other stages use, so a
                                                     // trim measured here carries
                                                     // forward without being
                                                     // silently re-clipped
static float gPhiOffset[3] = { 0.0f, 0.0f, 0.0f };  // rad

// The offset is PROJECTED onto the plane perpendicular to gB before it is
// stored. phi = -(gB x ghat) satisfies phi . gB == 0 identically, so a raw phi
// IS always perpendicular to gB; an offset with a gB component would push the
// sum off that plane and inject a tilt the estimator can never produce or
// correct -- a phantom yaw error fed straight into the phi columns of Kp from
// Stage 3 onward. Only the perpendicular part is a real trim.
static void setPhiOffsetRad(float x, float y, float z) {
  const float raw[3] = {
    isfinite(x) ? x : 0.0f,
    isfinite(y) ? y : 0.0f,
    isfinite(z) ? z : 0.0f,
  };
  const float* gB = gActiveGB;
  const float n2 = dot3(gB, gB);
  const float k  = (n2 > 1e-12f) ? (dot3(raw, gB) / n2) : 0.0f;
  for (int i = 0; i < 3; ++i) {
    float v = raw[i] - k * gB[i];
    v = v >  kPhiOffsetMax ?  kPhiOffsetMax : v;
    v = v < -kPhiOffsetMax ? -kPhiOffsetMax : v;
    gPhiOffset[i] = v;
  }
}

static void printPhiOffset(const char* prefix) {
  Serial.print(prefix);
  Serial.print(gPhiOffset[0] * (float)RAD_TO_DEG, 4); Serial.print(", ");
  Serial.print(gPhiOffset[1] * (float)RAD_TO_DEG, 4); Serial.print(", ");
  Serial.print(gPhiOffset[2] * (float)RAD_TO_DEG, 4);
  Serial.println(" deg");
}

// gB := ghat, so phi = -(gB x ghat) = 0 exactly at this pose.
static void captureEquilibrium() {
  const float* tab = kCorners[gCornerIdx].gB;
  const float old[3] = { gActiveGB[0], gActiveGB[1], gActiveGB[2] };

  gActiveGB[0] = ghat[0]; gActiveGB[1] = ghat[1]; gActiveGB[2] = ghat[2];
  normalize3(gActiveGB);
  gGBFromBench = true;
  gPhiOffset[0] = gPhiOffset[1] = gPhiOffset[2] = 0.0f;   // stale against the new gB

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
  gPhiRaw[0] = -t[0]; gPhiRaw[1] = -t[1]; gPhiRaw[2] = -t[2];
  phi[0] = gPhiRaw[0] + gPhiOffset[0];
  phi[1] = gPhiRaw[1] + gPhiOffset[1];
  phi[2] = gPhiRaw[2] + gPhiOffset[2];
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------

void printState(uint32_t t_ms, const float rho[3], const float tau[3],
                const float tauCmd[3], bool armed, float gainScale) {
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
  Serial.print('\t'); Serial.print(tau[0], 4);
  Serial.print('\t'); Serial.print(tau[1], 4);
  Serial.print('\t'); Serial.print(tau[2], 4);
  Serial.print('\t'); Serial.print(tauCmd[0], 4);
  Serial.print('\t'); Serial.print(tauCmd[1], 4);
  Serial.print('\t'); Serial.print(tauCmd[2], 4);
  Serial.print('\t'); Serial.print(armed ? 1 : 0);
  Serial.print('\t'); Serial.print(gainScale, 2);
  Serial.print('\t'); Serial.print(gPhiOffset[0] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(gPhiOffset[1] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.println(gPhiOffset[2] * (float)RAD_TO_DEG, 4);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL — om block only (phi and rho masked to zero)
// ----------------------------------------------------------------------------

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // fixed this stage -- single term, no ramp needed

static const float kMaxTilt    = 0.6f;    // rad, 34.4 deg -- wide on purpose this
                                             // stage, mirrors edge-bringup's Stage 2
                                             // (needs to clear real stiction at low
                                             // authority). Compared against norm3(phi)
                                             // now, not a scalar. NOT the Stage 4/5 value.
static const float kMaxOmega   = 40.0f;    // rad/s, OMEGA_CAP from cubli_gains.h
static const float kTauMax     = 0.12f;    // N*m, TAU_MAX from cubli_gains.h
static const float kTaperStart = 36.0f;    // rad/s, 90% of cap -- same taper edge-
                                             // bringup used throughout (including its
                                             // Stage 5 "real policy"), in place of
                                             // cubli_gains.h's pseudocode hard cutoff:
                                             // smoother, already hardware-validated,
                                             // asymptotically the same protection.

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

static float gLastTau[3]    = { 0.0f, 0.0f, 0.0f };
static float gLastTauCmd[3] = { 0.0f, 0.0f, 0.0f };

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
// The om block is supposed to inherit that null, and at the corner the gains
// were actually derived for it very nearly does. At THIS rig's corner it does
// NOT, because the Kp magnitudes are inherited from a same-sign corner (see
// the table header): K_om's null direction misses gB by ~4.6 deg here, leaving
//   |K_om . gB_hat| = 0.0758  ~= 10% of a full row
// of torque that pushes on yaw instead of ignoring it. Measured per corner:
//   [-1,-1,-1] 0.3 deg off, 0.7% residual     <- derived corner, fine
//   [+1,+1,+1] 1.4 deg off, 3.0% residual     <- derived corner, fine
//   the six MIXED corners: 4.5-5.9 deg off, 9.6-12.6% residual
//
// Projecting the rate onto the plane perpendicular to gB removes it exactly
// (|K_om_proj . gB| ~ 1e-17) and leaves the two real modes at -0.937/-0.93.
// Toggle with "y0"/"y1" so the difference can be felt on the bench.
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
  // x = [phi(3); om(3); rho(3)] -- phi and rho masked to zero this stage,
  // only the om block (columns 3-5) of each wheel's Kp row contributes.
  // The om entries are the yaw-projected body rate, not w_b raw (see above);
  // w_b itself is still reported unprojected in telemetry.
  float omUsed[3];
  projectOutYaw(w_b, omUsed);
  const float xVec[9] = {
    0.0f, 0.0f, 0.0f,
    omUsed[0], omUsed[1], omUsed[2],
    0.0f, 0.0f, 0.0f,
  };

  float tau[3];
  for (int i = 0; i < 3; ++i) {
    const float* row = kCorners[gCornerIdx].Kp[i];
    float u = 0.0f;
    for (int j = 0; j < 9; ++j) { u -= row[j] * xVec[j]; }
    u *= gGainScale;

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

    gLastTau[i]    = tau[i];
    gLastTauCmd[i] = tau_cmd;
  }
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
    gArmed = (val != 0.0f);
    Serial.print("# gArmed = "); Serial.println(gArmed ? "TRUE" : "FALSE");
  } else if (cmd == 'c') {
    resolveCornerCandidate();
  } else if (cmd == 'o') {
    // o              -- report the current offset
    // oc             -- CAPTURE: set offset = -phi_raw, so phi reads ~0 right now
    // oz             -- zero it
    // o<x> <y> <z>   -- set all three explicitly, in DEGREES (comma or space
    //                   separated, e.g. "o0.42 -0.15 0.08")
    String arg = line.substring(1);
    arg.trim();
    if (arg.length() == 0) {
      printPhiOffset("# phi offset = ");
    } else if (arg.charAt(0) == 'c') {
      setPhiOffsetRad(-gPhiRaw[0], -gPhiRaw[1], -gPhiRaw[2]);
      printPhiOffset("# phi offset CAPTURED from current attitude = ");
      Serial.println("# hold the cube where it actually balances before capturing");
    } else if (arg.charAt(0) == 'z') {
      setPhiOffsetRad(0.0f, 0.0f, 0.0f);
      printPhiOffset("# phi offset ZEROED = ");
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
        setPhiOffsetRad(v[0], v[1], v[2]);
        printPhiOffset("# phi offset = ");
      }
    }
  } else if (cmd == 'y') {
    gYawProject = (val != 0.0f);
    Serial.print("# gYawProject = ");
    Serial.println(gYawProject ? "TRUE (rate projected onto the plane perp to gB)"
                                : "FALSE (raw w_b into Kp -- yaw gets ~10% of a row here)");
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
    Serial.println("# unknown. use: a<0/1>  c (re-resolve corner)  h<0/1>  "
                    "o / oc / oz / o<x> <y> <z> (phi offset, deg)  "
                    "y<0/1> (yaw projection)");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 2: RATE ONLY (cube held by hand)");

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

  if (imu.setAccelODR(BMI2_ACC_ODR_400HZ) != BMI2_OK ||
      imu.setGyroODR(BMI2_GYR_ODR_400HZ)  != BMI2_OK) {
    Serial.println("Warning: could not raise BMI270 ODR to 400 Hz");
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
                  "tau_x\ttau_y\ttau_z\ttau_cmd_x\ttau_cmd_y\ttau_cmd_z\t"
                  "armed\tgain_scale\toff_x_deg\toff_y_deg\toff_z_deg");
  Serial.println("# STARTS DISARMED. Send a1 to arm, a0 to disarm.");
  Serial.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");
  Serial.println("# y0/y1 toggles the yaw projection (default ON). With it OFF,");
  Serial.println("#   K_om leaks ~10% of a row of torque onto the balancing axis");
  Serial.println("#   at this corner -- the inherited-magnitude Kp's null misses");
  Serial.println("#   gB by ~4.6 deg. Try both and feel the yaw difference.");
  Serial.println("# NOTE: rho is masked this stage, so NOTHING regulates wheel");
  Serial.println("#   speed. Any residual gyro bias becomes a CONSTANT torque and");
  Serial.println("#   ramps rho linearly to the cap in seconds. That is structural");
  Serial.println("#   to a rate-only law, not a fault -- keep runs short, and");
  Serial.println("#   expect it to stop once the rho columns come in at Stage 3+.");
  Serial.println("# phi offset: o reports, oc captures the current attitude as");
  Serial.println("#   zero, oz zeroes, o<x> <y> <z> sets it in deg. Does NOT");
  Serial.println("#   change torque this stage (phi is masked out of the law) --");
  Serial.println("#   it moves reported phi and the tilt trip, so you can measure");
  Serial.println("#   the trim here and carry it into Stage 3/4.");
  printPhiOffset("# phi offset = ");

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

  const float rho[3] = {
    kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI,
  };

  commandWheels(rho);

  printState(time, rho, gLastTau, gLastTauCmd, gArmed, gGainScale);
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: Stage3_PositionDamping/Stage3_PositionDamping.ino -- adds the phi
// block (columns 0-2) on top of om, gGainScale ramped by hand across runs
// (0.1 -> 0.3 -> 0.6 -> 1.0), same shape as edge-bringup's Stage 3. The rho
// block (columns 6-8, momentum management) still waits for Stage 4.
// ============================================================================
