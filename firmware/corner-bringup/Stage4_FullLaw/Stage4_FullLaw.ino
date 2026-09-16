// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 4: FULL LAW (phi + om + rho, friction FF, arm gate)
// ============================================================================
// CUBE STILL HELD BY HAND. Adds the rho block (columns 6-8, momentum
// management) on top of phi+om from Stage 3, PLUS friction feedforward per
// wheel (Firmware Lessons: "not optional" -- without it the wheels drift
// and permanently burn budget cancelling drag) and the arm gate from
// cubli_gains.h's contract: hold torque at zero until norm3(phi) <
// ARM_GATE, then engage. Wheels should now UNWIND after each correction
// instead of spinning up steadily the way they did in Stage 3.
//
// STAGE 4 CHECKLIST — cube held by hand, gGainScale = 1.0:
//   Position near the resolved corner's equilibrium BEFORE sending "a1" --
//   if norm3(phi) doesn't settle under ARM_GATE (0.5 deg) even holding the
//   cube still at its natural rest point, that's the COM offset described
//   below, not a bad hold -- tare it with "z1" first, then arm.
//   [ ] All three wheels unwind after each correction (watch the standing-
//       speed low-pass columns -- rho through a ~5s low-pass).
//   [ ] Standing speed near zero on all three -> healthy.
//   [ ] Standing speed nonzero and not settling on ONE wheel -> re-check
//       that wheel's gyro bias/sign before suspecting the coupled law.
//
// TWO ADDITIONS since the first pass at this stage, both requested for this
// specific build (still on the bench power supply, not the flight battery
// -- same root cause edge-bringup hit on the Y edge, generalized to 3D):
//   - "z1" tares gPhiOffset to the CURRENTLY measured phi (z0 clears it back
//     to zero). Without the battery/DC-DC mounted, this build's real corner
//     equilibrium sits measurably off cubli_gains.h's design gB -- more than
//     ARM_GATE's 0.5 deg -- so the gate was refusing to ever arm against an
//     equilibrium that doesn't physically exist yet with this mass
//     distribution. This is a real physical offset, not a sensor
//     calibration, same distinction edge-bringup's kPhiOffset drew -- but
//     phi here is a 3-vector, so instead of typing three degree values by
//     hand (error-prone, slow to iterate), "z1" captures whatever phi reads
//     RIGHT NOW as the new zero. Rest the cube at its natural, unforced
//     balance point, tare, then arm. Re-tare (or "z0") once the missing
//     mass is mounted and the real equilibrium moves back toward design.
//   - Velocity cap REMOVED (kMaxOmega effectively infinite), same as edge-
//     bringup's Stage 4 and for the same reason: fighting a genuine COM
//     offset needs sustained corrective torque the taper was cutting off
//     before the wheels built up enough momentum to actually win. TAU_MAX
//     (the real physical torque saturation) is untouched -- this only
//     affects the momentum-taper policy. Cube is hand-held and arming is an
//     explicit choice to disarm ("a0") if it runs away -- that IS the
//     safety mechanism now, not this constant. Do NOT carry this into
//     Stage 5: that stage keeps the real 40 rad/s OMEGA_CAP because it runs
//     unsupported.
//
// Trips here are still LOOSE (hand-held, watching combined-term behavior)
// -- Stage 5 tightens to the real DISARM/OMEGA_CAP policy from cubli_gains.h.
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

float phi[3] = { 0.0f, 0.0f, 0.0f };

// Live-settable via "z1" (tare) / "z0" (clear) -- see the header note above
// for the full reasoning. Subtracted from the raw measurement below, so
// EVERYWHERE phi is used from here down (control, arm gate, trips,
// telemetry) already sees the corrected value, same convention edge-
// bringup's kPhiOffset used.
static float gPhiOffset[3] = { 0.0f, 0.0f, 0.0f };

// Guards for the live phi offset: project onto the plane perpendicular to gB,
// then clamp. phi = -(gB x ghat) satisfies phi . gB == 0 identically, so an
// offset with a gB component would push the sum off that plane and inject a
// tilt the estimator can never produce or correct. The clamp stops a
// fat-fingered value from moving the tilt trip far enough to defeat it.
static const float kOffsetMax = 0.261799388f;   // rad, 15 deg
void applyOffsetGuards() {
  const float* gB = gActiveGB;
  const float n2 = dot3(gB, gB);
  if (n2 > 1e-12f) {
    const float along = dot3(gPhiOffset, gB) / n2;
    gPhiOffset[0] -= along * gB[0];
    gPhiOffset[1] -= along * gB[1];
    gPhiOffset[2] -= along * gB[2];
  }
  for (int i = 0; i < 3; ++i) {
    if (!isfinite(gPhiOffset[i])) { gPhiOffset[i] = 0.0f; }
  }
  const float n = norm3(gPhiOffset);
  if (n > kOffsetMax) {
    const float s = kOffsetMax / n;
    gPhiOffset[0] *= s; gPhiOffset[1] *= s; gPhiOffset[2] *= s;
  }
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
  phi[0] = -t[0] - gPhiOffset[0];
  phi[1] = -t[1] - gPhiOffset[1];
  phi[2] = -t[2] - gPhiOffset[2];
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------

static void printPhiOffset(const char* prefix) {
  Serial.print(prefix);
  Serial.print(gPhiOffset[0] * (float)RAD_TO_DEG, 4); Serial.print(", ");
  Serial.print(gPhiOffset[1] * (float)RAD_TO_DEG, 4); Serial.print(", ");
  Serial.print(gPhiOffset[2] * (float)RAD_TO_DEG, 4);
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
  Serial.print('\t'); Serial.println(gainScale, 2);
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
  // wheel's Kp row now contribute.
  // phi and om are yaw-projected before Kp (see projectOutYaw above);
  // telemetry still reports the unprojected values.
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
      gPhiOffset[0] += phi[0];
      gPhiOffset[1] += phi[1];
      gPhiOffset[2] += phi[2];
      Serial.print("# gPhiOffset TARED to: ");
      Serial.print(gPhiOffset[0] * (float)RAD_TO_DEG, 3); Serial.print(",");
      Serial.print(gPhiOffset[1] * (float)RAD_TO_DEG, 3); Serial.print(",");
      Serial.print(gPhiOffset[2] * (float)RAD_TO_DEG, 3); Serial.println(" deg");
      if (norm3(gPhiOffset) > 0.1745329f) {   // ~10 deg
        Serial.println("# NOTE: that's a large offset (>10 deg) -- expected for");
        Serial.println("#   a missing-mass COM shift, but confirm this is really");
        Serial.println("#   the cube resting naturally and not just held crooked.");
      }
    } else {
      gPhiOffset[0] = gPhiOffset[1] = gPhiOffset[2] = 0.0f;
      Serial.println("# gPhiOffset cleared to 0,0,0");
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
    // o<x> <y> <z> -- set gPhiOffset explicitly, in DEGREES (comma or space
    //                 separated). NOTE this file stores the offset with the
    //                 '-' convention (phi = raw - gPhiOffset), so an explicit
    //                 set is in THAT frame; oc/oz/report mean the same thing
    //                 in every stage regardless.
    String arg = line.substring(1);
    arg.trim();
    if (arg.length() == 0) {
      printPhiOffset("# offset = ");
    } else if (arg.charAt(0) == 'c') {
      gPhiOffset[0] += phi[0];
      gPhiOffset[1] += phi[1];
      gPhiOffset[2] += phi[2];
      applyOffsetGuards();
      printPhiOffset("# offset CAPTURED from current attitude = ");
      Serial.println("# hold the cube where it actually balances before capturing");
    } else if (arg.charAt(0) == 'z') {
      gPhiOffset[0] = gPhiOffset[1] = gPhiOffset[2] = 0.0f;
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
        gPhiOffset[0] = v[0]; gPhiOffset[1] = v[1]; gPhiOffset[2] = v[2];
        applyOffsetGuards();
        printPhiOffset("# offset = ");
      }
    }
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
                    "z<0/1> (clear/tare phi offset)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 4: FULL LAW (cube held by hand)");

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
                  "rho_x_lp\trho_y_lp\trho_z_lp\t"
                  "tau_x\ttau_y\ttau_z\ttau_cmd_x\ttau_cmd_y\ttau_cmd_z\t"
                  "armed\tgain_scale");
  Serial.println("# STARTS DISARMED. a1 refused unless norm3(phi) < ARM_GATE");
  Serial.println("# (0.5 deg) -- get close to the resolved corner first. If it");
  Serial.println("# won't settle under 0.5 deg even resting naturally (COM offset");
  Serial.println("# without the battery/DC-DC mounted), send z1 to tare, z0 to clear.");
  Serial.println("# Velocity cap is REMOVED this stage (kMaxOmega effectively inf)");
  Serial.println("# -- a0 (disarm) is the real safety net now, not a speed limit.");
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

  const float rho[3] = {
    kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI,
  };

  commandWheels(rho);

  printState(time, rho, gRhoLp, gLastTau, gLastTauCmd, gArmed, gGainScale);
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: Stage5_Release/Stage5_Release.ino -- control law identical to this
// stage, trip policy tightened to the real cubli_gains.h values (DISARM =
// 15 deg on norm3(phi), OMEGA_CAP = 40 rad/s per wheel -- already what this
// stage uses for kMaxOmega, but Stage 5 also LATCHES the trip reason and
// removes the hand-held assumption: stop/catch + e-stop in hand, cube free.
// ============================================================================
