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

static const CornerCandidate kCorners[8] = {
  { "[-1,-1,-1]"  // lean 0.797 deg vs body diagonal, ell 122.84 mm, Sg 1.8875, lambda 8.2572
    , { -0.571549475f, -0.588645577f, -0.571688354f }
    , { { -6.9157443f, 3.39226651f, 3.42117763f, -0.834802926f, 0.420730621f, 0.42704013f, -0.000700236589f, 0.00623514736f, 0.00621826435f },  // wheel X
        { 3.51941299f, -6.8296299f, 3.51364994f, 0.412131369f, -0.848057508f, 0.411711216f, 0.00382628222f, -0.00316504063f, 0.00381609239f },  // wheel Y
        { 3.42913675f, 3.39467001f, -6.92366505f, 0.426502436f, 0.419758797f, -0.837698817f, 0.00606757682f, 0.00607034843f, -0.000870343472f } } // wheel Z
    , 0.797f },
  { "[-1,-1,+1]"  // lean 3.170 deg vs body diagonal, ell 128.54 mm, Sg 1.9750, lambda 8.1212
    , { -0.546220303f, -0.562558711f, 0.620621562f }
    , { { -7.58488846f, 3.33574939f, -3.65192771f, -0.921932399f, 0.430031687f, -0.480414748f, 0.000473975349f, 0.00748626003f, -0.00814931281f },  // wheel X
        { 3.43748641f, -7.47500134f, -3.7502768f, 0.421860248f, -0.928083122f, -0.467502952f, 0.00511831883f, -0.00179392833f, -0.00555475708f },  // wheel Y
        { -3.84645557f, -3.83494377f, -6.8614974f, -0.464526713f, -0.460435808f, -0.881195128f, -0.00357620185f, -0.00353527209f, -0.00314604398f } } // wheel Z
    , 3.170f },
  { "[-1,+1,-1]"  // lean 2.773 deg vs body diagonal, ell 126.08 mm, Sg 1.9373, lambda 8.1591
    , { -0.556852818f, 0.616181135f, -0.55698812f }
    , { { -7.26747227f, -3.47609496f, 3.4201951f, -0.874630272f, -0.451983064f, 0.442295492f, 0.000827456824f, -0.00813719723f, 0.00776857371f },  // wheel X
        { -3.79863954f, -6.87362671f, -3.8063941f, -0.426038593f, -0.899968863f, -0.426554501f, -0.000622705789f, -0.00668813102f, -0.000633999531f },  // wheel Y
        { 3.41229653f, -3.47500682f, -7.25577497f, 0.442907125f, -0.45318532f, -0.871017814f, 0.00794851314f, -0.0083494084f, 0.00103329937f } } // wheel Z
    , 2.773f },
  { "[-1,+1,+1]"  // lean 3.097 deg vs body diagonal, ell 131.64 mm, Sg 2.0226, lambda 8.0480
    , { -0.533349514f, 0.590173781f, 0.605997622f }
    , { { -8.23398495f, -3.56334782f, -3.77657819f, -1.06839609f, -0.416523129f, -0.442939252f, -0.00582132814f, -0.00140620384f, -0.00158187468f },  // wheel X
        { -3.41282964f, -7.33259392f, 4.13743114f, -0.429089457f, -0.917372108f, 0.514714897f, -0.00497917458f, -0.00120868487f, 0.00603242731f },  // wheel Y
        { -3.50851822f, 4.01196194f, -6.99511194f, -0.465059072f, 0.524895251f, -0.847355127f, -0.00789372995f, 0.00895721f, 0.00243827817f } } // wheel Z
    , 3.097f },
  { "[+1,-1,-1]"  // lean 3.171 deg vs body diagonal, ell 128.56 mm, Sg 1.9753, lambda 8.1180
    , { 0.620658159f, -0.562471628f, -0.546268404f }
    , { { -6.85864925f, -3.8295908f, -3.8494637f, -0.880151272f, -0.460793495f, -0.465634525f, -0.00304374471f, -0.00361368596f, -0.00366454106f },  // wheel X
        { -3.75562048f, -7.4855423f, 3.44052219f, -0.467026591f, -0.931139171f, 0.421110034f, -0.00539576123f, -0.00195194408f, 0.0049761422f },  // wheel Y
        { -3.65655398f, 3.33088136f, -7.58417654f, -0.481462985f, 0.429957539f, -0.921488523f, -0.00823030341f, 0.00754544372f, 0.000543692615f } } // wheel Z
    , 3.171f },
  { "[+1,-1,+1]"  // lean 2.609 deg vs body diagonal, ell 134.01 mm, Sg 2.0591, lambda 7.9804
    , { 0.595400333f, -0.539581716f, 0.595273018f }
    , { { -7.30901575f, -3.4263885f, 4.20474958f, -0.895961821f, -0.451509207f, 0.553766072f, 0.00180089788f, -0.00741348462f, 0.00873730052f },  // wheel X
        { -3.81644511f, -8.42617893f, -3.82059836f, -0.41949293f, -1.11799276f, -0.419535488f, 0.00164802792f, -0.00882489327f, 0.00163950643f },  // wheel Y
        { 4.19695139f, -3.42353535f, -7.30109262f, 0.554382324f, -0.452106625f, -0.892860353f, 0.00891439989f, -0.00758054852f, 0.00199437374f } } // wheel Z
    , 2.609f },
  { "[+1,+1,-1]"  // lean 3.095 deg vs body diagonal, ell 131.66 mm, Sg 2.0229, lambda 8.0501
    , { 0.606037736f, 0.590086639f, -0.533400357f }
    , { { -6.99898767f, 4.01700401f, -3.50819087f, -0.848652959f, 0.524742544f, -0.464503765f, 0.00232795905f, 0.00885933265f, -0.00779828522f },  // wheel X
        { 4.13386154f, -7.32801533f, -3.40998602f, 0.515303075f, -0.915354431f, -0.429458082f, 0.00615369575f, -0.00108131079f, -0.00508674001f },  // wheel Y
        { -3.77546f, -3.56726503f, -8.23596478f, -0.442555219f, -0.416409701f, -1.06887817f, -0.00154884392f, -0.00138305803f, -0.00584927434f } } // wheel Z
    , 3.095f },
  { "[+1,+1,+1]"  // lean 0.714 deg vs body diagonal, ell 136.99 mm, Sg 2.1048, lambda 7.9341
    , { 0.582457066f, 0.567126632f, 0.582332492f }
    , { { -7.76383209f, 3.81748652f, 4.04768848f, -0.975789487f, 0.490672857f, 0.522951066f, -0.000524852891f, 0.00608501304f, 0.00639130361f },  // wheel X
        { 3.93991017f, -8.08397388f, 3.93213129f, 0.481777757f, -1.03972197f, 0.481087863f, 0.00368249253f, -0.00348382187f, 0.00367164146f },  // wheel Y
        { 4.05641413f, 3.81822968f, -7.77580976f, 0.522317231f, 0.489363909f, -0.97946316f, 0.00622009346f, 0.00590694463f, -0.000720091164f } } // wheel Z
    , 0.714f },
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
    Serial.println("# WARNING: best and runner-up corners are close -- cube may not");
    Serial.println("#   be resting stably on a single corner yet. Recheck before Stage 2.");
  }
}

float phi[3] = { 0.0f, 0.0f, 0.0f };
// om is just w_b, no separate variable needed -- read directly at use.
float rho[3] = { 0.0f, 0.0f, 0.0f };

void updateCornerProjection() {
  float t[3];
  cross3(kCorners[gCornerIdx].gB, ghat, t);
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

// Re-verified on THIS rig via this stage's own pulse checklist (2026-08-21),
// NOT carried forward from edge-bringup (which had all three +1.0f). All
// three entries are -1.0f here, matching this stage as flashed: a positive
// commanded torque drove rho negative on every wheel. (An older version of
// this note claimed only Y was inverted -- that prose disagreed with the
// array right below it; the array is what ran on the bench, and it is what
// stages 2-5 now carry.) Fix wiring-convention sign problems HERE, NEVER by
// flipping a sign inside Stage 2's Kp -- Kp comes verbatim from
// cubli_gains.h (Firmware Lessons S4).
static const float kAxisWheelSign[3] = {
  -1.0f,   // X -- CONFIRMED (this rig, id 1)
  -1.0f,   // Y -- CONFIRMED (this rig, id 3)
  -1.0f,   // Z -- CONFIRMED (this rig, id 2)
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
