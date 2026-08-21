// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 4: FULL LAW + FIXED OFFSET (no live trim, no arm gate)
// ============================================================================
// Copy of Stage4_AutoTrim.ino with TWO mechanisms removed: (1) the live-
// adapting gTrim is replaced by a compile-time constant kPhiOffset, and
// (2) the arm gate (kArmGate, refusing "a1" unless already within 0.5 deg
// of equilibrium) is gone -- "a1" always arms now. Read Stage4_AutoTrim
// .ino's header first for why automatic trim exists and how it works --
// this file doesn't re-explain that mechanism, it retires it in favor of
// a known-good number.
//
// ---------------------------- WHAT AND WHY ---------------------------------
// kPhiOffset below (0.2624, 0.2265, -0.4830 deg) is NOT a guess or a
// snapshot -- it's the mean of Automatic Trim's own converged gTrim over
// a 132.41 s continuous armed hold (perfect_equilibrium_2.log, 2026-08-20),
// with per-axis std < 0.0012 deg over that whole window. That's the
// signature of a genuine fixed point, not a value still drifting: whatever
// residual COM/mounting error automatic trim was correcting for, it found
// it and stopped moving. Once a corner's offset is known this precisely,
// there's no more reason to re-derive it live every session -- hardcode
// it and remove the adaptation machinery (gTrim, gKAdapt, updateTrim(),
// applyTrimGuards(), the z/x/k serial commands) that existed to find it.
//
// If the cube's mounting changes (battery moved, a cable re-routed, a
// bolt re-torqued) this number goes stale -- that's the tradeoff for
// dropping live adaptation. Re-run Stage4_AutoTrim.ino, let it re-converge,
// and update kPhiOffset from the new result rather than assuming this one
// still holds.
//
// ---------------------------- ARM GATE REMOVED ------------------------------
// kArmGate (0.5 deg) is GONE from the 'a' command below -- "a1" arms
// unconditionally now, regardless of the cube's current tilt. This is a
// REAL SAFETY TRADEOFF, not just a convenience change, worth being
// explicit about:
//
// The Kp control law was derived by linearizing around this corner's
// equilibrium and is only VALIDATED for small angles (the simulation
// study's own recovery envelope tops out around 2.7-3.1 deg worst case,
// see Cube-Performance-Envelope-Results.md). Arming from a large initial
// tilt commands full-authority torque from OUTSIDE that validated
// envelope -- the arm gate existed specifically to prevent that. With it
// gone, the ONLY remaining backstop is kMaxTilt (25 deg, unchanged,
// still an automatic disarm-on-trip below in commandWheels()) -- which
// stops a runaway AFTER it's already happened, not before it starts.
//
// Removing the gate does NOT change what "close to the corner" means or
// make it safe to arm from far away -- it just stops the firmware from
// enforcing it. Placing the cube near the resolved equilibrium before
// sending "a1" is now an OPERATOR DISCIPLINE requirement instead of a
// code-enforced one. Physical stop/catch and a hand on the e-stop path
// matter more here than in Stage4_AutoTrim.ino, not less.
//
// STAGE 4 CHECKLIST — cube held by hand, gGainScale = 1.0:
//   Still place near the resolved corner's equilibrium before "a1" (see
//   the ARM GATE REMOVED note above -- this is now on you, not the code).
//   [ ] phi_x/y/z_deg in telemetry should already read close to zero at
//       rest, with no trim column doing any work -- that's the point of
//       hardcoding a validated offset instead of waiting for adaptation.
//   [ ] If phi at rest looks meaningfully different from what
//       perfect_equilibrium_2.log showed, something about the corner's
//       mounting has changed -- see the WHAT AND WHY note above, this
//       file's offset needs re-deriving, not trusting blindly.
//
// Velocity cap loosened (kMaxOmega, not literally removed) same as
// Stage4_AutoTrim.ino -- see that file's header for the full reasoning.
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
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };

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
  float ellM;       // m, contact-to-COM lever arm -- NEW vs Stage4_FullLaw.ino,
                     // used only to convert a converged trim into an
                     // equivalent COM offset in mm for telemetry (Automatic
                     // Trim.md S4's "log trim continuously" guard).
};

// TODO: corner [-1,-1,-1]'s Kp below was UPDATED 2026-08-19 for the new
// plant (mass 1.633 kg + corner-to-housing strut); the other 7 corners are
// still the old 1.5668 kg/no-strut values -- do not treat this table as
// internally consistent across corners until they're all re-derived.
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

// FIXED OFFSET -- see the header's WHAT AND WHY note for provenance
// (perfect_equilibrium_2.log, mean over 132.41s, std<0.0012 deg/axis).
// ADDED to the raw measurement below, same sign/convention Stage4_AutoTrim
// .ino's gTrim used (verified on the bench, cos(trim,wheels)=-1.0000 --
// see that file's header) -- this is that same verified value, just no
// longer recomputed live.
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
  // Same 5 columns Stage4_AutoTrim.ino's telemetry has, kept for format
  // compatibility with the existing analysis tools (plot_session_csv.py
  // etc. recognize this exact 26-column shape) -- but these are now
  // CONSTANTS, not a live readout. trim_enabled=0 always, signaling
  // "fixed, not adapting" to anyone comparing logs across files.
  Serial.print('\t'); Serial.print(kPhiOffset[0] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(kPhiOffset[1] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(kPhiOffset[2] * (float)RAD_TO_DEG, 4);
  Serial.print('\t'); Serial.print(norm3(kPhiOffset) * kCorners[gCornerIdx].ellM * 1000.0f, 3);
  Serial.print('\t'); Serial.println(0);
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

// NO ARM GATE HERE -- see the header's "ARM GATE REMOVED" note. kArmGate
// existed in Stage4_AutoTrim.ino to refuse "a1" unless already near the
// resolved corner's equilibrium; that check is gone from the 'a' command
// below, deliberately, per the request that built this file.

// Re-verified on THIS rig via Stage 1's own pulse checklist (2026-08-21),
// NOT carried forward from edge-bringup (which had all three +1.0f). All
// three entries are -1.0f here, matching Stage 1 as flashed: a positive
// commanded torque drove rho negative on every wheel. (Stage 1's older prose
// claimed only Y was inverted -- that note disagreed with its own array; the
// array is what ran on the bench, and it is what is carried here.)
// Fix wiring-convention sign problems HERE, NEVER by flipping a sign inside
// Kp -- Kp comes verbatim from cubli_gains.h (Firmware Lessons S4).
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

static float gLastTau[3]      = { 0.0f, 0.0f, 0.0f };
static float gLastTauCmd[3]   = { 0.0f, 0.0f, 0.0f };
static float gRhoLp[3]        = { 0.0f, 0.0f, 0.0f };   // standing speed, tau = 5 s

Moteus& wheelObj(int i) {
  return i == 0 ? moteusX : (i == 1 ? moteusY : moteusZ);
}

void commandWheels(const float rho[3]) {
  // x = [phi(3); om(3); rho(3)] -- full state, all nine columns of each
  // wheel's Kp row now contribute.
  const float xVec[9] = {
    phi[0], phi[1], phi[2],
    w_b[0], w_b[1], w_b[2],
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
// WHAT AND WHY note. gTrim/gKAdapt/updateTrim()/applyTrimGuards() are gone;
// kPhiOffset above is a compile-time replacement, not a live-adapting one.


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
    // NO ARM GATE -- see the header's "ARM GATE REMOVED" note. Arms
    // unconditionally on "a1", regardless of current tilt.
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
                    "g<0..1> (gain scale)  c (re-resolve)  h<0/1> (halt)");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 4: FULL LAW + FIXED OFFSET, NO ARM GATE (cube held by hand)");

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
                  "trim_x_deg\ttrim_y_deg\ttrim_z_deg\ttrim_com_mm\ttrim_enabled");
  Serial.println("# STARTS DISARMED. NO ARM GATE -- a1 arms unconditionally,");
  Serial.println("# regardless of current tilt. Get close to the resolved corner");
  Serial.println("# before arming anyway -- see header 'ARM GATE REMOVED' note.");
  Serial.print("# Fixed offset (not adapting): "); Serial.print(kPhiOffset[0]*(float)RAD_TO_DEG, 4);
  Serial.print(", "); Serial.print(kPhiOffset[1]*(float)RAD_TO_DEG, 4);
  Serial.print(", "); Serial.print(kPhiOffset[2]*(float)RAD_TO_DEG, 4);
  Serial.println(" deg (from perfect_equilibrium_2.log).");
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
// This file vs Stage4_AutoTrim.ino: identical control law, identical corner
// candidate table -- the difference is entirely in how phi's offset is
// sourced (compile-time constant here vs live adaptation there) and that
// the arm gate is gone. If something looks wrong here that isn't about
// either of those two things, check whether Stage4_AutoTrim.ino has the
// same problem before assuming this file's changes caused it.
//
// This file exists because Automatic Trim already did its job and found a
// stable, precise answer (perfect_equilibrium_2.log) -- it is a
// consequence of trust in that mechanism, not a replacement for it.
// Stage4_AutoTrim.ino remains the way to (re-)derive kPhiOffset if the
// corner's mounting ever changes; this file is for running with an
// already-known-good value without paying the adaptation/convergence time
// every session, and without the arm gate blocking arming while iterating.
//
// Next steps:
//   - Port the same two changes (fixed offset + no arm gate) to the rate-
//     filter variant -- see Stage4_FixedOffset_RateFilter.ino.
//   - If this offset is trusted across multiple sessions/days, it's a
//     candidate for promoting into cubli_gains.h's per-corner table
//     directly (as a per-corner phi bias) rather than living only here.
//   - Re-derive kPhiOffset (re-run Stage4_AutoTrim.ino, let it converge,
//     take the new mean) after ANY mechanical change to this corner --
//     battery moved, cable re-routed, bolt re-torqued. This file has no
//     way to detect that its offset has gone stale on its own.
// ============================================================================
