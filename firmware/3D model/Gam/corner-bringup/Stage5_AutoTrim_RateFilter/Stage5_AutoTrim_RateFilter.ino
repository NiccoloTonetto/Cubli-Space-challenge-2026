// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 5: RELEASE + AUTO TRIM + RATE LOW-PASS FILTER + FINE-TUNING
// ============================================================================
// Copy of Stage5_AutoTrim.ino with the SAME ONE mechanism
// Stage4_AutoTrim_RateFilter.ino added to Stage4_AutoTrim.ino: a first-order
// low-pass on the body-rate signal feeding the control law's om block, per
// hw-run-analysis.md's fix 4.1. Read Stage4_AutoTrim_RateFilter.ino's header
// first for the full rate-filter derivation (the 35 Hz structural-mode
// finding, why only the om-block-into-K2 path is filtered, why the trim
// quiescence gate also switches to the filtered signal) -- this header only
// covers what's different about applying it at Stage 5.
//
// ---------------------------- WHAT AND WHY (condensed) ---------------------
// hw-run-analysis.md's 373.5 s continuous run found torque saturating 71.8%
// of the time, driven by a ~35 Hz structural mode riding on the gyro signal
// that a 1.3 Hz-bandwidth controller can only feed, never damp. The fix:
// low-pass the rate signal (15-25 Hz corner, 20 Hz default) BEFORE it
// multiplies the K2 gain columns in commandWheels() -- a separate w_filt[3],
// leaving raw w_b[3] driving ghat propagation and every trip/safety check
// (arm gate, tilt/omega/NaN trips below) unfiltered and un-delayed. THIS
// MATTERS MORE AT STAGE 5 THAN STAGE 4: the trip checks are the real
// DISARM/OMEGA_CAP policy here, not loose bench values, so w_filt is kept
// out of them just as deliberately as at Stage 4 -- filtering a safety
// check would add exactly the lag you don't want on a signal whose whole
// job is catching a fall fast.
//
// gKAdapt's default is likewise raised from tau_a=60s (2.922e-6) to 1e-4,
// per hw-run-analysis.md 4.5's stability limit (k_a < 0.00214 on the
// 15-state augmented model) -- see Stage4_AutoTrim_RateFilter.ino for the
// derivation. Still live-settable with "k<value>".
//
// UPDATE 2026-08-19: corner [-1,-1,-1]'s Kp/ell/theta_eq below are now the
// re-derived values for the new plant (mass 1.633 kg + strut) -- user-
// supplied. The other seven corners are still the OLD (1.5668 kg, no
// strut) table -- this is a mixed-generation table, not internally
// consistent corner-to-corner -- see the TODO at the table. Multi-corner
// finding (see that entry's own comment / Cube-Performance-Envelope-
// Results.md): this pole leaves only [-1,-1,-1] and [+1,+1,+1] with
// positive recovery margin -- corner balancing is unaffected, multi-
// corner locomotion is not, until it's rebalanced or counterweighted.
//
// PHYSICAL STOP/CATCH IN PLACE. E-STOP IN HAND. This is the stage where the
// cube is NOT held by hand. The control law is identical to Stage 4 (this
// one, with trim and the rate filter); nothing new about the law itself,
// only about the full closed loop running unsupported, on a corner, for
// real.
//
// ---------------------------- WHAT'S NEW HERE ---------------------------
// vs Stage5_AutoTrim.ino:
//   - w_filt[3] (SECTION 2b-2) replaces w_b in commandWheels()'s xVec om
//     block AND in updateTrim()'s quiescence gate. gKAdapt default raised
//     to 1e-4. "f<Hz>" sweeps the filter corner live (15-25 Hz per the
//     source note). See the condensed note above / Stage4_AutoTrim_
//     RateFilter.ino for the full reasoning.
//
// vs Stage5_Release.ino (unchanged from Stage5_AutoTrim.ino, repeated here
// for anyone jumping straight to this file):
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
// Deliberately does NOT touch gTripReason -- halting is an operator pause,
// not a trip, so if a real trip happened before you halted, that record
// stays visible until you actually re-arm. Same as edge-bringup's Stage 5.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from Stage 1-4)
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
// SECTION 2b-2: RATE LOW-PASS FILTER -- hw-run-analysis.md fix 4.1
// ----------------------------------------------------------------------------
// w_b (above) stays raw -- this is a SEPARATE signal, used only in
// commandWheels()'s om block and updateTrim()'s quiescence gate. See the
// header note and Stage4_AutoTrim_RateFilter.ino for the full reasoning on
// why only those two use points are filtered, and why every trip/safety
// check below still reads raw w_b.

// 15-25 Hz per the source note; 20 Hz is the value its own phase-lag
// number (3.7 deg at 1.3 Hz) was computed for. Live-settable with "f<Hz>"
// to sweep the range on the bench without reflashing.
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
  float ellM;       // m, contact-to-COM lever arm -- NEW vs Stage5_Release.ino,
                     // used only to convert a converged trim into an
                     // equivalent COM offset in mm for telemetry.
};

// TODO: MIXED-GENERATION PLANT. Corner [-1,-1,-1]'s Kp/ell/theta_eq below
// were UPDATED 2026-08-19 for mass 1.633 kg + the new corner-to-housing
// strut (user-supplied); the OTHER SEVEN corners are still the 1.5668
// kg/no-strut values. This table is NOT internally consistent across
// corners -- fine for continued single-corner [-1,-1,-1] bring-up, but
// do not resolve onto or trust any OTHER corner's numbers until they're
// re-derived the same way. [-1,-1,-1]'s own gB (corner-resolution
// direction) was NOT part of this update -- see that entry's own comment
// for why this is a low-risk gap. Replace the remaining seven corners
// (gB/Kp/ellM) once available.
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
                  // gB below is still the OLD direction (not provided
                  // alongside Kp/ell) -- low risk given theta_eq only
                  // moved +0.097 deg, well inside the trim clamp's
                  // +/-2 deg, not the ~3.7 deg this comment used to warn
                  // about before the pole's mounting angle (4.49 deg) got
                  // mistaken for theta_eq.
    , { -0.571549475f, -0.588645577f, -0.571688354f }
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

// AUTOMATIC TRIM state -- see Stage4_AutoTrim.ino's header for the full
// derivation. ADDED to the raw measurement below (phi_err = raw + trim,
// the note's own verified convention -- do not subtract).
static float gTrim[3]     = { 0.0f, 0.0f, 0.0f };   // rad, added to raw phi
static bool  gTrimEnabled = true;                    // "x0" freezes, "x1" resumes

void updateCornerProjection() {
  float t[3];
  cross3(kCorners[gCornerIdx].gB, ghat, t);
  phi[0] = -t[0] + gTrim[0];
  phi[1] = -t[1] + gTrim[1];
  phi[2] = -t[2] + gTrim[2];
}

// Strip yaw, clamp to TRIM_MAX -- see Stage4_AutoTrim.ino, identical.
static const float kTrimMax = 0.0349065850f;   // rad, 2 deg

void applyTrimGuards() {
  const float* gB = kCorners[gCornerIdx].gB;
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
  Serial.print('\t'); Serial.print(overrunCount);
  // Filtered rate readout -- compare against om_x/y/z_dps above to see the
  // fix working directly: hw-run-analysis.md's 35 Hz mode should show up
  // as a much bigger raw-vs-filtered gap than ordinary sensor noise would.
  Serial.print('\t'); Serial.print(w_filt[0] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(w_filt[1] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.println(w_filt[2] * (float)RAD_TO_DEG, 2);
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

static const float kAxisWheelSign[3] = {
  1.0f,   // X -- CONFIRMED
  1.0f,   // Y -- CONFIRMED
  1.0f,   // Z -- CONFIRMED
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

void commandWheels(const float rho[3]) {
  // om uses w_filt, NOT raw w_b -- hw-run-analysis.md fix 4.1. phi and rho
  // are unaffected; only the wheel's Kp row's om block sees the filtered
  // rate. Raw w_b still drives the trip checks below, unfiltered.
  const float xVec[9] = {
    phi[0], phi[1], phi[2],
    w_filt[0], w_filt[1], w_filt[2],
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

// hw-run-analysis.md 4.5: stability limit k_a < 0.00214 (15-state augmented
// model, tau_filt=5s -- see Stage4_AutoTrim_RateFilter.ino's header note).
// 1e-4 sits at ~21x margin and converges in ~10s, replacing Stage5_AutoTrim
// .ino's original tau_a=60s (2.922e-6) default. Live-settable with
// "k<value>" -- NON-MONOTONIC per the source measurement (1e-4 converges
// faster than 1e-3), so re-measure before assuming "higher = faster" past
// this point.
static float gKAdapt = 1.0e-4f;

static const float kTiltQuiet  = 0.00872664619f;   // rad, 0.5 deg
static const float kOmegaQuiet = 0.10f;              // rad/s

void updateTrim(float dt) {
  if (!gTrimEnabled || !gArmed) { return; }
  if (norm3(phi) >= kTiltQuiet)  { return; }
  // w_filt, not raw w_b -- see header note. hw-run-analysis.md's own data
  // showed phi clean while raw om is dominated by the 35 Hz mode, so
  // gating on raw w_b would block adaptation during exactly the condition
  // this fix targets.
  if (norm3(w_filt) >= kOmegaQuiet) { return; }

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
                    "k<value> (trim gain)  f<Hz> (rate filter corner)  "
                    "o<rad/s> (omega_cap)  p<rad/s> (taper_start)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 5: RELEASE + AUTO TRIM + RATE FILTER (stop/catch + e-stop, cube free)");

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
                  "loop_overrun_count\t"
                  "om_x_filt_dps\tom_y_filt_dps\tom_z_filt_dps");
  Serial.println("# STARTS DISARMED. Stop/catch + e-stop ready BEFORE sending a1.");
  Serial.println("# trip_reason: 0 none  1 tilt  2 omega  3 nan");
  Serial.println("# z1/z0 seed/clear trim, x1/x0 freeze/resume adaptation,");
  Serial.println("# k<value> sets adaptation gain (default k_a=1e-4).");
  Serial.print("# Rate low-pass at "); Serial.print(gRateFilterHz, 1);
  Serial.println(" Hz feeds the control law (om_x/y/z_filt_dps in");
  Serial.println("# telemetry) -- f<Hz> to sweep 15-25 Hz live, see hw-run-analysis.md.");
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
  updateRateFilter(dt);   // before commandWheels() -- this cycle's w_filt
                           // must be fresh, unlike gTrim's one-cycle lag

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
// 7), on ONE corner, PLUS hw-run-analysis.md's rate-filter fix (4.1) and
// adaptation-speed change (4.5). vs Stage5_AutoTrim.ino: identical control
// law, identical corner candidate table, identical trim mechanism (guards,
// sign, clamp) -- the only behavioral differences are w_filt replacing w_b
// in commandWheels()'s om block and updateTrim()'s quiescence gate, and
// gKAdapt's default raised from 2.922e-6 to 1e-4. If something looks wrong
// here that isn't about the 35 Hz mode or trim-adaptation speed
// specifically, it's worth checking whether Stage5_AutoTrim.ino has the
// same problem before assuming this file's changes caused it.
//
// Suggested order, matching the Test Plan's own:
//   1. Test 5: arm, let trim converge (watch trim_x/y/z_deg settle, ~4
//      time constants at this file's default k_a=1e-4 is ~10 s), or "z1"
//      to fast-start.
//   2. Test 6: with trim converged, read the settled rho_x/y/z_lp
//      (standing_speed_report.py, or just watch telemetry), set
//      "o<3x that>" and "p<90% of that>", verify it still balances a
//      couple minutes, then step both toward the real policy (o40 p36)
//      as trim settles further.
//   3. Test 7: once 6 is done, arm and leave it -- 30 min, then check
//      trim/standing-speed/temperature/loop_overrun_count/sat_duty
//      against Test 7's table (plot_session_csv.py --cols trim,wheels,
//      endurance, or fine-tuning/README.md for the exact commands).
//      Also watch om_x/y/z_filt_dps vs om_x/y/z_dps -- a persistently
//      large raw-vs-filtered gap over a 30-minute run is the 35 Hz mode
//      still present (see hw-run-analysis.md), not a bug in this file.
//
// Re-derive Kp once the battery cable/DC-DC (and eventually the flight
// battery itself) are actually mounted (measure -> derive -> re-tune, same
// workflow as everywhere else in this project) -- trim will re-converge to
// a different, smaller value once they are, that is not a sign anything
// here is wrong. The SAME applies to the new mass (1.633 kg) and strut
// hw-run-analysis.md references -- corner [-1,-1,-1]'s Kp is done
// (2026-08-19), the other seven are still the old table (see the TODO
// above it); drop in their re-derived Kp/gB/ellM once available, before
// drawing conclusions from a multi-corner comparison. Then repeat this
// file's Tests 5-7 for the OTHER SEVEN corners before trusting any of
// them -- a result on one corner says nothing about another (same
// discipline edge-bringup established per-axis, now per-corner). Once
// multiple corners are validated, corner-to-corner transitions are where
// the quaternion/MEKF estimator becomes load-bearing (Attitude
// representation for the firmware.md section 4) -- this reduced-attitude
// gam estimator is deliberately NOT that, and isn't meant to be pushed
// into large-angle territory.
// ============================================================================
