// ============================================================================
// TEENSY 4.1 + 3x moteus-n1 (CAN3) + BMI270 IMU (SPI) — 3D SKELETON (EULER)
// ============================================================================
// Sibling of "firmware/3D model/Quaternion/Skeleton_3Axis/Skeleton_3Axis.ino".
// Same CAN/IMU boilerplate, telemetry-mode selector, and serial-command
// interface as that file and the 2D Stage4 sketch. What differs is the
// attitude representation: this build is the CURRENT FOCUS -- balance-only,
// using the reduced gravity-direction state below. Full quaternion attitude
// (needed for jump-up, once we get there) lives in the Quaternion/ sibling
// instead of here -- do not add quaternion code to this file.
//
// ----------------------------- ATTITUDE PIPELINE ---------------------------
// State is g_hat (unit vector, direction of gravity in body axes -- 2 DOF,
// singularity-free everywhere on the sphere), NOT roll/pitch/yaw. Fused from
// accel + gyro by a complementary filter (attitudeUpdate() below). Euler
// angles (roll, pitch) are derived from g_hat for TELEMETRY ONLY -- never fed
// back into the control law. Yaw is neither observable from this IMU alone
// (accelerometer can't see rotation about gravity, and a magnetometer this
// close to 3 permanent-magnet reaction wheels at high current is unusable)
// nor needed for balance (the tilt error below is yaw-invariant by
// construction) -- only yaw RATE is used, for momentum damping.
//
// calculateState()'s replacement (attitudeUpdate()) is wired and live;
// commandWheels() is an EMPTY STUB -- always commands zero torque, same as
// the Quaternion sibling -- see the TODO inside it. Nothing LQR/state-
// feedback-shaped is implemented yet in any of the 4 sketches (Euler/
// Quaternion x USB/WiFi). Kp/Kd/K_YAW_RATE above are declared per the
// attitude-spec interface but not yet read anywhere.
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <stdio.h>   // snprintf(), for printState()'s aligned columns

// Declared here (ahead of everything else) rather than down by its usage in
// SECTION 2b -- Arduino's auto-generated function prototypes get inserted
// right after the last #include, in file order. getGRef() takes BalanceMode
// by value, so that type must already be visible at this point or the
// auto-generated prototype fails to compile with "was not declared in this
// scope".
enum class BalanceMode : uint8_t { FACE = 0, EDGE = 1, CORNER = 2 };


// ----------------------------------------------------------------------------
// SECTION 1b: TELEMETRY MODE SELECTOR
// ----------------------------------------------------------------------------
// SERIALMONITORMODE: human-readable, tab-delimited, with header + "#"
//                     status lines -- for reading directly in the Arduino
//                     Serial Monitor.
// PLOTMODE:           plain CSV, one line per control cycle, no header/
//                      comment lines -- for a matlab/3Dmodel/Validation
//                      live-plot script.
// Change the line below and re-upload to switch modes.
#define SERIALMONITORMODE 0
#define PLOTMODE 1
#define TELEMETRY_MODE SERIALMONITORMODE


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// TODO: confirm id <-> physical wheel-axis mapping during 3D bring-up (set/
// verified per-driver via tview, same as the 2D sketch's moteus1).
Moteus moteusX(canBus, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());
Moteus moteusY(canBus, []() {
  Moteus::Options options;
  options.id = 2;
  return options;
}());
Moteus moteusZ(canBus, []() {
  Moteus::Options options;
  options.id = 3;
  return options;
}());
Moteus* const gWheels[3] = { &moteusX, &moteusY, &moteusZ };

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Software pause -- see SECTION 2e. Does NOT stop loop() on its own, force-
// disarms, and does not auto-resume armed on "h0" -- a fresh "a1" is still
// required. Same semantics as the 2D sketch.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
// TODO: these calibration constants were measured for the 2D panel's IMU
// mount (Stage 0). Recalibrate for the 3D rig's mount/orientation before
// trusting attitudeUpdate() below -- run
// "firmware/3D model/IMU_Calibration/IMU_Calibration.ino" on the bench, then
// paste its printed block in place of the three lines below verbatim
// (SENSOR-frame values -- readIMURaw() below applies them before rotation).
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };

// ----------------------------------------------------------------------------
// STAGE 1: IMU MOUNT TRANSFORM (sensor frame -> body frame)
// ----------------------------------------------------------------------------
// The IMU sits at the cube's geometric centre (no lever arm -- d = 0, so no
// centripetal/Euler correction is needed on the accelerometer, and the gyro
// was never subject to this since angular velocity is identical at every
// point of a rigid body). The only correction needed is a ROTATION.
//
// Mounting sequence: intrinsic Z -> X -> Z (theta1 about sensor z, theta2
// about the resulting x, theta3 about the resulting z). Because this rotates
// AXES not vectors, C is the coordinate-transformation matrix (transpose of
// the active-rotation form): C = Rz(theta3)*Rx(theta2)*Rz(theta1), expanded
// to closed form below -- do not multiply the elementary matrices at
// runtime, and do not transpose the result.
//
// Closed-form verified by hand against theta1=30, theta2=-135, theta3=45:
//   C = [ 0.862372  -0.079459  -0.500000
//        -0.362372  -0.786566  -0.500000
//        -0.353553   0.612372  -0.707107 ]
// The defaults below (theta1=-30, theta2=35.26438968) are this rig's actual
// measured corner-mount geometry, not that reference case -- they're
// checked at runtime instead, by checkMountingDCMValid() in setup()
// (det(C)~=+1 and unit row norms).
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

void setMountingAngles(float t1Deg, float t2Deg, float t3Deg) {
  theta1_deg = t1Deg; theta2_deg = t2Deg; theta3_deg = t3Deg;
  updateMountingDCM();
}

// One-time startup sanity check -- verifies gMountDCM is a proper rotation
// (det ~= +1, each row unit-length). Row-length checks alone can't catch a
// reflection (det ~= -1) from a sign error in the angles above; this can.
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
  // Expect det ~= 1.0000 and all row lengths ~= 1.0000. If not, theta1_deg/
  // theta2_deg/theta3_deg were mistyped -- fix before trusting attitude output.
}

// v_body = C * v_imu, applied to both accel and gyro.
void rotateToBodyFrame(const float aImu[3], const float wImu[3],
                       float aBody[3], float wBody[3]) {
  for (int i = 0; i < 3; ++i) {
    wBody[i] = gMountDCM[i][0]*wImu[0] + gMountDCM[i][1]*wImu[1] + gMountDCM[i][2]*wImu[2];
    aBody[i] = gMountDCM[i][0]*aImu[0] + gMountDCM[i][1]*aImu[1] + gMountDCM[i][2]*aImu[2];
  }
}

// STAGE 0: unit conversion + (sensor-frame) calibration, done ONCE at the
// driver boundary and never again downstream. SENSOR frame -- rotation to
// body frame happens inside attitudeUpdate() (Stage 1), not here.
void readIMURaw(BMI270& sensor, float aImu[3], float wImu[3]) {
  aImu[0] = (sensor.data.accelX * kG0 - kAccelOffset[0]) / kAccelScale[0];
  aImu[1] = (sensor.data.accelY * kG0 - kAccelOffset[1]) / kAccelScale[1];
  aImu[2] = (sensor.data.accelZ * kG0 - kAccelOffset[2]) / kAccelScale[2];
  wImu[0] = sensor.data.gyroX * (float)DEG_TO_RAD - kGyroBias[0];
  wImu[1] = sensor.data.gyroY * (float)DEG_TO_RAD - kGyroBias[1];
  wImu[2] = sensor.data.gyroZ * (float)DEG_TO_RAD - kGyroBias[2];
}

// ----------------------------------------------------------------------------
// STAGE 2/3: CONTROL STATE -- gravity direction, NOT Euler angles
// ----------------------------------------------------------------------------
// g_hat: unit vector, direction of gravity in body axes (2 DOF, no
// parameterization singularity anywhere on the sphere).
// w_b:   body rates in body axes, rad/s, gyro-bias-corrected.
// e:     tilt error, g_hat x g_ref -- |e| ~= sin(tilt error), axis =
//        correction axis. Yaw-invariant by construction.
// r_vert: yaw RATE about the true vertical (w_b . g_hat) -- yaw ANGLE is
//        never computed or integrated, see file header.
float g_hat[3] = { 0.0f, 0.0f, 1.0f };
float w_b[3]   = { 0.0f, 0.0f, 0.0f };
float e[3]     = { 0.0f, 0.0f, 0.0f };
float r_vert   = 0.0f;
float dt_s     = 0.0f;   // last dt used by attitudeUpdate(), telemetry only

// Runtime tunables (variables, not #define -- see interface note below).
float K_ACC      = 0.02f;  // complementary-filter accel weight per cycle --
                            // TODO retune for this loop's actual rate (500 Hz
                            // here vs the 100 Hz this default assumes).
float ACC_GATE   = 1.5f;   // m/s^2, half-width of the |a|~=g0 trust gate --
                            // rejects samples contaminated by wheel-reaction
                            // tangential acceleration (same idea as the 2D
                            // sketch's accel gate, generalized to |a| not a
                            // single-axis check).
float K_YAW_RATE = 0.0f;   // N*m / (rad/s) -- TODO placeholder, see header.
float Kp         = 0.0f;   // N*m / rad -- TODO placeholder, see header.
float Kd         = 0.0f;   // N*m / (rad/s) -- TODO placeholder, see header.

static float gGyroBiasBody[3] = { 0.0f, 0.0f, 0.0f };
// Residual body-frame gyro bias, measured fresh at each power-on by
// calibrateGyroBias() below -- on top of (not instead of) the sensor-frame
// kGyroBias[] constants above, which come from the offline bench tool.

// Estimate and subtract gyro bias at startup: average body-frame rate over
// n_samples while the cube is CONFIRMED STATIONARY (call this from setup(),
// after updateMountingDCM() and IMU init, before ever arming).
void calibrateGyroBias(uint16_t n_samples) {
  float sum[3] = { 0.0f, 0.0f, 0.0f };
  for (uint16_t i = 0; i < n_samples; ++i) {
    imu.getSensorData();
    float aImu[3], wImu[3], aBody[3], wBody[3];
    readIMURaw(imu, aImu, wImu);
    rotateToBodyFrame(aImu, wImu, aBody, wBody);
    sum[0] += wBody[0]; sum[1] += wBody[1]; sum[2] += wBody[2];
    delay(kPeriodMs);
  }
  gGyroBiasBody[0] = sum[0] / n_samples;
  gGyroBiasBody[1] = sum[1] / n_samples;
  gGyroBiasBody[2] = sum[2] / n_samples;
}

static inline float norm3(const float v[3]) {
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static inline void normalize3(float v[3]) {
  const float n = norm3(v);
  if (n > 1e-9f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}
static inline void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

// Which surface we're balancing on -- selects g_ref, the body-axis direction
// that must align with g_hat at equilibrium. Assumes the post-rotation body
// frame is axis-aligned with the cube faces; re-derive these if it isn't.
static BalanceMode gBalanceMode = BalanceMode::CORNER;   // 3-wheel Cubli default

static const float G_FACE[3]   = { 0.0f, 0.0f, 1.0f };
static const float G_EDGE[3]   = { 0.0f, 0.70710678f, 0.70710678f };
static const float G_CORNER[3] = { 0.57735027f, 0.57735027f, 0.57735027f };

void getGRef(BalanceMode mode, float out[3]) {
  const float* src = (mode == BalanceMode::FACE) ? G_FACE
                    : (mode == BalanceMode::EDGE) ? G_EDGE
                                                   : G_CORNER;
  out[0] = src[0]; out[1] = src[1]; out[2] = src[2];
}

// Full attitude pipeline for one control cycle: Stage 1 (mount rotation) +
// Stage 2 (complementary filter on g_hat) + Stage 3 (tilt error + yaw rate).
// Input:  a_imu, w_imu -- SENSOR-frame, calibrated (Stage 0, readIMURaw()).
//         dt            -- seconds since the previous call.
// Output: g_hat/w_b/e/r_vert updated in place (module-level, in/out -- carry
//         the previous estimate in, like the 2D sketch's static theta).
void attitudeUpdate(const float a_imu[3], const float w_imu[3], float dt) {
  dt_s = dt;

  float aBody[3], wBodyRaw[3];
  rotateToBodyFrame(a_imu, w_imu, aBody, wBodyRaw);

  w_b[0] = wBodyRaw[0] - gGyroBiasBody[0];
  w_b[1] = wBodyRaw[1] - gGyroBiasBody[1];
  w_b[2] = wBodyRaw[2] - gGyroBiasBody[2];

  static bool initialized = false;
  if (!initialized) {
    g_hat[0] = aBody[0]; g_hat[1] = aBody[1]; g_hat[2] = aBody[2];
    normalize3(g_hat);
    initialized = true;
  } else {
    // 1. predict with gyro: g_dot = -(w x g_hat)
    float wxg[3];
    cross3(w_b, g_hat, wxg);
    float g_pred[3] = {
      g_hat[0] - wxg[0]*dt,
      g_hat[1] - wxg[1]*dt,
      g_hat[2] - wxg[2]*dt,
    };
    normalize3(g_pred);

    // 2. accel measurement, GATED -- during jump-up/impacts/hard wheel
    // braking the accelerometer reads specific force, not tilt, and will
    // actively lie. When |a| != g0, trust the gyro and coast.
    const float aNorm = norm3(aBody);
    const float dev = fabsf(aNorm - kG0);
    float k = K_ACC;
    if (dev > ACC_GATE) { k = 0.0f; }

    // 3. blend
    if (k > 0.0f && aNorm > 1e-6f) {
      g_hat[0] = (1.0f - k)*g_pred[0] + k*(aBody[0]/aNorm);
      g_hat[1] = (1.0f - k)*g_pred[1] + k*(aBody[1]/aNorm);
      g_hat[2] = (1.0f - k)*g_pred[2] + k*(aBody[2]/aNorm);
      normalize3(g_hat);
    } else {
      g_hat[0] = g_pred[0]; g_hat[1] = g_pred[1]; g_hat[2] = g_pred[2];
    }
  }

  float gRef[3];
  getGRef(gBalanceMode, gRef);
  cross3(g_hat, gRef, e);

  r_vert = w_b[0]*g_hat[0] + w_b[1]*g_hat[1] + w_b[2]*g_hat[2];
}

// ----------------------------------------------------------------------------
// STAGE 4: Euler angles -- TELEMETRY ONLY, never fed back into control.
// ----------------------------------------------------------------------------
// Z-Y-X (yaw-pitch-roll) from g_hat. Yaw is not observable from g_hat alone
// (rotation about gravity is invisible to it) -- omitted (NAN), not
// computed. Uses atan2f, never atan, so this stays defined over the full
// range. If an Euler-RATE expression is ever added on top of this, it must
// guard the 1/cos(pitch) term (freeze yaw rate + raise a near-singularity
// flag below |cos(pitch)| ~= 0.15, never clamp cos to a floor -- that
// fabricates a rate that doesn't exist). On corner balance the face normals
// sit at 54.7 deg from vertical, leaving only ~35 deg of margin before that
// singularity -- exactly why the control loop above uses g_hat instead.
void eulerForTelemetry(const float gHat[3], float& rollRad, float& pitchRad) {
  rollRad  = atan2f(gHat[1], gHat[2]);
  pitchRad = atan2f(-gHat[0], sqrtf(gHat[1]*gHat[1] + gHat[2]*gHat[2]));
}


// ----------------------------------------------------------------------------
// SECTION 2c: TELEMETRY
// ----------------------------------------------------------------------------

// Fixed-width columns (right-justified, matching printState()'s field
// widths/precisions below) so rows stay aligned under their header
// regardless of sign or magnitude -- tabs alone don't guarantee that in a
// Serial Monitor. Trimmed to just orientation + per-wheel speed/torque,
// plus each wheel's own mode/fault (see the read loop in loop() for what
// these mean -- mode 1 = FAULT is the one to watch for while debugging).
// Full state (g_hat, body rates, tilt error, armed/gain, wheel position) is
// still available via telemetryPlot() in PLOTMODE.
static const char kHeaderLine[] =
    "    t_ms roll_deg pitch_deg velX_rps velY_rps velZ_rps  tauX_Nm  tauY_Nm  tauZ_Nm   m1   f1   m2   f2   m3   f3";

void printState(uint32_t t_ms, const float tauCmd[3], const float wheelVel[3],
                const int wheelMode[3], const int wheelFault[3]) {
  // Header printed ahead of every data line (not just once at boot) so each
  // row is self-labeled in a scrolling Serial Monitor.
  Serial.println(kHeaderLine);

  float rollRad, pitchRad;
  eulerForTelemetry(g_hat, rollRad, pitchRad);

  char line[128];
  snprintf(line, sizeof(line),
           "%8lu %8.2f %9.2f %8.3f %8.3f %8.3f %8.4f %8.4f %8.4f %4d %4d %4d %4d %4d %4d",
           (unsigned long)t_ms,
           rollRad  * (float)RAD_TO_DEG,
           pitchRad * (float)RAD_TO_DEG,
           wheelVel[0], wheelVel[1], wheelVel[2],
           tauCmd[0], tauCmd[1], tauCmd[2],
           wheelMode[0], wheelFault[0],
           wheelMode[1], wheelFault[1],
           wheelMode[2], wheelFault[2]);
  Serial.println(line);
}

// Same fields as printState() above, same order, comma-separated, no
// header/comment lines -- for a PLOTMODE live-plot script.
void telemetryPlot(uint32_t t_ms, const float tauCmd[3], bool armed, float gainScale,
                   float wheelOmegaVert, const float wheelPos[3], const float wheelVel[3]) {
  float rollRad, pitchRad;
  eulerForTelemetry(g_hat, rollRad, pitchRad);

  Serial.print(t_ms);
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(g_hat[i], 6); }
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(w_b[i] * (float)RAD_TO_DEG, 4); }
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(e[i], 6); }
  Serial.print(','); Serial.print(r_vert * (float)RAD_TO_DEG, 4);
  Serial.print(','); Serial.print(rollRad  * (float)RAD_TO_DEG, 4);
  Serial.print(','); Serial.print(pitchRad * (float)RAD_TO_DEG, 4);
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(tauCmd[i], 6); }
  Serial.print(','); Serial.print(armed ? 1 : 0);
  Serial.print(','); Serial.print(gainScale, 4);
  Serial.print(','); Serial.print(wheelOmegaVert, 6);
  for (int i = 0; i < 3; ++i) {
    Serial.print(','); Serial.print(wheelPos[i], 6);
    Serial.print(','); Serial.print(wheelVel[i], 6);
  }
  Serial.println();
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL (commandWheels() is a STUB -- see TODO)
// ----------------------------------------------------------------------------
// Estimator (g_hat/w_b/e/r_vert above) is wired and live; the control law is
// not -- nothing LQR/state-feedback-shaped here yet. Kp/Kd/K_YAW_RATE are
// declared above (per the attitude-spec interface) but not read below.

static bool  gArmed     = false;
static float gGainScale = 0.1f;   // START LOW -- same convention as the 2D
                                   // sketches (see Stage3_PositionDamping.ino):
                                   // once commandWheels() below outputs real
                                   // torque, ramp this by hand ('g0.3', 'g0.6',
                                   // 'g1') rather than arming straight to 1.0.

// TODO: placeholder so sendWheelTorque() below has a concrete number to
// compile against. Re-derive per-wheel for the 3D hardware (mirror the 2D
// Stage 1-3 bring-up process) before arming for real.
static const float kTauMax = 0.12f;

// Per-wheel mounting sign convention -- TODO: calibrate one sign check per
// axis, same method as the 2D sketch's Stage 1 sign check 4. Does NOT
// generalize from one wheel to three -- verify each independently.
static const float kWheelSign[3] = { 1.0f, 1.0f, 1.0f };

// Same DEFAULT-wire-format gotcha as the 2D sketch: SetPosition() silently
// drops feedforward_torque/kp_scale/kd_scale/maximum_torque/
// watchdog_timeout unless this Format explicitly turns each one on.
static Moteus::PositionMode::Format kTorqueFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque = Moteus::kFloat;
  f.kp_scale            = Moteus::kFloat;
  f.kd_scale             = Moteus::kFloat;
  f.maximum_torque       = Moteus::kFloat;
  f.watchdog_timeout     = Moteus::kFloat;
  f.ignore_position_bounds = Moteus::kFloat;
  return f;
}();

void sendWheelTorque(Moteus& wheel, float tau) {
  Moteus::PositionMode::Command cmd;
  cmd.position           = NaN;
  cmd.velocity           = 0.0f;
  cmd.kp_scale           = 0.0f;
  cmd.kd_scale           = 0.0f;
  cmd.feedforward_torque = tau;
  cmd.maximum_torque     = kTauMax;
  cmd.watchdog_timeout   = 0.10f;
  cmd.ignore_position_bounds = 1.0f;  // reaction wheels have no travel limit.
  wheel.SetPosition(cmd, &kTorqueFormat);
}

// TODO: the actual 3-axis control law (state feedback / LQR / etc.), mapping
// g_hat/w_b/e/r_vert (from attitudeUpdate()) and wheelOmega to 3 output
// torques. Mirrors commandWheel() in the 2D Stage4 sketch (tau = -(k1*theta
// + k2*theta_dot + k3*wheel_omega) * gGainScale, then taper/saturate/trip-
// on-limit), generalized to 3 axes -- e in place of theta, w_b in place of
// theta_dot, r_vert feeding a yaw-damping term. When implementing this, also
// move the arm-gating, gGainScale scaling, saturation, and safety-limit
// trips (gArmed = false on overtilt/overspeed/non-finite) in here.
//
// Input:  wheelOmega -- current wheel speeds, rad/s (wheelOmega[0..2]).
// Output: tauCmd     -- commanded torque per wheel, N*m (tauCmd[0..2]),
//                        written by this function.
//
// Safe placeholder until implemented: always commands zero torque.
void commandWheels(const float wheelOmega[3], float tauCmd[3]) {
  for (int i = 0; i < 3; ++i) { tauCmd[i] = 0.0f; }
  (void)wheelOmega;
}


// ----------------------------------------------------------------------------
// SECTION 2e: SERIAL COMMANDS
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
#if TELEMETRY_MODE == SERIALMONITORMODE
    Serial.print("# gArmed = "); Serial.println(gArmed ? "TRUE" : "FALSE");
#endif
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
#if TELEMETRY_MODE == SERIALMONITORMODE
    Serial.print("# gGainScale = "); Serial.println(gGainScale, 3);
#endif
  } else if (cmd == 'h') {
    gHalted = (val != 0.0f);
    if (gHalted && gArmed) {
      gArmed = false;
#if TELEMETRY_MODE == SERIALMONITORMODE
      Serial.println("# disarmed by halt");
#endif
    }
#if TELEMETRY_MODE == SERIALMONITORMODE
    Serial.print("# gHalted = ");
    Serial.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                            : "FALSE (resumed, still DISARMED -- send a1)");
#endif
  } else if (cmd == 'm') {
    gBalanceMode = (val < 0.5f) ? BalanceMode::FACE
                 : (val < 1.5f) ? BalanceMode::EDGE
                                : BalanceMode::CORNER;
#if TELEMETRY_MODE == SERIALMONITORMODE
    Serial.print("# gBalanceMode = ");
    Serial.println(gBalanceMode == BalanceMode::FACE ? "FACE"
                  : gBalanceMode == BalanceMode::EDGE ? "EDGE" : "CORNER");
#endif
#if TELEMETRY_MODE == SERIALMONITORMODE
  } else {
    Serial.println("# unknown. use: a<0/1>  g<0..1>  h<0/1>  m<0=face/1=edge/2=corner>");
#endif
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - 3D SKELETON (EULER/g_hat build -- commandWheels() not yet implemented)");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  for (int i = 0; i < 3; ++i) { gWheels[i]->SetStop(); }
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

  if (imu.setAccelODR(BMI2_ACC_ODR_400HZ) != BMI2_OK ||
      imu.setGyroODR(BMI2_GYR_ODR_400HZ)  != BMI2_OK) {
    Serial.println("Warning: could not raise BMI270 ODR to 400 Hz");
  }

  Serial.println("# calibrating gyro bias -- keep the cube PERFECTLY STILL (~2s)");
  calibrateGyroBias(1000);
  Serial.print("# gyro bias (body, rad/s): ");
  Serial.print(gGyroBiasBody[0], 6); Serial.print('\t');
  Serial.print(gGyroBiasBody[1], 6); Serial.print('\t');
  Serial.println(gGyroBiasBody[2], 6);

#if TELEMETRY_MODE == SERIALMONITORMODE
  Serial.println(kHeaderLine);
  Serial.println("# STARTS DISARMED. Send a1 to arm.");
  Serial.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");
  Serial.println("# m0/m1/m2 selects face/edge/corner balance mode.");
  Serial.println("# NOTE: commandWheels() is a stub -- always commands zero torque.");
#endif

  gNextSendMillis = millis();

}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {

  handleSerialCommands();

  // Halted: skip IMU reads, CAN traffic, and telemetry entirely. Serial
  // commands above still run every pass, so "h0" always gets through.
  if (gHalted) { return; }

  if (static_cast<int32_t>(millis() - gNextSendMillis) < 0) { return; }
  gNextSendMillis += kPeriodMs;

  const uint32_t time = millis();

  imu.getSensorData();

  float aImu[3], wImu[3];
  readIMURaw(imu, aImu, wImu);

  static uint32_t lastMicros = 0;
  static bool dtInitialized = false;
  const uint32_t nowMicros = micros();
  float dt = dtInitialized ? (nowMicros - lastMicros) * 1e-6f : kPeriodMs * 1e-3f;
  lastMicros = nowMicros;
  dtInitialized = true;
  if (dt <= 0.0f || dt > 0.5f) { dt = kPeriodMs * 1e-3f; }

  attitudeUpdate(aImu, wImu, dt);

  // --- wheel speeds from the previous cycle's moteus replies ---
  float wheelPos[3];
  float wheelVel[3];      // rev/s, raw from moteus
  float wheelOmega[3];    // rad/s, sign-corrected -- what commandWheels() consumes
  // mode/fault: the moteus's OWN reported state -- register 0x000/0x00f.
  // Mode 1 = FAULT (fault code tells you why: 39 outside position limit, 36
  // motor never calibrated via moteus_tool, 34/40 over/under voltage --
  // full list at mjbots.github.io/moteus/protocol/registers). A moteus in
  // FAULT keeps replying to CAN queries with normal-looking position/
  // velocity, it just silently stops applying torque -- this is the field
  // that tells "not working" apart from "working but nothing's moving".
  // v.fault is int8_t and v.mode is an enum -- Serial.print() has a
  // print(char) overload either can silently bind to (fault code 39 would
  // print as "'" instead of "39"); cast both to (int) to force the numeric
  // overload. See firmware/Firmware Lessons -- 2D Panel to 3D Cube.md #3.
  int wheelMode[3];
  int wheelFault[3];
  for (int i = 0; i < 3; ++i) {
    const auto& v = gWheels[i]->last_result().values;
    wheelPos[i] = v.position;
    wheelVel[i] = v.velocity;
    wheelOmega[i] = kWheelSign[i] * v.velocity * 2.0f * (float)PI;   // rev/s -> rad/s
    wheelMode[i]  = (int)v.mode;
    wheelFault[i] = (int)v.fault;
  }
  const float wheelOmegaVert = wheelOmega[0]*g_hat[0] + wheelOmega[1]*g_hat[1] + wheelOmega[2]*g_hat[2];
#if TELEMETRY_MODE != PLOTMODE
  // Only telemetryPlot() (PLOTMODE) reads these -- printState()'s trimmed
  // columns don't. Kept computed either way (cheap, and PLOTMODE needs them
  // unchanged) but silenced here to avoid unused-variable warnings.
  (void)wheelPos;
  (void)wheelOmegaVert;
#endif

  // --- control + command ---
  float tauCmd[3];
  commandWheels(wheelOmega, tauCmd);
  for (int i = 0; i < 3; ++i) {
    sendWheelTorque(*gWheels[i], kWheelSign[i] * tauCmd[i]);
  }

  // --- telemetry ---
#if TELEMETRY_MODE == PLOTMODE
  telemetryPlot(time, tauCmd, gArmed, gGainScale, wheelOmegaVert, wheelPos, wheelVel);
#else
  printState(time, tauCmd, wheelVel, wheelMode, wheelFault);
#endif

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// The estimator (attitudeUpdate() -> g_hat/w_b/e/r_vert) is architecture,
// live and wired. commandWheels() is an EMPTY STUB -- always zero torque,
// nothing LQR/state-feedback-shaped yet. Before this sketch does anything to
// real hardware:
//   1. Recalibrate kGyroBias/kAccelOffset/kAccelScale for this IMU mount
//      (run IMU_Calibration.ino) -- calibrateGyroBias() above only corrects
//      RESIDUAL bias on top of these, it doesn't replace them.
//   2. Confirm theta1_deg/theta2_deg/theta3_deg against the actual built
//      mount (checkMountingDCMValid() catches typos, not wrong geometry).
//   3. Calibrate kWheelSign[3] with a per-axis sign check (2D Stage 1
//      method) before ever combining wheels in a closed loop.
//   4. Implement commandWheels() -- 3-axis control law (state feedback,
//      LQR, or otherwise) using g_hat/w_b/e/r_vert, including the arm-
//      gating/gGainScale scaling/saturation/safety-trip logic that lives
//      inside commandWheel() in the 2D Stage4 sketch, then stage gains in
//      exactly like that sketch did (isolated pulse -> one term at a time ->
//      ramped combined -> full authority) -- never jump straight to three
//      wheels closed loop simultaneously.
// ============================================================================
