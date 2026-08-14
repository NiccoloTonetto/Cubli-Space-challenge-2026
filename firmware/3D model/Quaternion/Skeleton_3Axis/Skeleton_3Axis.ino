// ============================================================================
// TEENSY 4.1 + 3x moteus-n1 (CAN3) + BMI270 IMU (SPI) — 3D SKELETON
// ============================================================================
// ARCHITECTURE ONLY. Mirrors the section layout, telemetry-mode selector,
// serial-command interface, and CAN/IMU boilerplate of
// "firmware/2D model/panel-bringup/Stage4_FullLaw/Stage4_FullLaw.ino",
// extended from ONE reaction wheel + a single tilt angle to THREE reaction
// wheels (nominally one per body axis) + a full 3D attitude.
//
// calculateState() and commandWheels() are intentionally EMPTY STUBS -- see
// the TODOs inside each. Everything else (CAN setup, IMU setup, per-wheel
// torque command send, telemetry, serial commands) is wired the same way
// the 2D sketch wires it, just generalized from 1 wheel to 3.
//
// Does NOT compile into a bring-up stage yet -- there is no Stage 0-5
// sequence for 3D defined. This is the starting point that sequence will
// be built on top of, once calculateState()/commandWheels() are written.
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"


// ----------------------------------------------------------------------------
// SECTION 1b: TELEMETRY MODE SELECTOR
// ----------------------------------------------------------------------------
// Same pattern as the 2D Stage4 sketch.
// SERIALMONITORMODE: human-readable, tab-delimited, with header + "#"
//                     status lines -- for reading directly in the Arduino
//                     Serial Monitor.
// PLOTMODE:           plain CSV, one line per control cycle, no header/
//                      comment lines -- for a matlab/3Dmodel/Validation
//                      live-plot script (TODO, mirror
//                      matlab/2Dmodel/Validation/telemetry_matlab.m or
//                      telemetry_python.py -- both read the same format).
// Change the line below and re-upload to switch modes.
#define SERIALMONITORMODE 0
#define PLOTMODE 1
#define TELEMETRY_MODE SERIALMONITORMODE


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// Three drivers on one CAN3 FD bus, addressed by id -- same bus wiring as
// the 2D sketch's single moteus1, just three nodes instead of one.
// Confirmed on bench via Stage0_SingleMoteusQuery (cube-bringup/, gam build --
// this is hardware fact, not representation-specific): id 3 -> X, id 2 -> Y,
// id 1 -> Z. Sign is NOT part of this mapping -- kWheelSign gets determined
// per-wheel by the isolated-pulse test (Phase 1.3), separately.
Moteus moteusX(canBus, []() {
  Moteus::Options options;
  options.id = 3;
  return options;
}());
Moteus moteusY(canBus, []() {
  Moteus::Options options;
  options.id = 2;
  return options;
}());
Moteus moteusZ(canBus, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());
Moteus* const gWheels[3] = { &moteusX, &moteusY, &moteusZ };

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Software pause -- see SECTION 2e. Same semantics as the 2D sketch: does
// NOT stop loop() on its own, force-disarms, and does not auto-resume
// armed on "h0" -- a fresh "a1" is still required.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION (calculateState() is a STUB -- see TODO)
// ----------------------------------------------------------------------------

// Unchanged from the 2D sketch -- the BMI270 is still just a 6-axis IMU
// regardless of how many wheels read it.
struct IMUData {
  float ax, ay, az;
  float wx, wy, wz;
};

static const float kG0 = 9.80665f;
// TODO: these calibration constants were measured for the 2D panel's IMU
// mount (Stage 0). Recalibrate for the 3D rig's mount/orientation before
// trusting calculateState() below -- run
// "firmware/3D model/IMU_Calibration/IMU_Calibration.ino" on the bench,
// then paste its printed block in place of the three lines below verbatim
// (SENSOR-frame values, same convention this file already applies in
// readIMU() -- see that sketch's header for why it must be run against the
// board's own axes, not the cube's).
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };

// ----------------------------------------------------------------------------
// IMU MOUNT TRANSFORM: sensor frame -> body frame
// ----------------------------------------------------------------------------
// The IMU sits at the cube's geometric centre (no measurable offset), so
// the only correction needed is a ROTATION -- gMountDCM rotates sensor-axis
// readings into body axes. There is no lever arm, so no omega-x-r
// (centripetal) or alpha-x-r (Euler) term is needed for the accelerometer
// either -- those only arise when the sensor is offset from the point whose
// motion you care about, and here it isn't. (The gyro was never subject to
// this: angular velocity is identical at every point of a rigid body.)
//
// Mounting sequence: intrinsic Z -> X -> Z (theta1 about sensor z, theta2
// about the resulting x, theta3 about the resulting z). gMountDCM is the
// coordinate-transformation matrix C = Rz(theta3)*Rx(theta2)*Rz(theta1),
// expanded to closed form in updateMountingDCM() -- do not multiply the
// elementary matrices at runtime, and do not transpose the result.
//
// gTheta1Deg/gTheta2Deg/gTheta3Deg below are the EXACT equivalent of this
// mount's known geometry: the IMU sits at the cube's centre with sensor Z
// toward corner G=(1,1,1) and sensor X through the FB/DH edge midpoints.
// theta1=0, theta2=acos(1/sqrt(3))=54.7356 deg (the cube space-diagonal
// half-angle), theta3=45. Verified by hand against the direct unit-vector
// derivation -- reproduces it exactly, so this parametrization changes
// NOTHING about current behaviour; it only makes the mount angle
// runtime-tunable. Re-measure and call setMountingAngles() if the assembled
// mount ever deviates from this ideal corner geometry.
//
// GIMBAL LOCK: this Z-X-Z sequence is degenerate at theta2 = 0 or 180 deg,
// where theta1 and theta3 become indistinguishable. 54.7356 deg is nowhere
// near either, so this is not a live concern for the default mount -- but
// don't retune theta2 toward 0/180 without accounting for it.
float gTheta1Deg = 30.0f;
float gTheta2Deg = -144.7356f;
float gTheta3Deg = 45.0f;

float gMountDCM[3][3];

void updateMountingDCM() {
  const float k  = (float)DEG_TO_RAD;
  const float c1 = cosf(gTheta1Deg * k), s1 = sinf(gTheta1Deg * k);
  const float c2 = cosf(gTheta2Deg * k), s2 = sinf(gTheta2Deg * k);
  const float c3 = cosf(gTheta3Deg * k), s3 = sinf(gTheta3Deg * k);

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
  gTheta1Deg = t1Deg; gTheta2Deg = t2Deg; gTheta3Deg = t3Deg;
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
  // Expect det ~= 1.0000 and all row lengths ~= 1.0000. If not, gTheta1Deg/
  // gTheta2Deg/gTheta3Deg were mistyped -- fix before trusting attitude output.
}

// Rotates sensor-frame accel+gyro into body axes: w_body = C * w_imu,
// a_body = C * a_imu.
void rotateToBodyFrame(const float aImu[3], const float wImu[3],
                       float aBody[3], float wBody[3]) {
  for (int i = 0; i < 3; ++i) {
    wBody[i] = gMountDCM[i][0]*wImu[0] + gMountDCM[i][1]*wImu[1] + gMountDCM[i][2]*wImu[2];
    aBody[i] = gMountDCM[i][0]*aImu[0] + gMountDCM[i][1]*aImu[1] + gMountDCM[i][2]*aImu[2];
  }
}

// Essential -- unit conversion + calibration (sensor frame), then rotation
// into body frame via rotateToBodyFrame() above.
IMUData readIMU(BMI270& sensor) {
  const float aImu[3] = {
    (sensor.data.accelX * kG0 - kAccelOffset[0]) / kAccelScale[0],
    (sensor.data.accelY * kG0 - kAccelOffset[1]) / kAccelScale[1],
    (sensor.data.accelZ * kG0 - kAccelOffset[2]) / kAccelScale[2],
  };
  const float wImu[3] = {
    sensor.data.gyroX * (float)DEG_TO_RAD - kGyroBias[0],
    sensor.data.gyroY * (float)DEG_TO_RAD - kGyroBias[1],
    sensor.data.gyroZ * (float)DEG_TO_RAD - kGyroBias[2],
  };

  float aBody[3], wBody[3];
  rotateToBodyFrame(aImu, wImu, aBody, wBody);

  IMUData d;
  d.ax = aBody[0]; d.ay = aBody[1]; d.az = aBody[2];
  d.wx = wBody[0]; d.wy = wBody[1]; d.wz = wBody[2];
  return d;
}

// Full 3D attitude + body rate, replacing the 2D sketch's single
// (theta, theta_dot) pair.
// TODO: attitude representation below is a placeholder (quaternion,
// body-to-world, convention TBD) -- revisit if Euler angles turn out to
// be a better fit for the eventual control law.
struct AttitudeState {
  float q[4];   // attitude quaternion: q[0..3], TODO: confirm (w,x,y,z) vs (x,y,z,w) convention
  float w[3];   // body angular rate, rad/s: w[0]=wx, w[1]=wy, w[2]=wz
};

// TODO: 3D attitude/rate estimation from the 6-axis IMU data -- e.g. a
// quaternion complementary filter, Madgwick/Mahony, or an EKF. Mirrors
// calculateState() in the 2D Stage4 sketch (accel-angle + gyro
// complementary filter for a single theta/theta_dot), extended to a full
// 3D attitude. NOTE: a 6-axis IMU alone cannot observe yaw from
// accelerometer -- decide how (or whether) yaw is estimated/controlled
// before implementing this.
//
// Input:  imuData  -- calibrated accel (m/s^2) + gyro (rad/s), from readIMU().
// Output: state    -- updated in place (in/out -- carries the previous
//                      estimate in, like the 2D sketch's static theta/
//                      theta_dot, so the filter has memory across calls).
//
// Safe placeholder until implemented: identity attitude, gyro passed
// through unfiltered.
void calculateState(const IMUData& imuData, AttitudeState& state) {
  state.q[0] = 1.0f; state.q[1] = 0.0f; state.q[2] = 0.0f; state.q[3] = 0.0f;
  state.w[0] = imuData.wx;
  state.w[1] = imuData.wy;
  state.w[2] = imuData.wz;
}


// ----------------------------------------------------------------------------
// SECTION 2c: TELEMETRY
// ----------------------------------------------------------------------------

// Column header for printState() below -- shared so it can be reprinted
// periodically without drifting out of sync with the copy printed at boot.
static const char kHeaderLine[] =
    "t_ms\tq0\tq1\tq2\tq3\twx_dps\twy_dps\twz_dps\t"
    "tau_x\ttau_y\ttau_z\tarmed\tgain_scale\t"
    "wheelX_pos\twheelX_vel\twheelY_pos\twheelY_vel\twheelZ_pos\twheelZ_vel";

// Same idea as the 2D sketch's printState(): one line per control cycle.
// Columns: t_ms, attitude quaternion (q0..q3), body rate (wx/wy/wz, dps),
// commanded torque per wheel (tau_x/y/z), armed, gain_scale, then
// position/velocity per wheel (as reported by each moteus).
void printState(uint32_t t_ms, const AttitudeState& state,
                const float tauCmd[3], bool armed, float gainScale,
                const float wheelPos[3], const float wheelVel[3]) {
  // Header printed ahead of every data line (not just once at boot) so each
  // row is self-labeled in a scrolling Serial Monitor.
  Serial.println(kHeaderLine);

  Serial.print(t_ms);
  for (int i = 0; i < 4; ++i) { Serial.print('\t'); Serial.print(state.q[i], 4); }
  for (int i = 0; i < 3; ++i) { Serial.print('\t'); Serial.print(state.w[i] * (float)RAD_TO_DEG, 2); }
  for (int i = 0; i < 3; ++i) { Serial.print('\t'); Serial.print(tauCmd[i], 4); }
  Serial.print('\t'); Serial.print(armed ? 1 : 0);
  Serial.print('\t'); Serial.print(gainScale, 2);
  for (int i = 0; i < 3; ++i) {
    Serial.print('\t'); Serial.print(wheelPos[i], 3);
    Serial.print('\t'); Serial.print(wheelVel[i], 3);
  }
  Serial.println();
}

// Same fields as printState() above, same order, but plain comma-separated
// with no header/comment lines -- for a PLOTMODE live-plot script (TODO,
// mirror matlab/2Dmodel/Validation/telemetry_matlab.m or
// telemetry_python.py -- both read this exact same format).
void telemetryPlot(uint32_t t_ms, const AttitudeState& state,
                   const float tauCmd[3], bool armed, float gainScale,
                   const float wheelPos[3], const float wheelVel[3]) {
  Serial.print(t_ms);
  for (int i = 0; i < 4; ++i) { Serial.print(','); Serial.print(state.q[i], 6); }
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(state.w[i] * (float)RAD_TO_DEG, 4); }
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(tauCmd[i], 6); }
  Serial.print(','); Serial.print(armed ? 1 : 0);
  Serial.print(','); Serial.print(gainScale, 4);
  for (int i = 0; i < 3; ++i) {
    Serial.print(','); Serial.print(wheelPos[i], 6);
    Serial.print(','); Serial.print(wheelVel[i], 6);
  }
  Serial.println();
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL (commandWheels() is a STUB -- see TODO)
// ----------------------------------------------------------------------------

static bool  gArmed     = false;
static float gGainScale = 1.0f;

// TODO: these are the 2D sketch's single-wheel limits, kept only as
// placeholders so kTorqueFormat/sendWheelTorque below have a concrete
// number to compile against. Re-derive per-wheel for the 3D hardware
// (mirror the 2D Stage 1-3 bring-up process) before arming for real.
static const float kTauMax = 0.12f;

// Per-wheel mounting sign convention -- TODO: calibrate one sign check per
// axis, same method as the 2D sketch's Stage 1 sign check 4.
static const float kWheelSign[3] = { 1.0f, 1.0f, 1.0f };

// Same DEFAULT-wire-format gotcha as the 2D sketch: SetPosition() silently
// drops feedforward_torque/kp_scale/kd_scale/maximum_torque/
// watchdog_timeout unless this Format explicitly turns each one on. Shared
// across all three wheels -- it's a protocol-format constant, not
// per-driver state.
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

// Essential/unchanged -- sends one torque command to one wheel. Factored
// out of the 2D sketch's commandWheel() (which combined this with the
// control law) since here it's called once per wheel, 3x per cycle.
//
// Input:  wheel   -- which driver (gWheels[0..2]).
//         tau     -- commanded torque, N*m, already sign-corrected by the
//                     caller.
//         Output: none (writes to the CAN bus).
void sendWheelTorque(Moteus& wheel, float tau) {
  Moteus::PositionMode::Command cmd;
  cmd.position           = NaN;
  cmd.velocity           = 0.0f;
  cmd.kp_scale           = 0.0f;
  cmd.kd_scale           = 0.0f;
  cmd.feedforward_torque = tau;
  cmd.maximum_torque     = kTauMax;
  cmd.watchdog_timeout   = 0.10f;
  cmd.ignore_position_bounds = 1.0f;  // reaction wheels have no travel limit,
                                       // same reasoning as the 2D sketch.
  wheel.SetPosition(cmd, &kTorqueFormat);
}

// TODO: the actual 3-axis control law (state feedback / LQR / etc.),
// mapping the AttitudeState (4 quaternion + 3 rate terms) and 3 wheel
// speeds to 3 output torques. Mirrors commandWheel() in the 2D Stage4
// sketch (tau = -(k1*theta + k2*theta_dot + k3*wheel_omega) * gGainScale,
// then taper/saturate/trip-on-limit), extended to a MIMO law. When
// implementing this, also move the arm-gating, gGainScale scaling,
// saturation, and safety-limit trips (gArmed = false on overtilt/overspeed
// /non-finite, same as the 2D sketch) in here.
//
// Input:  state       -- current attitude/rate estimate, from calculateState().
//         wheelOmega  -- current wheel speeds, rad/s (wheelOmega[0..2]).
// Output: tauCmd      -- commanded torque per wheel, N*m (tauCmd[0..2]),
//                         written by this function.
//
// Safe placeholder until implemented: always commands zero torque.
void commandWheels(const AttitudeState& state, const float wheelOmega[3],
                   float tauCmd[3]) {
  for (int i = 0; i < 3; ++i) { tauCmd[i] = 0.0f; }
  (void)state;
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

  // Command handling itself always runs, in both telemetry modes -- only
  // the "#" status echoes below are Serial-Monitor-mode only, so a
  // PLOTMODE stream stays clean CSV even while you arm/disarm from the
  // terminal.
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
#if TELEMETRY_MODE == SERIALMONITORMODE
  } else {
    // TODO: 2D's 'o' (mount-offset) command has no 3D equivalent yet --
    // add per-axis calibration commands here once attitude calibration
    // for the 3D rig is worked out.
    Serial.println("# unknown. use: a<0/1>  g<0..1>  h<0/1>");
#endif
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - 3D SKELETON (calculateState/commandWheels not yet implemented)");

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

#if TELEMETRY_MODE == SERIALMONITORMODE
  Serial.println(kHeaderLine);
  Serial.println("# STARTS DISARMED. Send a1 to arm.");
  Serial.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");
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

  static AttitudeState state = { {1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f} };
  calculateState(readIMU(imu), state);

  // --- wheel speeds from the previous cycle's moteus replies ---
  float wheelPos[3];
  float wheelVel[3];      // rev/s, raw from moteus (matches 2D sketch's v.velocity)
  float wheelOmega[3];    // rad/s, sign-corrected -- what commandWheels() consumes
  for (int i = 0; i < 3; ++i) {
    const auto& v = gWheels[i]->last_result().values;
    wheelPos[i] = v.position;
    wheelVel[i] = v.velocity;
    wheelOmega[i] = kWheelSign[i] * v.velocity * 2.0f * (float)PI;   // rev/s -> rad/s
  }

  // --- control (stub) + command ---
  float tauCmd[3];
  commandWheels(state, wheelOmega, tauCmd);
  for (int i = 0; i < 3; ++i) {
    sendWheelTorque(*gWheels[i], kWheelSign[i] * tauCmd[i]);
  }

  // --- telemetry ---
#if TELEMETRY_MODE == PLOTMODE
  telemetryPlot(time, state, tauCmd, gArmed, gGainScale, wheelPos, wheelVel);
#else
  printState(time, state, tauCmd, gArmed, gGainScale, wheelPos, wheelVel);
#endif

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// This is architecture, not a working control loop. Before this sketch does
// anything to real hardware:
//   1. Implement calculateState() -- 3D attitude/rate estimation.
//   2. Implement commandWheels() -- 3-axis control law, including the
//      arm-gating / gGainScale scaling / saturation / safety-trip logic
//      that lives inside commandWheel() in the 2D Stage4 sketch.
//   3. Recalibrate kGyroBias/kAccelOffset/kAccelScale for this IMU mount.
//   4. Calibrate kWheelSign[3] with a per-axis sign check (2D Stage 1
//      method), then define a staged bring-up (mirroring 2D's Stage 0-5)
//      before ever arming with wheels spinning near a human.
// ============================================================================
