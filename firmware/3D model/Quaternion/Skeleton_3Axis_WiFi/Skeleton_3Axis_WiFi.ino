// ============================================================================
// TEENSY 4.1 + 3x moteus-n1 (CAN3) + BMI270 IMU (SPI) — 3D SKELETON (WiFi)
// ============================================================================
// Same architecture, estimator stub, control stub, and IMU mount transform
// as Skeleton_3Axis.ino -- this file does NOT change any of that physics/
// estimator/control code. It only adds a second telemetry/command channel
// (Serial1, wired to a XIAO ESP32C6 running
// firmware/XIAO/xiao_teensy_bridge/xiao_teensy_bridge.ino) so the cube can
// run untethered from USB, mirroring
// "firmware/2D model/panel-bringup/Stage4_FullLaw_WiFi/Stage4_FullLaw_WiFi.ino".
//
// Skeleton_3Axis.ino itself is intentionally left untouched -- it remains
// the plain USB-tethered reference/fallback build. This is a parallel file.
//
// --------------------------- LINK MODE ---------------------------
// gLinkMode selects which stream (Serial = USB, Serial1 = XIAO) is used for
// telemetry OUT and is watched for the WiFi-link watchdog. It does NOT gate
// which stream commands are read FROM -- both Serial and Serial1 are always
// polled for incoming lines, every loop() iteration, so a mode-switch
// command is never missed. Switch it live with:
//   t0   -> USB mode   (telemetry/watchdog on Serial)
//   t1   -> WiFi mode  (telemetry/watchdog on Serial1)      [boot default]
// Recognized on EITHER stream.
//
// --------------------------- COMMAND GRAMMAR: 'h' COLLISION -------------
// The 2D WiFi build (Stage4_FullLaw_WiFi.ino) uses 'h' as a no-op heartbeat
// to keep the WiFi-link watchdog alive during idle periods. This sketch
// CANNOT reuse that: Skeleton_3Axis.ino already defines 'h' as HALT (idle +
// force-disarm, see SECTION 2e) -- an untethered PC script sending a 2D-style
// "h" heartbeat here would silently halt and disarm the cube instead of
// doing nothing. So the halt semantics of 'h' are kept EXACTLY as in
// Skeleton_3Axis.ino, and a separate no-op heartbeat command, 'k'
// (keepalive), is added instead. Any line at all (not just 'k') already
// refreshes the watchdog -- see pollCommands() -- so 'k' only matters during
// stretches with no other traffic; the PC-side WiFi telemetry script for
// this sketch must send "k" (not "h") as its periodic keepalive.
//
// Wiring to the XIAO (see xiao_teensy_bridge.ino):
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   Common GND between the two boards.
//   Serial1.begin(1000000, ...) -- 1 Mbaud. A full PLOTMODE CSV line here
//   carries ~18 fields (quaternion + 3-axis rate + 3 torques + armed/gain +
//   3x wheel pos/vel) vs the 2D build's 9 -- still comfortably under 1 Mbaud
//   at 500 Hz, but worth a bandwidth sanity check once running.
//
// Does NOT compile into a bring-up stage yet, same as Skeleton_3Axis.ino --
// calculateState()/commandWheels() are still empty stubs. See that file's
// NOTES section for the pre-hardware checklist; it applies here unchanged.
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
// SERIALMONITORMODE: human-readable, tab-delimited, with header + "#"
//                     status lines -- for reading directly in a Serial
//                     Monitor (USB or, over the link, whatever terminal the
//                     XIAO's UDP bridge feeds into).
// PLOTMODE:           plain CSV, one line per control cycle, no header/
//                      comment lines -- for a matlab/3Dmodel/Validation
//                      live-plot script (TODO, mirror
//                      matlab/2Dmodel/Validation/telemetry_python_wifi.py).
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
// TODO: confirm id <-> physical wheel-axis mapping during 3D bring-up
// (set/verified per-driver via tview, same as the 2D sketch's moteus1).
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
// required. Same semantics as Skeleton_3Axis.ino -- NOT reused as a WiFi
// heartbeat, see the header note above.

// ---------------- Link mode + command channels ----------------
enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;   // boot default -- see header

const uint32_t kLinkBaud      = 1000000;   // Serial1 <-> XIAO
const uint32_t kLinkTimeoutMs = 300;       // auto-disarm if WIFI mode and no
                                            // Serial1 line in this long
static uint32_t gLastSerial1RxMillis = 0;

// Small non-blocking line accumulator -- used for BOTH Serial and Serial1 so
// neither channel can ever stall loop() waiting on bytes that haven't
// arrived yet (Stream::readStringUntil() can block up to its timeout on a
// partial line; at 500 Hz / 2 ms per cycle that's not acceptable here).
struct LineReader {
  char buf[64];
  uint8_t len = 0;

  // Returns true (with outLine filled + NUL-terminated) at most once per
  // call, i.e. at most one complete line consumed per poll() -- bounded
  // work per loop() iteration. Any remaining queued lines are picked up on
  // the following iteration(s).
  bool poll(Stream& s, char* outLine, size_t outSize) {
    while (s.available()) {
      const char c = (char)s.read();
      if (c == '\n' || c == '\r') {
        if (len == 0) { continue; }   // swallow CRLF / blank artifacts
        buf[len] = '\0';
        const uint8_t copyLen = (len < outSize - 1) ? len : (uint8_t)(outSize - 1);
        memcpy(outLine, buf, copyLen);
        outLine[copyLen] = '\0';
        len = 0;
        return true;
      }
      if (len < sizeof(buf) - 1) { buf[len++] = c; }
      // else: line too long -- drop the overflow char, keep accumulating
      // until the terminator so we resync on the next line.
    }
    return false;
  }
};

static LineReader gUsbReader;
static LineReader gLinkReader;


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION (calculateState() is a STUB -- see TODO)
// ----------------------------------------------------------------------------
// Unchanged from Skeleton_3Axis.ino -- IMU mount transform + calibration are
// identical, see that file for the full derivation/comments.

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
// See Skeleton_3Axis.ino for the full derivation. Summary: the IMU sits at
// the cube's geometric centre (no measurable offset), so the only
// correction needed is a ROTATION -- gMountDCM rotates sensor-axis readings
// into body axes (intrinsic Z-X-Z mount sequence). There is no lever arm,
// so no omega-x-r/alpha-x-r terms are needed for the accelerometer either
// (the gyro was never subject to this: angular velocity is identical at
// every point of a rigid body).
// gTheta1Deg/gTheta2Deg/gTheta3Deg = 0, 54.7356, 45 are the EXACT equivalent
// of this mount's known corner geometry (sensor Z toward corner G, sensor X
// through the FB/DH edge midpoints) -- changes nothing about current
// behaviour, just makes the mount angle runtime-tunable. GIMBAL LOCK at
// theta2 = 0/180 deg -- not a concern at 54.7356.
float gTheta1Deg = 0.0f;
float gTheta2Deg = 54.7356f;
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
// body-to-world, convention TBD) -- revisit if Euler angles turn out to be
// a better fit for the eventual control law.
struct AttitudeState {
  float q[4];   // attitude quaternion: q[0..3], TODO: confirm (w,x,y,z) vs (x,y,z,w) convention
  float w[3];   // body angular rate, rad/s: w[0]=wx, w[1]=wy, w[2]=wz
};

// TODO: 3D attitude/rate estimation from the 6-axis IMU data -- e.g. a
// quaternion complementary filter, Madgwick/Mahony, or an EKF. NOTE: a
// 6-axis IMU alone cannot observe yaw from accelerometer -- decide how (or
// whether) yaw is estimated/controlled before implementing this.
//
// Input:  imuData  -- calibrated, body-frame accel (m/s^2) + gyro (rad/s),
//                      from readIMU().
// Output: state    -- updated in place (in/out -- carries the previous
//                      estimate in, so the filter has memory across calls).
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
// Both functions below now take the destination `Stream&` explicitly --
// loop() picks Serial or Serial1 each cycle based on gLinkMode. Field
// content/order is otherwise identical to Skeleton_3Axis.ino.

// Column header for printState() below -- shared so it can be reprinted
// periodically without drifting out of sync with the copy printed at boot.
static const char kHeaderLine[] =
    "t_ms\tq0\tq1\tq2\tq3\twx_dps\twy_dps\twz_dps\t"
    "tau_x\ttau_y\ttau_z\tarmed\tgain_scale\t"
    "wheelX_pos\twheelX_vel\twheelY_pos\twheelY_vel\twheelZ_pos\twheelZ_vel";

void printState(Stream& out, uint32_t t_ms, const AttitudeState& state,
                const float tauCmd[3], bool armed, float gainScale,
                const float wheelPos[3], const float wheelVel[3]) {
  // Header printed ahead of every data line (not just once at boot) so each
  // row is self-labeled in a scrolling Serial Monitor.
  out.println(kHeaderLine);

  out.print(t_ms);
  for (int i = 0; i < 4; ++i) { out.print('\t'); out.print(state.q[i], 4); }
  for (int i = 0; i < 3; ++i) { out.print('\t'); out.print(state.w[i] * (float)RAD_TO_DEG, 2); }
  for (int i = 0; i < 3; ++i) { out.print('\t'); out.print(tauCmd[i], 4); }
  out.print('\t'); out.print(armed ? 1 : 0);
  out.print('\t'); out.print(gainScale, 2);
  for (int i = 0; i < 3; ++i) {
    out.print('\t'); out.print(wheelPos[i], 3);
    out.print('\t'); out.print(wheelVel[i], 3);
  }
  out.println();
}

// Same fields as printState() above, same order, but plain comma-separated
// with no header/comment lines -- for a PLOTMODE live-plot script (TODO,
// mirror matlab/2Dmodel/Validation/telemetry_python_wifi.py).
void telemetryPlot(Stream& out, uint32_t t_ms, const AttitudeState& state,
                   const float tauCmd[3], bool armed, float gainScale,
                   const float wheelPos[3], const float wheelVel[3]) {
  out.print(t_ms);
  for (int i = 0; i < 4; ++i) { out.print(','); out.print(state.q[i], 6); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(state.w[i] * (float)RAD_TO_DEG, 4); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(tauCmd[i], 6); }
  out.print(','); out.print(armed ? 1 : 0);
  out.print(','); out.print(gainScale, 4);
  for (int i = 0; i < 3; ++i) {
    out.print(','); out.print(wheelPos[i], 6);
    out.print(','); out.print(wheelVel[i], 6);
  }
  out.println();
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL (commandWheels() is a STUB -- see TODO)
// ----------------------------------------------------------------------------
// Unchanged from Skeleton_3Axis.ino.

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

// Essential/unchanged -- sends one torque command to one wheel.
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
// speeds to 3 output torques. When implementing this, also move the
// arm-gating, gGainScale scaling, saturation, and safety-limit trips
// (gArmed = false on overtilt/overspeed/non-finite) in here.
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
// SECTION 2e: COMMANDS (from either Serial or Serial1)
// ----------------------------------------------------------------------------
// handleCommandLine() is the same a/g/h parsing Skeleton_3Axis.ino has,
// plus 'k' (keepalive/no-op -- see LineReader/gLastSerial1RxMillis above,
// and the header note on why this is 'k' and NOT 'h' here) and 't'
// (link-mode select -- handled before this is even called, see
// pollCommands() below). `echo` is whichever stream the line arrived on, so
// a SERIALMONITORMODE ack goes back down the same channel it came in on.
void handleCommandLine(const char* line, Stream& echo) {
  if (line[0] == '\0') { return; }

  const char cmd = line[0];
  const float val = atof(line + 1);

  if (cmd == 'a') {
    gArmed = (val != 0.0f);
#if TELEMETRY_MODE == SERIALMONITORMODE
    echo.print("# gArmed = "); echo.println(gArmed ? "TRUE" : "FALSE");
#endif
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
#if TELEMETRY_MODE == SERIALMONITORMODE
    echo.print("# gGainScale = "); echo.println(gGainScale, 3);
#endif
  } else if (cmd == 'h') {
    gHalted = (val != 0.0f);
    if (gHalted && gArmed) {
      gArmed = false;
#if TELEMETRY_MODE == SERIALMONITORMODE
      echo.println("# disarmed by halt");
#endif
    }
#if TELEMETRY_MODE == SERIALMONITORMODE
    echo.print("# gHalted = ");
    echo.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                          : "FALSE (resumed, still DISARMED -- send a1)");
#endif
  } else if (cmd == 'k') {
    // keepalive / no-op -- receipt alone is enough (see pollCommands()).
    // NOT 'h' here -- 'h' is HALT, see the file header note.
#if TELEMETRY_MODE == SERIALMONITORMODE
  } else {
    echo.println("# unknown. use: a<0/1>  g<0..1>  h<0/1>  t<0/1>  k");
#endif
  }
}

// Polls BOTH Serial and Serial1 every loop() iteration -- see the
// LineReader comment above for why this can never block. 't'-prefixed
// lines switch gLinkMode locally and are NOT passed to handleCommandLine();
// everything else (a/g/h/k) is, regardless of which stream it arrived on.
void pollCommands() {
  static char lineBuf[64];

  if (gUsbReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    if (lineBuf[0] == 't') {
      const LinkMode newMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
      if (newMode == LinkMode::WIFI && gLinkMode != LinkMode::WIFI) {
        gLastSerial1RxMillis = millis();   // fresh grace period on entry
      }
      gLinkMode = newMode;
    } else {
      handleCommandLine(lineBuf, Serial);
    }
  }

  if (gLinkReader.poll(Serial1, lineBuf, sizeof(lineBuf))) {
    gLastSerial1RxMillis = millis();   // any line at all counts as link-alive
    if (lineBuf[0] == 't') {
      const LinkMode newMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
      gLinkMode = newMode;   // already just heard from Serial1, so no need
                              // to reset the grace period here
    } else {
      handleCommandLine(lineBuf, Serial1);
    }
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  // Bounded wait, NOT while(!Serial){} -- Skeleton_3Axis.ino's USB-tethered
  // build can afford to hang here since a laptop is always attached; this
  // WiFi build must still boot into WIFI mode with nothing on USB at all.
  uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) { delay(10); }

  Serial1.begin(kLinkBaud);
  gLastSerial1RxMillis = millis();

  Serial.println("\nstarted - 3D SKELETON (WiFi build, calculateState/commandWheels not yet implemented)");

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
  Serial.println("# t0 = USB link mode, t1 = WiFi link mode (default t1). k = keepalive.");
  Serial.println("# NOTE: commandWheels() is a stub -- always commands zero torque.");
#endif

  gNextSendMillis = millis();

}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {

  pollCommands();

  // WiFi-mode link watchdog -- only enforced in WIFI mode, mirrors what an
  // explicit a0 does. USB mode never auto-disarms from this.
  if (gLinkMode == LinkMode::WIFI && gArmed &&
      millis() - gLastSerial1RxMillis > kLinkTimeoutMs) {
    gArmed = false;
  }

  // Halted: skip IMU reads, CAN traffic, and telemetry entirely. pollCommands()
  // above still runs every pass, so "h0" always gets through.
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

  // --- telemetry: goes to Serial or Serial1 depending on gLinkMode ---
  Stream& out = (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
#if TELEMETRY_MODE == PLOTMODE
  telemetryPlot(out, time, state, tauCmd, gArmed, gGainScale, wheelPos, wheelVel);
#else
  printState(out, time, state, tauCmd, gArmed, gGainScale, wheelPos, wheelVel);
#endif

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Companion files:
//   firmware/XIAO/xiao_teensy_bridge/xiao_teensy_bridge.ino  -- the XIAO
//     ESP32C6 sketch that relays Serial1 <-> UDP. Reused as-is from the 2D
//     build -- it just relays bytes, it has no opinion on command grammar.
//   matlab/3Dmodel/Validation/  -- TODO: a 3D analogue of
//     telemetry_python_wifi.py needs to (a) parse this file's ~18-field CSV
//     instead of the 2D build's 9 fields, and (b) send "k" as its periodic
//     keepalive, NOT "h" -- see the file header note above.
//
// Same pre-hardware checklist as Skeleton_3Axis.ino applies unchanged:
//   1. Implement calculateState() -- 3D attitude/rate estimation.
//   2. Implement commandWheels() -- 3-axis control law + arm-gating/
//      saturation/safety-trip logic.
//   3. Recalibrate kGyroBias/kAccelOffset/kAccelScale for this IMU mount.
//   4. Calibrate kWheelSign[3] with a per-axis sign check, then define a
//      staged bring-up before ever arming with wheels spinning near a human.
// ============================================================================
