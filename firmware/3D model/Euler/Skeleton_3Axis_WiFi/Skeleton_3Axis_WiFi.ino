// ============================================================================
// TEENSY 4.1 + 3x moteus-n1 (CAN3) + BMI270 IMU (SPI) — 3D SKELETON (EULER, WiFi)
// ============================================================================
// Same attitude pipeline, estimator, and control law as
// "firmware/3D model/Euler/Skeleton_3Axis/Skeleton_3Axis.ino" -- this file
// does NOT change any of that physics/estimator/control code. It only adds a
// second telemetry/command channel (Serial1, wired to a XIAO ESP32C6 running
// firmware/XIAO/xiao_teensy_bridge/xiao_teensy_bridge.ino) so the cube can
// run untethered from USB, mirroring
// "firmware/2D model/panel-bringup/Stage4_FullLaw_WiFi/Stage4_FullLaw_WiFi.ino"
// and "firmware/3D model/Quaternion/Skeleton_3Axis_WiFi/Skeleton_3Axis_WiFi.ino".
//
// Skeleton_3Axis.ino (Euler, non-WiFi) is intentionally left untouched -- it
// remains the plain USB-tethered reference/fallback build. This is a
// parallel file.
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
// Same collision as the Quaternion/WiFi sibling: 'h' is HALT here (idle +
// force-disarm, see SECTION 2e), NOT a heartbeat -- a separate no-op
// keepalive, 'k', is used instead. Any line at all already refreshes the
// watchdog (see pollCommands()); 'k' only matters during stretches with no
// other traffic. The PC-side WiFi telemetry script for this sketch must
// send "k" (not "h") as its periodic keepalive.
//
// Wiring to the XIAO (see xiao_teensy_bridge.ino):
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   Common GND between the two boards.
//   Serial1.begin(1000000, ...) -- 1 Mbaud.
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
// Change the line below and re-upload to switch modes.
#define SERIALMONITORMODE 0
#define PLOTMODE 1
#define TELEMETRY_MODE SERIALMONITORMODE


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// TODO: confirm id <-> physical wheel-axis mapping during 3D bring-up.
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
// required. NOT reused as a WiFi heartbeat, see the header note above.

// ---------------- Link mode + command channels ----------------
enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;   // boot default -- see header

const uint32_t kLinkBaud      = 1000000;   // Serial1 <-> XIAO
const uint32_t kLinkTimeoutMs = 300;       // auto-disarm if WIFI mode and no
                                            // Serial1 line in this long
static uint32_t gLastSerial1RxMillis = 0;

// Small non-blocking line accumulator -- used for BOTH Serial and Serial1 so
// neither channel can ever stall loop() waiting on bytes that haven't
// arrived yet.
struct LineReader {
  char buf[64];
  uint8_t len = 0;

  bool poll(Stream& s, char* outLine, size_t outSize) {
    while (s.available()) {
      const char c = (char)s.read();
      if (c == '\n' || c == '\r') {
        if (len == 0) { continue; }
        buf[len] = '\0';
        const uint8_t copyLen = (len < outSize - 1) ? len : (uint8_t)(outSize - 1);
        memcpy(outLine, buf, copyLen);
        outLine[copyLen] = '\0';
        len = 0;
        return true;
      }
      if (len < sizeof(buf) - 1) { buf[len++] = c; }
    }
    return false;
  }
};

static LineReader gUsbReader;
static LineReader gLinkReader;


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION
// ----------------------------------------------------------------------------
// Identical pipeline to Skeleton_3Axis.ino (Euler, non-WiFi) -- see that file
// for the full derivation/comments on every step below.

static const float kG0 = 9.80665f;
// TODO: recalibrate for this IMU mount, see IMU_Calibration.ino.
static const float kGyroBias[3]    = { 0.0f, 0.0f, 0.0f };
static const float kAccelOffset[3] = { 0.0f, 0.0f, 0.0f };
static const float kAccelScale[3]  = { 1.0f, 1.0f, 1.0f };

// STAGE 1: IMU mount transform, intrinsic Z-X-Z. See Skeleton_3Axis.ino for
// the closed-form derivation and the verification matrix.
float theta1_deg = 30.0f;
float theta2_deg = -135.0f;
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

void rotateToBodyFrame(const float aImu[3], const float wImu[3],
                       float aBody[3], float wBody[3]) {
  for (int i = 0; i < 3; ++i) {
    wBody[i] = gMountDCM[i][0]*wImu[0] + gMountDCM[i][1]*wImu[1] + gMountDCM[i][2]*wImu[2];
    aBody[i] = gMountDCM[i][0]*aImu[0] + gMountDCM[i][1]*aImu[1] + gMountDCM[i][2]*aImu[2];
  }
}

// STAGE 0: unit conversion + sensor-frame calibration, at the driver
// boundary only.
void readIMURaw(BMI270& sensor, float aImu[3], float wImu[3]) {
  aImu[0] = (sensor.data.accelX * kG0 - kAccelOffset[0]) / kAccelScale[0];
  aImu[1] = (sensor.data.accelY * kG0 - kAccelOffset[1]) / kAccelScale[1];
  aImu[2] = (sensor.data.accelZ * kG0 - kAccelOffset[2]) / kAccelScale[2];
  wImu[0] = sensor.data.gyroX * (float)DEG_TO_RAD - kGyroBias[0];
  wImu[1] = sensor.data.gyroY * (float)DEG_TO_RAD - kGyroBias[1];
  wImu[2] = sensor.data.gyroZ * (float)DEG_TO_RAD - kGyroBias[2];
}

// STAGE 2/3: control state -- gravity direction, not Euler angles. See
// Skeleton_3Axis.ino for the full rationale.
float g_hat[3] = { 0.0f, 0.0f, 1.0f };
float w_b[3]   = { 0.0f, 0.0f, 0.0f };
float e[3]     = { 0.0f, 0.0f, 0.0f };
float r_vert   = 0.0f;
float dt_s     = 0.0f;

float K_ACC      = 0.02f;   // TODO retune for this loop's actual rate.
float ACC_GATE   = 1.5f;    // m/s^2
float K_YAW_RATE = 0.0f;    // TODO placeholder, see Skeleton_3Axis.ino header.
float Kp         = 0.0f;    // TODO placeholder.
float Kd         = 0.0f;    // TODO placeholder.

static float gGyroBiasBody[3] = { 0.0f, 0.0f, 0.0f };

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

enum class BalanceMode : uint8_t { FACE = 0, EDGE = 1, CORNER = 2 };
static BalanceMode gBalanceMode = BalanceMode::CORNER;

static const float G_FACE[3]   = { 0.0f, 0.0f, 1.0f };
static const float G_EDGE[3]   = { 0.0f, 0.70710678f, 0.70710678f };
static const float G_CORNER[3] = { 0.57735027f, 0.57735027f, 0.57735027f };

void getGRef(BalanceMode mode, float out[3]) {
  const float* src = (mode == BalanceMode::FACE) ? G_FACE
                    : (mode == BalanceMode::EDGE) ? G_EDGE
                                                   : G_CORNER;
  out[0] = src[0]; out[1] = src[1]; out[2] = src[2];
}

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
    float wxg[3];
    cross3(w_b, g_hat, wxg);
    float g_pred[3] = {
      g_hat[0] - wxg[0]*dt,
      g_hat[1] - wxg[1]*dt,
      g_hat[2] - wxg[2]*dt,
    };
    normalize3(g_pred);

    const float aNorm = norm3(aBody);
    const float dev = fabsf(aNorm - kG0);
    float k = K_ACC;
    if (dev > ACC_GATE) { k = 0.0f; }

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

// STAGE 4: Euler angles, telemetry only -- see Skeleton_3Axis.ino header for
// the full rationale (yaw omitted, singularity note).
void eulerForTelemetry(const float gHat[3], float& rollRad, float& pitchRad) {
  rollRad  = atan2f(gHat[1], gHat[2]);
  pitchRad = atan2f(-gHat[0], sqrtf(gHat[1]*gHat[1] + gHat[2]*gHat[2]));
}


// ----------------------------------------------------------------------------
// SECTION 2c: TELEMETRY
// ----------------------------------------------------------------------------
// Both functions below take the destination `Stream&` explicitly -- loop()
// picks Serial or Serial1 each cycle based on gLinkMode. Field content/order
// otherwise identical to Skeleton_3Axis.ino (Euler, non-WiFi).

void printState(Stream& out, uint32_t t_ms, const float tauCmd[3], bool armed, float gainScale,
                float wheelOmegaVert, const float wheelPos[3], const float wheelVel[3]) {
  float rollRad, pitchRad;
  eulerForTelemetry(g_hat, rollRad, pitchRad);

  out.print(t_ms);
  for (int i = 0; i < 3; ++i) { out.print('\t'); out.print(g_hat[i], 4); }
  for (int i = 0; i < 3; ++i) { out.print('\t'); out.print(w_b[i] * (float)RAD_TO_DEG, 2); }
  for (int i = 0; i < 3; ++i) { out.print('\t'); out.print(e[i], 4); }
  out.print('\t'); out.print(r_vert * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(rollRad  * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(pitchRad * (float)RAD_TO_DEG, 2);
  for (int i = 0; i < 3; ++i) { out.print('\t'); out.print(tauCmd[i], 4); }
  out.print('\t'); out.print(armed ? 1 : 0);
  out.print('\t'); out.print(gainScale, 2);
  out.print('\t'); out.print(wheelOmegaVert, 3);
  for (int i = 0; i < 3; ++i) {
    out.print('\t'); out.print(wheelPos[i], 3);
    out.print('\t'); out.print(wheelVel[i], 3);
  }
  out.println();
}

void telemetryPlot(Stream& out, uint32_t t_ms, const float tauCmd[3], bool armed, float gainScale,
                   float wheelOmegaVert, const float wheelPos[3], const float wheelVel[3]) {
  float rollRad, pitchRad;
  eulerForTelemetry(g_hat, rollRad, pitchRad);

  out.print(t_ms);
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(g_hat[i], 6); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(w_b[i] * (float)RAD_TO_DEG, 4); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(e[i], 6); }
  out.print(','); out.print(r_vert * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(rollRad  * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(pitchRad * (float)RAD_TO_DEG, 4);
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(tauCmd[i], 6); }
  out.print(','); out.print(armed ? 1 : 0);
  out.print(','); out.print(gainScale, 4);
  out.print(','); out.print(wheelOmegaVert, 6);
  for (int i = 0; i < 3; ++i) {
    out.print(','); out.print(wheelPos[i], 6);
    out.print(','); out.print(wheelVel[i], 6);
  }
  out.println();
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL (commandWheels() is a STUB -- see TODO)
// ----------------------------------------------------------------------------
// Identical to Skeleton_3Axis.ino (Euler, non-WiFi) -- estimator is wired
// and live, control law is not. Kp/Kd/K_YAW_RATE are declared above (per the
// attitude-spec interface) but not read below.

static bool  gArmed     = false;
static float gGainScale = 1.0f;

static const float kTauMax = 0.12f;

static const float kWheelSign[3] = { 1.0f, 1.0f, 1.0f };

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
  cmd.ignore_position_bounds = 1.0f;
  wheel.SetPosition(cmd, &kTorqueFormat);
}

// TODO: the actual 3-axis control law (state feedback / LQR / etc.), mapping
// g_hat/w_b/e/r_vert (from attitudeUpdate()) and wheelOmega to 3 output
// torques -- see Skeleton_3Axis.ino (Euler, non-WiFi) for the full TODO.
//
// Safe placeholder until implemented: always commands zero torque.
void commandWheels(const float wheelOmega[3], float tauCmd[3]) {
  for (int i = 0; i < 3; ++i) { tauCmd[i] = 0.0f; }
  (void)wheelOmega;
}


// ----------------------------------------------------------------------------
// SECTION 2e: COMMANDS (from either Serial or Serial1)
// ----------------------------------------------------------------------------
// Same a/g/h/m parsing as Skeleton_3Axis.ino, plus 'k' (keepalive/no-op) and
// 't' (link-mode select -- handled before this is even called, see
// pollCommands() below). `echo` is whichever stream the line arrived on.
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
  } else if (cmd == 'm') {
    gBalanceMode = (val < 0.5f) ? BalanceMode::FACE
                 : (val < 1.5f) ? BalanceMode::EDGE
                                : BalanceMode::CORNER;
#if TELEMETRY_MODE == SERIALMONITORMODE
    echo.print("# gBalanceMode = ");
    echo.println(gBalanceMode == BalanceMode::FACE ? "FACE"
                : gBalanceMode == BalanceMode::EDGE ? "EDGE" : "CORNER");
#endif
  } else if (cmd == 'k') {
    // keepalive / no-op -- receipt alone is enough (see pollCommands()).
#if TELEMETRY_MODE == SERIALMONITORMODE
  } else {
    echo.println("# unknown. use: a<0/1>  g<0..1>  h<0/1>  m<0/1/2>  t<0/1>  k");
#endif
  }
}

// Polls BOTH Serial and Serial1 every loop() iteration. 't'-prefixed lines
// switch gLinkMode locally and are NOT passed to handleCommandLine();
// everything else (a/g/h/m/k) is, regardless of which stream it arrived on.
void pollCommands() {
  static char lineBuf[64];

  if (gUsbReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    if (lineBuf[0] == 't') {
      const LinkMode newMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
      if (newMode == LinkMode::WIFI && gLinkMode != LinkMode::WIFI) {
        gLastSerial1RxMillis = millis();
      }
      gLinkMode = newMode;
    } else {
      handleCommandLine(lineBuf, Serial);
    }
  }

  if (gLinkReader.poll(Serial1, lineBuf, sizeof(lineBuf))) {
    gLastSerial1RxMillis = millis();
    if (lineBuf[0] == 't') {
      const LinkMode newMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
      gLinkMode = newMode;
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
  uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) { delay(10); }

  Serial1.begin(kLinkBaud);
  gLastSerial1RxMillis = millis();

  Serial.println("\nstarted - 3D SKELETON (EULER/g_hat, WiFi build -- commandWheels() not yet implemented)");

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
  Serial.println("t_ms\tgx\tgy\tgz\twx_dps\twy_dps\twz_dps\t"
                  "ex\tey\tez\tr_vert_dps\troll_deg\tpitch_deg\t"
                  "tau_x\ttau_y\ttau_z\tarmed\tgain_scale\twheel_omega_vert\t"
                  "wheelX_pos\twheelX_vel\twheelY_pos\twheelY_vel\twheelZ_pos\twheelZ_vel");
  Serial.println("# STARTS DISARMED. Send a1 to arm.");
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
  float wheelVel[3];
  float wheelOmega[3];
  for (int i = 0; i < 3; ++i) {
    const auto& v = gWheels[i]->last_result().values;
    wheelPos[i] = v.position;
    wheelVel[i] = v.velocity;
    wheelOmega[i] = kWheelSign[i] * v.velocity * 2.0f * (float)PI;
  }
  const float wheelOmegaVert = wheelOmega[0]*g_hat[0] + wheelOmega[1]*g_hat[1] + wheelOmega[2]*g_hat[2];

  // --- control + command ---
  float tauCmd[3];
  commandWheels(wheelOmega, tauCmd);
  for (int i = 0; i < 3; ++i) {
    sendWheelTorque(*gWheels[i], kWheelSign[i] * tauCmd[i]);
  }

  // --- telemetry: goes to Serial or Serial1 depending on gLinkMode ---
  Stream& out = (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
#if TELEMETRY_MODE == PLOTMODE
  telemetryPlot(out, time, tauCmd, gArmed, gGainScale, wheelOmegaVert, wheelPos, wheelVel);
#else
  printState(out, time, tauCmd, gArmed, gGainScale, wheelOmegaVert, wheelPos, wheelVel);
#endif

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Companion files:
//   firmware/XIAO/xiao_teensy_bridge/xiao_teensy_bridge.ino  -- the XIAO
//     ESP32C6 sketch that relays Serial1 <-> UDP, reused as-is.
//   matlab/3Dmodel/Validation/  -- TODO: a live-plot script needs to parse
//     this file's CSV (see the SERIALMONITORMODE header string in setup())
//     and send "k" as its periodic keepalive, NOT "h".
//
// Same pre-hardware checklist as Skeleton_3Axis.ino (Euler, non-WiFi)
// applies unchanged -- see that file's NOTES section.
// ============================================================================
