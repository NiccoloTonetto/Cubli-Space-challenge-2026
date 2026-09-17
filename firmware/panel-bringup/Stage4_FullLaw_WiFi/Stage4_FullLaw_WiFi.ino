// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — STAGE 4: FULL LAW (WiFi)
// ============================================================================
// Same control law and hardware wiring as Stage4_FullLaw/Stage4_FullLaw.ino
// -- this file does NOT change the physics/estimator/control code at all.
// It only adds a second telemetry/command channel (Serial1, wired to a XIAO
// ESP32C6 running xiao_teensy_bridge.ino) so the panel can run untethered
// from USB, with the XIAO relaying everything over WiFi/UDP to the laptop.
//
// Stage4_FullLaw.ino itself is intentionally left untouched -- it remains
// the plain USB-tethered reference/fallback build. This is a parallel file.
//
// --------------------------- LINK MODE ---------------------------
// gLinkMode selects which stream (Serial = USB, Serial1 = XIAO) is used
// for telemetry OUT and is watched for the WiFi-link watchdog. It does NOT
// gate which stream commands are read FROM -- both Serial and Serial1 are
// always polled for incoming lines, every loop() iteration, regardless of
// mode, so a mode-switch command is never missed no matter what the board
// is currently doing. Switch it live with:
//   t0   -> USB mode   (telemetry/watchdog on Serial)
//   t1   -> WiFi mode  (telemetry/watchdog on Serial1)      [boot default]
// Recognized on EITHER stream, so you can always flip it from whichever
// channel is currently reachable.
//
// Wiring to the XIAO (see xiao_teensy_bridge.ino):
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   Common GND between the two boards.
//   Serial1.begin(1000000, ...) -- 1 Mbaud. A full PLOTMODE CSV line is
//   ~90 bytes; at 500 Hz that's ~450 kbps needed, so 1 Mbaud leaves
//   headroom (115200, the USB default, would be a 3-4x bottleneck here).
//
// STAGE 4 CHECKLIST — panel held by hand, gGainScale = 1.0:
//   Send "a1" to arm (on either Serial or Serial1).
//   [ ] Wheel unwinds after each correction (watch wheel_omega_lp below --
//       it's wheel_omega through a ~5 s low-pass, i.e. the STANDING speed,
//       not the instantaneous one).
//   [ ] wheel_omega_lp near zero -> healthy.
//   [ ] wheel_omega_lp a few rad/s -> gyro bias or a COM offset. Run the
//       180-DEG FLIP TEST to tell them apart: rotate the panel 180 deg
//       about the pivot axis and re-run.
//         - Standing speed REVERSES SIGN  -> COM offset (re-measure
//           kThetaOffset in Stage 0, or the physical mount).
//         - Standing speed DOES NOT reverse -> gyro bias (re-run the IMU
//           calibration, or accept it and watch it in Stage 5).
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
//                     Monitor (USB or, over the link, whatever terminal
//                     the XIAO's UDP bridge feeds into).
// PLOTMODE:           plain CSV, one line per control cycle, no header/
//                      comment lines -- for telemetry_python_wifi.py (or
//                      the original telemetry_matlab.m / telemetry_python.py
//                      if run over USB in t0/USB mode). Same wire format
//                      regardless of which stream it's sent on.
// Change the line below and re-upload to switch modes.
#define SERIALMONITORMODE 0
#define PLOTMODE 1
#define TELEMETRY_MODE PLOTMODE


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

Moteus moteus1(canBus, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz

static uint32_t gNextSendMillis = 0;

// ---------------- Link mode + command channels ----------------
enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;   // boot default -- see header

const uint32_t kLinkBaud       = 1000000;   // Serial1 <-> XIAO
const uint32_t kLinkTimeoutMs  = 300;       // auto-disarm if WIFI mode and
                                             // no Serial1 line in this long
                                             // (telemetry_python_wifi.py
                                             // sends a heartbeat every
                                             // 100 ms, well under this)
static uint32_t gLastSerial1RxMillis = 0;

// Small non-blocking line accumulator -- used for BOTH Serial and Serial1
// so neither channel can ever stall loop() waiting on bytes that haven't
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
// SECTION 2b: STATE ESTIMATION (unchanged from Stage4_FullLaw.ino)
// ----------------------------------------------------------------------------

struct IMUData {
  float ax, ay, az;
  float wx, wy, wz;
};

static const float kG0 = 9.80665f;
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };
static const float kTau = 1.00f;
static const float kAccelGateLo = 0.85f * kG0;
static const float kAccelGateHi = 1.15f * kG0;

// TODO: carry forward the value measured in Stage 0. This is also the
// value the 180-deg flip test checklist item above may send you back to.
static float kThetaOffset = 0.0f;   // rad

IMUData readIMU(BMI270& sensor) {
  IMUData d;
  d.ax = (sensor.data.accelX * kG0 - kAccelOffset[0]) / kAccelScale[0];
  d.ay = (sensor.data.accelY * kG0 - kAccelOffset[1]) / kAccelScale[1];
  d.az = (sensor.data.accelZ * kG0 - kAccelOffset[2]) / kAccelScale[2];
  d.wx = sensor.data.gyroX * (float)DEG_TO_RAD - kGyroBias[0];
  d.wy = sensor.data.gyroY * (float)DEG_TO_RAD - kGyroBias[1];
  d.wz = sensor.data.gyroZ * (float)DEG_TO_RAD - kGyroBias[2];
  return d;
}

void calculateState(const IMUData& imuData, float& theta, float& theta_dot) {
  static uint32_t lastMicros = 0;
  static bool initialized = false;

  const uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.5f) { dt = kPeriodMs * 1e-3f; }

  theta_dot = imuData.wz;

  const float theta_acc = atan2f(imuData.ax - imuData.ay,
                                 imuData.ax + imuData.ay) - kThetaOffset;

  const float aMag = sqrtf(imuData.ax*imuData.ax +
                           imuData.ay*imuData.ay +
                           imuData.az*imuData.az);
  const bool accelTrusted = (aMag > kAccelGateLo && aMag < kAccelGateHi);

  if (!initialized) {
    theta = theta_acc;
    initialized = true;
    return;
  }

  const float theta_gyro = theta + theta_dot * dt;
  if (accelTrusted) {
    const float alpha = kTau / (kTau + dt);
    theta = alpha * theta_gyro + (1.0f - alpha) * theta_acc;
  } else {
    theta = theta_gyro;
  }
}


// ----------------------------------------------------------------------------
// SECTION 2c: TELEMETRY
// ----------------------------------------------------------------------------
// Both functions below now take the destination `Stream&` explicitly --
// loop() picks Serial or Serial1 each cycle based on gLinkMode. Field
// content/order is otherwise identical to Stage4_FullLaw.ino.

// wheel_omega_lp: wheel_omega through a tau=5s low-pass -- the STANDING
// wheel speed the Stage 4/5 checklists ask you to watch, with the
// per-cycle noise/ripple averaged out.
void printState(Stream& out, uint32_t t_ms, float theta, float theta_dot,
                float tau, float tauCmd, bool armed, float gainScale,
                float wheelOmegaLp, const Moteus::Query::Result& v) {
  out.print(t_ms);
  out.print('\t'); out.print(theta     * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(theta_dot * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(tau, 4);
  out.print('\t'); out.print(tauCmd, 4);
  out.print('\t'); out.print(armed ? 1 : 0);
  out.print('\t'); out.print(gainScale, 2);
  out.print('\t'); out.print(wheelOmegaLp, 3);
  out.print('\t'); out.print(v.position, 3);
  out.print('\t'); out.println(v.velocity, 3);
}

// Same fields as printState() above, same order, but plain comma-separated
// with no header/comment lines -- one clean line per cycle for
// telemetry_python_wifi.py (or telemetry_matlab.m / telemetry_python.py
// over USB in t0/USB mode) to parse with a simple split-on-comma.
void telemetryPlot(Stream& out, uint32_t t_ms, float theta, float theta_dot,
                   float tau, float tauCmd, bool armed, float gainScale,
                   float wheelOmegaLp, const Moteus::Query::Result& v) {
  out.print(t_ms);
  out.print(','); out.print(theta     * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(theta_dot * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(tau, 6);
  out.print(','); out.print(tauCmd, 6);
  out.print(','); out.print(armed ? 1 : 0);
  out.print(','); out.print(gainScale, 4);
  out.print(','); out.print(wheelOmegaLp, 6);
  out.print(','); out.print(v.position, 6);
  out.print(','); out.println(v.velocity, 6);
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL — full law: k1 + k2 + k3 (unchanged from Stage4_FullLaw.ino)
// ----------------------------------------------------------------------------

static const float kK1 = -1.0998f;    // N*m / rad
static const float kK2 = -0.1232f;    // N*m / (rad/s)
static const float kK3 = -0.001732f;  // N*m / (rad/s)

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // already validated by the Stage 3 ramp

static const float kMaxTilt    = 0.52f;
static const float kMaxOmega   = 40.0f;
static const float kTauMax     = 0.12f;
static const float kTaperStart = 36.0f;

// Physical wheel-mounting sign convention -- CONFIRMED backwards on this
// hardware in Stage 1 (sign check 4: a positive pulse pushed the panel
// toward increasing |theta| instead of back toward vertical). Applied to
// BOTH the outgoing torque and the incoming wheel speed, together, so tau
// and wheel_omega stay mutually self-consistent (the taper's spinning_up
// check AND the k3 momentum term both depend on that) while the panel-
// reaction direction gets corrected. K1/K2/K3 are untouched -- this is a
// hardware convention fix, not a control-law change. See Stage 1 for the
// full writeup.
static const float kWheelSign = -1.0f;

// SetPosition()'s DEFAULT wire format only transmits position/velocity --
// every other Command field (feedforward_torque, kp_scale, kd_scale,
// maximum_torque, watchdog_timeout) defaults to Resolution::kIgnore and is
// silently DROPPED before it reaches the CAN bus unless this Format
// explicitly turns each one on. Without it, everything set on `cmd` below
// is real in this C++ struct and never leaves the Teensy: the moteus falls
// back to its own onboard kp_scale=1/kd_scale=1 and just holds velocity=0
// -- plain velocity mode, not torque mode. Confirmed against
// mjbots/moteus-arduino's moteus_protocol.h.
static Moteus::PositionMode::Format kTorqueFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque = Moteus::kFloat;
  f.kp_scale            = Moteus::kFloat;
  f.kd_scale             = Moteus::kFloat;
  f.maximum_torque       = Moteus::kFloat;
  f.watchdog_timeout     = Moteus::kFloat;
  f.ignore_position_bounds = Moteus::kFloat;   // see cmd.ignore_position_bounds
  return f;
}();

static float gLastTau      = 0.0f;
static float gLastTauCmd   = 0.0f;
static float gWheelOmegaLp = 0.0f;   // standing wheel speed, tau = 5 s

void commandWheel(float theta, float theta_dot, float wheel_omega) {

  // full law -- all three terms active.
  float tau = -(kK1 * theta + kK2 * theta_dot + kK3 * wheel_omega) * gGainScale;

  const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
  if (spinning_up) {
    float s = (kMaxOmega - fabsf(wheel_omega)) / (kMaxOmega - kTaperStart);
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    tau *= s;
  }

  if (!isfinite(tau)) { tau = 0.0f; gArmed = false; }

  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  if (fabsf(theta) > kMaxTilt)        { gArmed = false; }
  if (fabsf(wheel_omega) > kMaxOmega) { gArmed = false; }
  if (!isfinite(theta) || !isfinite(theta_dot)) { gArmed = false; }

  const float tau_cmd = gArmed ? tau : 0.0f;

  Moteus::PositionMode::Command cmd;
  cmd.position           = NaN;
  cmd.velocity           = 0.0f;
  cmd.kp_scale           = 0.0f;
  cmd.kd_scale           = 0.0f;
  cmd.feedforward_torque = kWheelSign * tau_cmd;
  cmd.maximum_torque     = kTauMax;
  cmd.watchdog_timeout   = 0.10f;
  cmd.ignore_position_bounds = 1.0f;  // a reaction wheel has no travel limit --
                                       // moteus defaults assume a bounded joint
                                       // and can fault (39, "outside limit")
                                       // entering Position mode outside
                                       // servopos.position_min/_max otherwise.
                                       // See Stage 1 for the full writeup.
  moteus1.SetPosition(cmd, &kTorqueFormat);

  // Standing wheel speed: low-pass, tau = 5 s. Nominal dt (kPeriodMs) is
  // fine here -- this filter only needs to be accurate to within seconds,
  // not milliseconds.
  static const float kLpTau = 5.0f;
  const float dtNom = kPeriodMs * 1e-3f;
  const float alphaLp = dtNom / (kLpTau + dtNom);
  gWheelOmegaLp += alphaLp * (wheel_omega - gWheelOmegaLp);

  gLastTau    = tau;
  gLastTauCmd = tau_cmd;
}


// ----------------------------------------------------------------------------
// SECTION 2e: COMMANDS (from either Serial or Serial1)
// ----------------------------------------------------------------------------
// handleCommandLine() is the same a/g/o parsing Stage4_FullLaw.ino has,
// plus 'h' (heartbeat/no-op -- see LineReader/gLastSerial1RxMillis above)
// and 't' (link-mode select -- handled before this is even called, see
// pollCommands() below). `echo` is whichever stream the line arrived on,
// so a SERIALMONITORMODE ack goes back down the same channel it came in on.
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
  } else if (cmd == 'o') {
    kThetaOffset = val * (float)DEG_TO_RAD;
#if TELEMETRY_MODE == SERIALMONITORMODE
    echo.print("# kThetaOffset = "); echo.print(val, 4); echo.println(" deg");
#endif
  } else if (cmd == 'h') {
    // heartbeat / no-op -- receipt alone is enough (see pollCommands()).
#if TELEMETRY_MODE == SERIALMONITORMODE
  } else {
    echo.println("# unknown. use: a<0/1>  g<0..1>  o<deg>  t<0/1>  h");
#endif
  }
}

// Polls BOTH Serial and Serial1 every loop() iteration -- see the
// LineReader comment above for why this can never block. 't'-prefixed
// lines switch gLinkMode locally and are NOT passed to handleCommandLine();
// everything else (a/g/o/h) is, regardless of which stream it arrived on.
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
  // Bounded wait, NOT while(!Serial){} -- Stage4_FullLaw.ino's USB-tethered
  // build can afford to hang here since a laptop is always attached; this
  // WiFi build must still boot into WIFI mode with nothing on USB at all.
  uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) { delay(10); }

  Serial1.begin(kLinkBaud);
  gLastSerial1RxMillis = millis();

  Serial.println("\nstarted - STAGE 4: FULL LAW (WiFi build, panel held by hand)");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  moteus1.SetStop();
  Serial.println("all stopped");

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
  Serial.println("t_ms\ttheta_deg\ttheta_dot_dps\ttau_Nm\ttau_cmd_Nm\tarmed\t"
                  "gain_scale\twheel_omega_lp\twheel_pos\twheel_vel");
  Serial.println("# STARTS DISARMED. Send a1 to arm. o<deg> sets mount offset.");
  Serial.println("# t0 = USB link mode, t1 = WiFi link mode (default t1).");
#endif

  gNextSendMillis = millis();

}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {

  pollCommands();

  // WiFi-mode link watchdog -- only enforced in WIFI mode, mirrors what an
  // explicit a0 does. USB mode never auto-disarms from this (matches
  // Stage4_FullLaw.ino: only a0 or the control law's own safety trips do).
  if (gLinkMode == LinkMode::WIFI && gArmed &&
      millis() - gLastSerial1RxMillis > kLinkTimeoutMs) {
    gArmed = false;
  }

  if (static_cast<int32_t>(millis() - gNextSendMillis) < 0) { return; }
  gNextSendMillis += kPeriodMs;

  const uint32_t time = millis();

  imu.getSensorData();

  static float theta = 0.0f;
  static float theta_dot = 0.0f;
  calculateState(readIMU(imu), theta, theta_dot);

  // --- wheel speed from the previous cycle's moteus reply ---
  const auto& v = moteus1.last_result().values;
  const float wheel_omega = kWheelSign * v.velocity * 2.0f * (float)PI;   // rev/s -> rad/s

  // --- control + command ---
  commandWheel(theta, theta_dot, wheel_omega);

  // --- telemetry: goes to Serial or Serial1 depending on gLinkMode ---
  Stream& out = (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
#if TELEMETRY_MODE == PLOTMODE
  telemetryPlot(out, time, theta, theta_dot, gLastTau, gLastTauCmd, gArmed,
                gGainScale, gWheelOmegaLp, v);
#else
  printState(out, time, theta, theta_dot, gLastTau, gLastTauCmd, gArmed,
             gGainScale, gWheelOmegaLp, v);
#endif

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Companion files:
//   firmware/XIAO/xiao_teensy_bridge/xiao_teensy_bridge.ino  -- the XIAO
//     ESP32C6 sketch that relays Serial1 <-> UDP.
//   matlab/2Dmodel/Validation/telemetry_python_wifi.py  -- the PC-side
//     viewer/logger/command console for this build (UDP instead of a
//     direct COM port).
// Stage4_FullLaw.ino, xiao_esp32c6_wifi_test1.ino, xiao_laptop_listener1.py,
// telemetry_python.py and telemetry_matlab.m are all untouched by this and
// remain the USB-tethered / standalone-WiFi-test reference versions.
// ============================================================================
