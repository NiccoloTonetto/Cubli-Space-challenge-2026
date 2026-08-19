// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — EDGE BALANCE:
// FULL LAW -- AXIS-SELECTABLE, WiFi BUILD
// ============================================================================
// Same physics, estimator, edge-candidate table, control law, gains, trips
// and arm gate as ../../USBMODE/EdgeBalance/EdgeBalance.ino (which is itself
// a byte-identical copy of Gam/edge-bringup/Stage4_FullLaw/Stage4_FullLaw.ino,
// the hardware-validated USB build). NOTHING in the control path is changed
// here.
//
// !! THIS WiFi BUILD HAS NOT BEEN VALIDATED ON HARDWARE YET. !!
// The USB build it derives from has flown; the WiFi link layer below has
// never been exercised on the cube. Treat the first run as a bring-up, not a
// repeat: hand-held, one hand on the cube, a0 ready.
//
// What this file adds, and only this:
//   - a second telemetry/command channel on Serial1, wired to a XIAO ESP32C6
//     running ../xiao_teensy_bridge/xiao_teensy_bridge.ino, so the cube can
//     balance on its edge untethered from USB;
//   - a link watchdog that auto-disarms if the WiFi link goes quiet;
//   - non-blocking line reading on BOTH streams (the USB build's
//     Serial.readStringUntil() can block up to its timeout on a partial line,
//     which is not acceptable inside a 2 ms control cycle);
//   - boot diagnostics mirrored to both streams, so an untethered bring-up
//     still shows the CAN check, IMU status, the "hold still" gyro-calibration
//     prompt and the resolved edge candidate in the WiFi console.
//
// The sibling files this mirrors, structurally:
//   ../CornerBalance_WiFi/CornerBalance_WiFi.ino  -- same link layer, 3 wheels
//   firmware/2D model/panel-bringup/Stage4_FullLaw_WiFi/Stage4_FullLaw_WiFi.ino
//   firmware/3D model/Gam/Skeleton_3Axis_WiFi/Skeleton_3Axis_WiFi.ino
//
// GRAMMAR NOTE vs the corner build: here 'h'/'k' are the keepalive and there
// is no halt command at all. CornerBalance_WiFi.ino keeps a halt, bound to
// 'p' (its USB build uses 'h' for halt, which collides with the keepalive).
//
// --------------------------- LINK MODE ---------------------------
// gLinkMode selects which stream (Serial = USB, Serial1 = XIAO) telemetry
// goes OUT on and which one the watchdog watches. It does NOT gate which
// stream commands are read FROM -- both Serial and Serial1 are polled every
// loop() iteration, so a mode-switch command is never missed no matter what
// the board is currently doing. Switch it live with:
//   t0   -> USB mode   (telemetry/watchdog on Serial, no auto-disarm)
//   t1   -> WiFi mode  (telemetry/watchdog on Serial1)      [boot default]
// Recognized on EITHER stream, so you can always flip it from whichever
// channel is currently reachable.
//
// --------------------------- COMMAND GRAMMAR ---------------------------
//   a<0/1>   arm / disarm      (a1 still passes through the 0.5 deg arm gate)
//   g<0..1>  gain scale
//   o<deg>   kPhiOffset -- per axis, live
//   e        re-resolve the edge candidate, answer echoed back
//   t<0/1>   link mode (USB / WiFi)
//   h  or  k no-op keepalive
// 'h' is free here (unlike Skeleton_3Axis_WiFi.ino, where 'h' is HALT), so
// BOTH 'h' and 'k' are accepted as the keepalive. That means the existing
// matlab/2Dmodel/Validation/telemetry_python_wifi.py, which sends "h" every
// 100 ms, works against this sketch unmodified.
//
// Wiring to the XIAO (see xiao_teensy_bridge.ino):
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   Common GND between the two boards.
//   1 Mbaud. A PLOTMODE CSV line is ~90 bytes; at 500 Hz that's ~450 kbps,
//   so 1 Mbaud leaves headroom (115200 would be a 3-4x bottleneck).
//
// STAGE 4 CHECKLIST — cube held by hand, gGainScale = 1.0 (unchanged):
//   Position near the edge equilibrium (|phi_edge| < 0.5 deg) BEFORE
//   sending "a1" -- the gate will otherwise refuse to arm and tell you so.
//   [ ] Wheel unwinds after each correction (watch wheel_omega_lp).
//   [ ] wheel_omega_lp near zero -> healthy.
//   [ ] wheel_omega_lp a few rad/s and not settling -> re-check gyro bias
//       or the mount/COM assumption (Firmware Lessons S8's 180-deg flip test).
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
// SERIALMONITORMODE: human-readable, tab-delimited, with header + "#" status
//                     lines -- for reading directly in a Serial Monitor (USB,
//                     or whatever terminal the XIAO's UDP bridge feeds).
// PLOTMODE:           plain CSV, one line per control cycle -- for
//                      matlab/2Dmodel/Validation/telemetry_python_wifi.py.
// Defaults to PLOTMODE here (the USB build defaults to SERIALMONITORMODE):
// the whole point of the WiFi build is running with the PC-side plotter, and
// the 10 CSV fields below are the same shape that script already parses.
#define SERIALMONITORMODE 0
#define PLOTMODE 1
#define TELEMETRY_MODE PLOTMODE


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// Confirmed on bench, physically verified (Gam/Skeleton_3Axis.ino's mapping
// comment): id 2 -> X, id 3 -> Y, id 1 -> Z.
enum Axis { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };
static const Axis kAxis = AXIS_Z;   // <<< CHANGE THIS to test X or Z

static const int8_t kAxisMoteusId[3] = { 2, 3, 1 };
static const char*  kAxisName[3]     = { "X", "Y", "Z" };

Moteus moteusActive(canBus, []() {
  Moteus::Options options;
  options.id = kAxisMoteusId[kAxis];
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

const uint32_t kLinkBaud      = 1000000;   // Serial1 <-> XIAO
const uint32_t kLinkTimeoutMs = 3000;       // auto-disarm if WIFI mode and no
                                            // Serial1 line in this long
                                            // (telemetry_python_wifi.py sends
                                            // a keepalive every 100 ms, well
                                            // under this)
static uint32_t gLastSerial1RxMillis = 0;

// Small non-blocking line accumulator -- used for BOTH Serial and Serial1 so
// neither channel can ever stall loop() waiting on bytes that haven't
// arrived yet. Returns true (outLine filled + NUL-terminated) at most once
// per call, i.e. bounded work per loop() iteration; queued lines are picked
// up on the following iteration(s).
struct LineReader {
  char buf[64];
  uint8_t len = 0;

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

// Boot/diagnostic sink: writes to USB and to the XIAO link at the same time.
// Only used outside the control path (setup(), and the 'e' command's echo)
// -- per-cycle telemetry still goes to exactly one stream, chosen by
// gLinkMode, so the PC-side CSV parser never sees doubled lines.
class DualPrint : public Print {
 public:
  using Print::write;   // keep the base write(const char*) overload visible
  size_t write(uint8_t c) override {
    Serial.write(c);
    Serial1.write(c);
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
    Serial1.write(b, n);
    return n;
  }
};
static DualPrint gBoot;


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from Stage4_FullLaw.ino)
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

// Takes the sink explicitly so the same check is readable over WiFi at boot.
void checkMountingDCMValid(Print& out) {
  const float (&C)[3][3] = gMountDCM;
  const float det = C[0][0]*(C[1][1]*C[2][2] - C[1][2]*C[2][1])
                   - C[0][1]*(C[1][0]*C[2][2] - C[1][2]*C[2][0])
                   + C[0][2]*(C[1][0]*C[2][1] - C[1][1]*C[2][0]);
  out.print("# mount DCM check: det="); out.print(det, 4);
  for (int i = 0; i < 3; ++i) {
    const float len = sqrtf(C[i][0]*C[i][0] + C[i][1]*C[i][1] + C[i][2]*C[i][2]);
    out.print("  |row"); out.print(i); out.print("|="); out.print(len, 4);
  }
  out.println();
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

static const float kP_FILT = 4.0f;
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
  for (int i = 0; i < 3; ++i) { bhat[i] += kI_FILT * e[i] * dt; }
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
// SECTION 2c: EDGE CANDIDATE (unchanged from Stage4_FullLaw.ino)
// ----------------------------------------------------------------------------

struct EdgeCandidate {
  const char* name;
  float e[3];
  float gB[3];
  float K[3];
  float placeOffsetDeg;
};

static const EdgeCandidate kCandidates[3][2] = {
  // AXIS_X (id 2) -- X[+1,+1] and X[-1,-1], offsets 0.758/0.837 deg
  { { "X[+1,+1] (+Y,+Z up)", { 1.0f, 0.0f, 0.0f }, { -0.0f, 0.697691619f, 0.716398239f },
      { -9.22453213f, -1.09680569f, -0.00692820316f }, 0.758f },
    { "X[-1,-1] (-Y,-Z up)", { 1.0f, 0.0f, 0.0f }, { -0.0f, -0.717363894f, -0.696698666f },
      { -8.2052536f, -0.948087275f, -0.00692820316f }, 0.837f } },
  // AXIS_Y (id 3) -- Y[+1,+1] and Y[-1,-1], offsets 0.006/0.007 deg (best on the cube)
  { { "Y[+1,+1] (+X,+Z up)", { 0.0f, 1.0f, 0.0f }, { 0.707182407f, -0.0f, 0.70703119f },
      { -9.33804893f, -1.10871983f, -0.00692820316f }, 0.006f },
    { "Y[-1,-1] (-X,-Z up)", { 0.0f, 1.0f, 0.0f }, { -0.707020879f, -0.0f, -0.707192659f },
      { -8.03074265f, -0.918068051f, -0.00692820316f }, 0.007f } },
  // AXIS_Z (id 1) -- Z[+1,+1] and Z[-1,-1], offsets 0.764/0.844 deg
  { { "Z[+1,+1] (+X,+Y up)", { 0.0f, 0.0f, 1.0f }, { 0.716472805f, 0.697615027f, -0.0f },
      { -9.22558498f, -1.09693909f, -0.00692820316f }, 0.764f },
    { "Z[-1,-1] (-X,-Y up)", { 0.0f, 0.0f, 1.0f }, { -0.696611583f, -0.717448473f, -0.0f },
      { -8.20397282f, -0.947880447f, -0.00692820316f }, 0.844f } },
};

int gEdgeIdx = 0;

// Same measurement-based resolution as the USB build; only the print sink is
// parameterised, so the 'e' command's answer comes back on whichever channel
// asked for it.
void resolveEdgeCandidate(Print& out) {
  const float d0 = dot3(ghat, kCandidates[kAxis][0].gB);
  const float d1 = dot3(ghat, kCandidates[kAxis][1].gB);
  gEdgeIdx = (d0 >= d1) ? 0 : 1;
  out.print("# axis="); out.print(kAxisName[kAxis]);
  out.print("  edge candidate resolved: "); out.print(kCandidates[kAxis][gEdgeIdx].name);
  out.print("  K="); out.print(kCandidates[kAxis][gEdgeIdx].K[0], 4);
  out.print(","); out.print(kCandidates[kAxis][gEdgeIdx].K[1], 4);
  out.print(","); out.print(kCandidates[kAxis][gEdgeIdx].K[2], 5);
  out.print("  (dot0="); out.print(d0, 4);
  out.print(" dot1="); out.print(d1, 4); out.println(")");
  if (fabsf(d0 - d1) < 0.2f) {
    out.println("# WARNING: dot0 and dot1 are close -- verify before arming.");
  }
}

float phi_edge = 0.0f;
float om_edge  = 0.0f;

// Live-settable, NOT a sensor/mount calibration (that's already handled by
// gMountDCM and the accel/gyro constants above) -- this corrects for the
// cube's CURRENT physical resting point being genuinely offset from the
// designed equilibrium. PER AXIS: re-measure and re-set for whichever edge
// is currently active. Set live with "o<deg>", subtracted from phi_edge
// everywhere it's used below (control, trips, arm gate, telemetry). Reset to
// 0 once the missing mass (battery cable, DC-DC) is mounted.
static float kPhiOffset = 0.0f;

void updateEdgeProjection() {
  const EdgeCandidate& c = kCandidates[kAxis][gEdgeIdx];
  float phi[3];
  { float t[3]; cross3(c.gB, ghat, t); phi[0]=-t[0]; phi[1]=-t[1]; phi[2]=-t[2]; }
  phi_edge = dot3(c.e, phi) - kPhiOffset;
  om_edge  = dot3(c.e, w_b);
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------
// Both functions take the destination Stream& explicitly -- loop() picks
// Serial or Serial1 each cycle from gLinkMode. Field content and order are
// otherwise identical to Stage4_FullLaw.ino, so every existing parser and
// logged CSV stays valid:
//
//   t_ms, phi_edge_deg, om_edge_dps, tau_Nm, tau_cmd_Nm,
//   armed, gain_scale, wheel_omega_lp, wheel_pos, wheel_vel
//
// That is the same 10-field shape the panel's telemetry_python_wifi.py
// expects (its columns 1/2 are labelled theta/theta_dot there instead of
// phi_edge/om_edge -- same positions, same units, deg and deg/s).

void printState(Stream& out, uint32_t t_ms, float tau, float tauCmd, bool armed,
                float gainScale, float wheelOmegaLp, const Moteus::Query::Result& v) {
  out.print(t_ms);
  out.print('\t'); out.print(phi_edge * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(om_edge  * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(tau, 4);
  out.print('\t'); out.print(tauCmd, 4);
  out.print('\t'); out.print(armed ? 1 : 0);
  out.print('\t'); out.print(gainScale, 2);
  out.print('\t'); out.print(wheelOmegaLp, 3);
  out.print('\t'); out.print(v.position, 3);
  out.print('\t'); out.println(v.velocity, 3);
}

void telemetryPlot(Stream& out, uint32_t t_ms, float tau, float tauCmd, bool armed,
                   float gainScale, float wheelOmegaLp, const Moteus::Query::Result& v) {
  out.print(t_ms);
  out.print(','); out.print(phi_edge * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(om_edge  * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(tau, 6);
  out.print(','); out.print(tauCmd, 6);
  out.print(','); out.print(armed ? 1 : 0);
  out.print(','); out.print(gainScale, 4);
  out.print(','); out.print(wheelOmegaLp, 6);
  out.print(','); out.print(v.position, 6);
  out.print(','); out.println(v.velocity, 6);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL — full law: K_phi + K_om + K_rho, plus friction FF
// ----------------------------------------------------------------------------
// Byte-for-byte the same law, gains, trips and saturation as
// Stage4_FullLaw.ino. Do not retune here -- retune there and re-copy.

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // already validated through Stage 3's ramp

// Loose this stage -- hand-held, watching combined-term behavior. Stage 5
// tightens to the real DISARM/OMEGA_CAP policy from cubli_gains.h.
static const float kMaxTilt    = 0.4363f;   // rad, 25 deg
// Velocity cap REMOVED for this stage, on request: the taper was choking off
// spin-up torque before the wheel reached enough momentum to actually
// recover, and TAU_MAX (the real torque saturation, below) is untouched.
// Cube is hand-held and you're arming with the explicit intent to disarm
// ("a0") if it runs away -- that, plus the link watchdog this build adds, is
// the actual safety mechanism here, not this constant. Do NOT carry this
// into Stage 5: that stage keeps the real 40 rad/s OMEGA_CAP.
static const float kMaxOmega   = 1.0e6f;    // rad/s -- effectively uncapped
static const float kTauMax     = 0.12f;     // N*m, TAU_MAX
static const float kTaperStart = 36.0f;     // rad/s -- irrelevant with
                                              // kMaxOmega this large

// Friction feedforward -- Firmware Lessons: "not optional". PLACEHOLDER
// values from cubli_gains.h pending the real spin-down/breakaway test.
static const float kTauCw  = 0.008f;   // N*m, PLACEHOLDER
static const float kBw     = 0.0f;     // N*m*s, PLACEHOLDER
static const float kEpsFf  = 0.05f;    // rad/s, tanh width

// Arm gate (mega-prompt 3.3): refuse "a1" unless already near the edge
// equilibrium. Checked at the moment of arming, not continuously.
static const float kArmGate = 0.1672664619f;   // rad, 0.5 deg

// Per-axis: all three confirmed via each axis's own Stage 1 pulse test.
// Re-verify per axis rather than assuming if the mount or wiring changes
// (Firmware Lessons S4).
static const float kAxisWheelSign[3] = {
  1.0f,   // X -- CONFIRMED
  1.0f,   // Y -- CONFIRMED
  1.0f,   // Z -- CONFIRMED
};
static const float kWheelSign = kAxisWheelSign[kAxis];

// SetPosition()'s DEFAULT wire format only transmits position/velocity --
// every other Command field defaults to Resolution::kIgnore and is silently
// DROPPED before it reaches the CAN bus unless this Format turns it on.
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

static float gLastTau      = 0.0f;
static float gLastTauCmd   = 0.0f;
static float gWheelOmegaLp = 0.0f;   // standing wheel speed, tau = 5 s

void commandWheel(float wheel_omega) {
  const EdgeCandidate& c = kCandidates[kAxis][gEdgeIdx];

  float tau = -(c.K[0] * phi_edge + c.K[1] * om_edge + c.K[2] * wheel_omega) * gGainScale;
  tau += kTauCw * tanhf(wheel_omega / kEpsFf) + kBw * wheel_omega;

  const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
  if (spinning_up) {
    float s = (kMaxOmega - fabsf(wheel_omega)) / (kMaxOmega - kTaperStart);
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    tau *= s;
  }

  if (!isfinite(tau)) { tau = 0.0f; gArmed = false; }

  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  if (fabsf(phi_edge) > kMaxTilt)     { gArmed = false; }
  if (fabsf(wheel_omega) > kMaxOmega) { gArmed = false; }
  if (!isfinite(phi_edge) || !isfinite(om_edge)) { gArmed = false; }

  const float tau_cmd = gArmed ? tau : 0.0f;

  Moteus::PositionMode::Command cmd;
  cmd.position               = NaN;
  cmd.velocity               = 0.0f;
  cmd.kp_scale                = 0.0f;
  cmd.kd_scale                 = 0.0f;
  cmd.feedforward_torque       = kWheelSign * tau_cmd;
  cmd.maximum_torque           = kTauMax;
  cmd.watchdog_timeout          = 0.10f;
  cmd.ignore_position_bounds    = 1.0f;
  moteusActive.SetPosition(cmd, &kTorqueFormat);

  static const float kLpTau = 5.0f;
  const float dtNom = kPeriodMs * 1e-3f;
  const float alphaLp = dtNom / (kLpTau + dtNom);
  gWheelOmegaLp += alphaLp * (wheel_omega - gWheelOmegaLp);

  gLastTau    = tau;
  gLastTauCmd = tau_cmd;
}


// ----------------------------------------------------------------------------
// SECTION 2f: COMMANDS (from either Serial or Serial1)
// ----------------------------------------------------------------------------
// Same a/g/o/e grammar as Stage4_FullLaw.ino, plus 'h'/'k' (keepalive no-op)
// and 't' (link mode -- handled in pollCommands() before this is called).
// `echo` is whichever stream the line arrived on, so an ack goes back down
// the channel it came in on.
//
// NOTE on the arm-gate refusal message: it is printed in BOTH telemetry
// modes, deliberately. In PLOTMODE it lands in the CSV stream as a '#' line
// that the PC-side parser drops -- so the authoritative confirmation that an
// "a1" actually took is the `armed` column staying at 1 (the plot script
// prints "Teensy: ARMED"/"DISARMED" whenever it changes). If a1 seems to do
// nothing, the gate refused it: get closer to vertical, or set o<deg> first.
void handleCommandLine(const char* line, Stream& echo) {
  if (line[0] == '\0') { return; }

  const char cmd = line[0];
  const float val = atof(line + 1);

  if (cmd == 'a') {
    if (val != 0.0f) {
      if (fabsf(phi_edge) < kArmGate) {
        gArmed = true;
        echo.println("# gArmed = TRUE");
      } else {
        gArmed = false;
        echo.print("# ARM REFUSED: |phi_edge|="); echo.print(fabsf(phi_edge) * (float)RAD_TO_DEG, 3);
        echo.print(" deg exceeds ARM_GATE="); echo.print(kArmGate * (float)RAD_TO_DEG, 2);
        echo.println(" deg. Get closer to vertical and retry.");
      }
    } else {
      gArmed = false;
      echo.println("# gArmed = FALSE");
    }
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
#if TELEMETRY_MODE == SERIALMONITORMODE
    echo.print("# gGainScale = "); echo.println(gGainScale, 3);
#endif
  } else if (cmd == 'o') {
    kPhiOffset = val * (float)DEG_TO_RAD;
#if TELEMETRY_MODE == SERIALMONITORMODE
    echo.print("# kPhiOffset = "); echo.print(val, 4); echo.println(" deg");
#endif
  } else if (cmd == 'e') {
    resolveEdgeCandidate(echo);
  } else if (cmd == 'h' || cmd == 'k') {
    // keepalive / no-op -- receipt alone is enough (see pollCommands()).
    // Both letters accepted: 'h' keeps telemetry_python_wifi.py working
    // unmodified, 'k' matches Skeleton_3Axis_WiFi.ino's grammar.
#if TELEMETRY_MODE == SERIALMONITORMODE
  } else {
    echo.println("# unknown. use: a<0/1>  g<0..1>  o<deg>  e  t<0/1>  h|k");
#endif
  }
}

// Polls BOTH Serial and Serial1 every loop() iteration -- see LineReader
// above for why this can never block. 't'-prefixed lines switch gLinkMode
// locally and are NOT passed to handleCommandLine(); everything else
// (a/g/o/e/h/k) is, regardless of which stream it arrived on.
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
      gLinkMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
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
  // Bounded wait, NOT while(!Serial){} -- the USB build can afford to hang
  // there since a laptop is always attached; this build must still boot into
  // WIFI mode with nothing on USB at all.
  uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) { delay(10); }

  Serial1.begin(kLinkBaud);
  gLastSerial1RxMillis = millis();

  gBoot.println("\nstarted - EDGE STAGE 4: FULL LAW (WiFi build, cube held by hand)");
  gBoot.print("# ACTIVE AXIS: "); gBoot.print(kAxisName[kAxis]);
  gBoot.print("  (moteus id "); gBoot.print(kAxisMoteusId[kAxis]);
  gBoot.println(") -- confirm this matches your Stage 1/2/3 runs.");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    gBoot.print("CAN error 0x");
    gBoot.println(errorCode, HEX);
    delay(1000);
  }

  moteusActive.SetStop();
  gBoot.println("all stopped");

  updateMountingDCM();
  checkMountingDCMValid(gBoot);

  pinMode(imuChipSelectPin, OUTPUT);
  digitalWrite(imuChipSelectPin, HIGH);
  SPI.begin();

  while (imu.beginSPI(imuChipSelectPin, imuClockFrequency) != BMI2_OK) {
    gBoot.println("Error: BMI270 not connected, check wiring and CS pin!");
    delay(1000);
  }
  gBoot.println("BMI270 connected!");

  if (imu.setAccelODR(BMI2_ACC_ODR_400HZ) != BMI2_OK ||
      imu.setGyroODR(BMI2_GYR_ODR_400HZ)  != BMI2_OK) {
    gBoot.println("Warning: could not raise BMI270 ODR to 400 Hz");
  }

  gBoot.println("# calibrating gyro bias -- keep the cube PERFECTLY STILL (~2s)");
  calibrateGyroBias(1000);

  for (int i = 0; i < 50; ++i) {
    float aImu[3], wImu[3];
    imu.getSensorData();
    readIMURaw(imu, aImu, wImu);
    attitudeUpdate(aImu, wImu, 0.002f);
    delay(2);
  }
  resolveEdgeCandidate(gBoot);

#if TELEMETRY_MODE == SERIALMONITORMODE
  gBoot.println("t_ms\tphi_edge_deg\tom_edge_dps\ttau_Nm\ttau_cmd_Nm\tarmed\t"
                 "gain_scale\twheel_omega_lp\twheel_pos\twheel_vel");
  gBoot.println("# STARTS DISARMED. Position near vertical, then send a1");
  gBoot.println("# (arm gate: refuses unless |phi_edge| < 0.5 deg).");
  gBoot.println("# o<deg> sets kPhiOffset for this build's actual resting point.");
  gBoot.println("# velocity cap REMOVED this stage -- disarm (a0) is the safety net.");
  gBoot.println("# t0 = USB link mode, t1 = WiFi link mode (default t1). h/k = keepalive.");
  gBoot.println("# WiFi mode auto-disarms if no line arrives on Serial1 for 300 ms.");
#endif

  gNextSendMillis = millis();

}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {

  pollCommands();

  // WiFi-mode link watchdog -- only enforced in WIFI mode, does exactly what
  // an explicit a0 does. USB mode never auto-disarms from this (matches
  // Stage4_FullLaw.ino: only a0 or the control law's own trips do).
  if (gLinkMode == LinkMode::WIFI && gArmed &&
      millis() - gLastSerial1RxMillis > kLinkTimeoutMs) {
    gArmed = false;
  }

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
  updateEdgeProjection();

  const auto& v = moteusActive.last_result().values;
  const float wheel_omega = kWheelSign * v.velocity * 2.0f * (float)PI;

  commandWheel(wheel_omega);

  // --- telemetry: goes to Serial or Serial1 depending on gLinkMode ---
  Stream& out = (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
#if TELEMETRY_MODE == PLOTMODE
  telemetryPlot(out, time, gLastTau, gLastTauCmd, gArmed, gGainScale, gWheelOmegaLp, v);
#else
  printState(out, time, gLastTau, gLastTauCmd, gArmed, gGainScale, gWheelOmegaLp, v);
#endif

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Companion files:
//   ../../USBMODE/EdgeBalance/EdgeBalance.ino  -- the USB-tethered reference
//     build this is derived from, kept byte-identical to the hardware-
//     validated edge-bringup copy. Retune THERE, then re-copy the changed
//     constants.
//   ../xiao_teensy_bridge/xiao_teensy_bridge.ino  -- the XIAO ESP32C6 sketch
//     that relays Serial1 <-> UDP, reused as-is (no edit needed beyond WiFi
//     credentials / static IP). SHARED with CornerBalance_WiFi.ino, so only
//     ONE of the two WiFi builds can be on the link at a time.
//   ../telemetry_python_wifi.py  -- the PC-side viewer/logger/command
//     console. Works against this sketch unmodified: same 10-field PLOTMODE
//     CSV, same commands, and its "h" keepalive is accepted here. Its plot
//     labels say theta/theta_dot -- read those as phi_edge/om_edge.
//   ../../plot_session_csv.py  -- offline plotter for the saved session CSV.
//
// See ../README.md for the end-to-end flashing procedure.
// ============================================================================
