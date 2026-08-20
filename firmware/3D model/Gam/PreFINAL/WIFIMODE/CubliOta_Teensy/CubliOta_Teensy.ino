// ============================================================================
// TEENSY 4.1 -- MODE STATE MACHINE + WIRELESS (OTA) FLASHING  -- TEMPLATE
// ============================================================================
// Companion to ../xiao_cubli_console/xiao_cubli_console.ino and the web
// console it serves. This is a TEMPLATE, and the word is meant literally:
//
//   WHAT IS REAL AND WORKS AS-IS
//     - the Serial1 link layer, watchdog and command grammar;
//     - the BMI270 read path, mounting DCM and the gam attitude estimator,
//       transplanted verbatim from CornerBalance_WiFi.ino (SECTION 2b);
//     - corner and edge pivot resolution and projection, so |phi| / theta are
//       real numbers you can watch on the console the moment you flash this;
//     - the IDLE / EDGE / CORNER state machine and its interlocks;
//     - telemetry in both existing wire formats, plus the '#H' health line;
//     - the whole OTA pipeline: hex parsing, checksum validation, staging into
//       the FlasherX buffer and flash_move().
//
//   WHAT IS A STUB AND WILL NOT BALANCE ANYTHING
//     - the control laws. cornerControlStep() and edgeControlStep() command
//       ZERO torque and only query the controllers. The gain tables that go
//       with them (kCorners[].Kp, kCandidates[].K) are deliberately NOT copied
//       here either -- they belong with the law.
//
// Flashed as-is this is a safe, useful instrument: real attitude telemetry,
// real wheel speeds, real battery voltage, working E-STOP, working OTA. It
// just never commands torque. See TRANSPLANTING A CONTROL LAW at the bottom.
//
// !! NOTHING IN THIS FILE HAS BEEN VALIDATED ON HARDWARE. !!
// The corner and edge WiFi builds themselves are still marked unvalidated in
// their own headers. Treat the first run as a bring-up: cube on the bench,
// one hand on it, E-STOP in reach.
//
// ---------------------------------------------------------------------------
// FLASHERX SETUP -- REQUIRED, ONE TIME
// ---------------------------------------------------------------------------
// FlasherX is not an Arduino library; it is source you copy into the sketch
// folder. From https://github.com/joepasquariello/FlasherX, copy into THIS
// directory (next to this .ino):
//
//     FlashTxx.c        FlashTxx.h
//
// FXUtil.cpp / FXUtil.h are NOT needed and should NOT be copied. Their
// update_firmware() is an interactive routine that prompts "enter %d to flash
// or 0 to abort" on a Stream and waits for a typed reply -- unusable across a
// WiFi bridge. This file drives the stable primitives directly instead
// (firmware_buffer_init / flash_write_block / check_flash_id / flash_move) and
// carries its own Intel HEX parser, which is also where the per-record
// checksum validation happens.
//
// ---------------------------------------------------------------------------
// WHAT YOU CAN AND CANNOT FLASH OVER THE AIR
// ---------------------------------------------------------------------------
// check_flash_id() searches the staged image for the literal string
// "fw_teensy41", which FlashTxx.c puts into any sketch that compiles it in.
// So THE IMAGE YOU UPLOAD MUST ITSELF INCLUDE FLASHERX. Upload a sketch built
// without it and the commit is refused -- which is the check doing its job,
// because that image would also be the last one you could ever flash
// wirelessly. Every OTA-able build needs FlashTxx.c in its sketch folder.
//
// ---------------------------------------------------------------------------
// COMMAND GRAMMAR
// ---------------------------------------------------------------------------
//   m<0/1/2> mode: 0 IDLE, 1 EDGE, 2 CORNER. Always disarms and stops wheels.
//   a<0/1>   arm / disarm. a1 passes the tilt gate and is refused in IDLE.
//   g<0..1>  gain scale
//   c / e    re-resolve the corner / edge pivot
//   z<0/1>   tare / clear the corner phi offset
//   o<deg>   set the edge phi offset
//   p<0/1>   halt / resume
//   t<0/1>   link mode: 0 USB, 1 WiFi (default 1)
//   h | k    no-op keepalive
//   u1 / u? / u2 / u0    OTA: begin / probe / commit / abort  (see SECTION 6)
//
// 'h' is a KEEPALIVE here, not halt -- same choice CornerBalance_WiFi.ino made
// and for the same reason: the link heartbeat is a bare letter sent ten times
// a second, and binding halt to it would halt the cube continuously.
//
// Wiring (unchanged):
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   Common GND.
// ============================================================================

#if !__has_include("FlashTxx.h")
#error "FlasherX missing. Copy FlashTxx.c and FlashTxx.h from https://github.com/joepasquariello/FlasherX into this sketch folder. See the FLASHERX SETUP note at the top of this file."
#endif

#include <MoteusTeensy.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// FlashTxx.h declares C functions but does NOT wrap itself in extern "C", so
// the wrap has to happen here or every symbol comes out C++-mangled and the
// link fails. This is the pattern FlasherX's own example sketch uses.
extern "C" {
  #include "FlashTxx.h"
}


// ----------------------------------------------------------------------------
// SECTION 0: CONFIGURATION
// ----------------------------------------------------------------------------

// Set to 1 ONLY after transplanting a real control law into SECTION 4. While
// this is 0, commandWheels() issues query-only frames and never commands
// torque, no matter what mode is selected or whether the cube is armed.
#define CONTROL_LAW_TRANSPLANTED 0

#define SERIALMONITORMODE 0
#define PLOTMODE 1
#define TELEMETRY_MODE PLOTMODE

// Serial1 <-> XIAO. MUST match TEENSY_LINK_BAUD in xiao_cubli_console.ino.
// 1 Mbaud, not 115200: a 21-field corner line is ~170 B at 250 Hz = ~340
// kbit/s, which does not fit in 115200 -- Serial1 writes would start blocking
// inside the 2 ms control cycle. See CornerBalance_WiFi.ino:33 for the full
// budget. A mismatch between the two sketches is silent and looks like a dead
// link, so change both together or neither.
const uint32_t kLinkBaud = 1000000;

// Auto-disarm if WIFI mode and no Serial1 line for this long. The XIAO's
// keepalive is 100 ms and is itself gated on the browser still pinging, so
// this is the last link in an end-to-end deadman chain -- see the long note
// in xiao_cubli_console.ino.
const uint32_t kLinkTimeoutMs = 300;

const uint32_t kPeriodMs = 2;            // 500 Hz control loop
static const uint8_t kTelemetryDecim = 2;  // -> 250 Hz on the wire
const uint32_t kHealthIntervalMs = 500;  // '#H' battery/mode/armed line

// The edge law is single-axis: one wheel axis has to lie along the contact
// line. Must match the edge build whose law you transplant.
// EdgeBalance_WiFi.ino:112 uses AXIS_Y ("best on the cube" -- its candidates
// have 0.006/0.007 deg place offsets against 0.75-0.85 for X and Z).
enum Axis { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };
static const Axis kAxis = AXIS_Y;
static const char* kAxisName[3] = { "X", "Y", "Z" };

// Tilt gate for a1, compared against |phi| (corner) or |theta| (edge).
// 1.0 deg matches CornerBalance_WiFi.ino:584. NOTE: EdgeBalance_WiFi.ino:484
// holds 0.1672664619 rad = 9.58 deg, which looks like a corrupted copy of
// cubli_gains.h's 0.00872664619 (shared '72664619' tail) and whose comment
// still claims 0.5 deg. This file does not inherit that: one gate, stated
// once, and the console is told what it is.
static const float kArmGate = 0.01745329252f;   // rad, 1.0 deg


// ----------------------------------------------------------------------------
// SECTION 1: OBJECTS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

Moteus moteusX(canBus, []() { Moteus::Options options; options.id = 2; return options; }());
Moteus moteusY(canBus, []() { Moteus::Options options; options.id = 3; return options; }());
Moteus moteusZ(canBus, []() { Moteus::Options options; options.id = 1; return options; }());

static Moteus& wheelObj(int i) { return (i == 0) ? moteusX : (i == 1) ? moteusY : moteusZ; }

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;

// Serial1 RX buffer, enlarged from the 64 B default. THIS IS LOAD-BEARING FOR
// OTA, not a tuning knob: a 4 KB flash sector erase stalls this loop for tens
// of milliseconds, and at 1 Mbaud that is several KB of hex records still
// arriving. 8 KB is 4x the XIAO's 2048-byte acknowledge chunk, so even a
// worst-case stall in the middle of a chunk cannot overflow it.
static uint8_t gSerial1RxBuf[8192];


// ----------------------------------------------------------------------------
// SECTION 2: LINK LAYER (transplanted from CornerBalance_WiFi.ino)
// ----------------------------------------------------------------------------

enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;
static uint32_t gLastSerial1RxMillis = 0;

// Non-blocking line accumulator. Bounded work per call: returns at most one
// line, and queued lines are picked up on following iterations. The buffer is
// 200 B rather than the 64 B the balance builds use because an Intel HEX
// record with 32 data bytes is 75 characters and OTA reuses this reader.
struct LineReader {
  char buf[200];
  uint16_t len = 0;

  bool poll(Stream& s, char* outLine, size_t outSize) {
    while (s.available()) {
      const char c = (char)s.read();
      if (c == '\n' || c == '\r') {
        if (len == 0) continue;
        buf[len] = '\0';
        const uint16_t copyLen = (len < outSize - 1) ? len : (uint16_t)(outSize - 1);
        memcpy(outLine, buf, copyLen);
        outLine[copyLen] = '\0';
        len = 0;
        return true;
      }
      if (len < sizeof(buf) - 1) buf[len++] = c;
      // else: overlong line -- drop the overflow, resync at the terminator
    }
    return false;
  }
};

static LineReader gUsbReader;
static LineReader gLinkReader;

class DualPrint : public Print {
 public:
  using Print::write;
  size_t write(uint8_t c) override { Serial.write(c); Serial1.write(c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n); Serial1.write(b, n); return n;
  }
};
static DualPrint gBoot;

/** Whichever stream telemetry is currently going out on. */
static Stream& outStream() {
  return (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
}


// ----------------------------------------------------------------------------
// SECTION 3: STATE ESTIMATION -- gam
// ----------------------------------------------------------------------------
// Verbatim from CornerBalance_WiFi.ino SECTION 2b, which is itself byte-for-
// byte the USB reference build. Identical in the edge build. Do not retune
// here; retune in the reference build and re-copy.

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
  for (int i = 0; i < 3; ++i) ghat[i] += dt * (-wxg[i] + kP_FILT * exg[i]);
  normalize3(ghat);
  for (int i = 0; i < 3; ++i) bhat[i] += kI_FILT * e[i] * dt;
  for (int i = 0; i < 3; ++i) w_b[i] = wRaw[i] - bhat[i];

  static int badCount = 0;
  if (dot3(ghat, ga) < 0.0f) {
    if (++badCount > 20) {
      ghat[0] = ga[0]; ghat[1] = ga[1]; ghat[2] = ga[2];
      bhat[0] = bhat[1] = bhat[2] = 0.0f;
      badCount = 0;
    }
  } else {
    badCount = 0;
  }
}


// ----------------------------------------------------------------------------
// SECTION 3b: PIVOT GEOMETRY
// ----------------------------------------------------------------------------
// gB is the body-frame gravity direction at balance (cubli_gains.h:49), so it
// points DOWN in body frame -- at whichever feature is resting on the table.
// That is the pivot.
//
// The gain tables that live alongside these in the balance builds
// (kCorners[].Kp, 8x3x9 floats; kCandidates[].K) are deliberately NOT here.
// They are part of the control law and belong with the transplant, not with
// the geometry -- keeping them apart is what stops this file from looking
// like a controller it is not.
//
// Values verbatim from CornerBalance_WiFi.ino:382 and EdgeBalance_WiFi.ino:348.

struct CornerCandidate { const char* name; float gB[3]; float placeOffsetDeg; };

static const CornerCandidate kCorners[8] = {
  { "[-1,-1,-1]", { -0.571549475f, -0.588645577f, -0.571688354f }, 0.797f },
  { "[-1,-1,+1]", { -0.546220303f, -0.562558711f,  0.620621562f }, 3.170f },
  { "[-1,+1,-1]", { -0.556852818f,  0.616181135f, -0.556988120f }, 2.773f },
  { "[-1,+1,+1]", { -0.533349514f,  0.590173781f,  0.605997622f }, 3.097f },
  { "[+1,-1,-1]", {  0.620658159f, -0.562471628f, -0.546268404f }, 3.171f },
  { "[+1,-1,+1]", {  0.595400333f, -0.539581716f,  0.595273018f }, 2.609f },
  { "[+1,+1,-1]", {  0.606037736f,  0.590086639f, -0.533400357f }, 3.095f },
  { "[+1,+1,+1]", {  0.582457066f,  0.567126632f,  0.582332492f }, 0.714f },
};

struct EdgeCandidate { const char* name; float e[3]; float gB[3]; float placeOffsetDeg; };

static const EdgeCandidate kCandidates[3][2] = {
  { { "X[+1,+1] (+Y,+Z up)", { 1, 0, 0 }, { -0.0f,  0.697691619f,  0.716398239f }, 0.758f },
    { "X[-1,-1] (-Y,-Z up)", { 1, 0, 0 }, { -0.0f, -0.717363894f, -0.696698666f }, 0.837f } },
  { { "Y[+1,+1] (+X,+Z up)", { 0, 1, 0 }, {  0.707182407f, -0.0f,  0.707031190f }, 0.006f },
    { "Y[-1,-1] (-X,-Z up)", { 0, 1, 0 }, { -0.707020879f, -0.0f, -0.707192659f }, 0.007f } },
  { { "Z[+1,+1] (+X,+Y up)", { 0, 0, 1 }, {  0.716472805f,  0.697615027f, -0.0f }, 0.764f },
    { "Z[-1,-1] (-X,-Y up)", { 0, 0, 1 }, { -0.696611583f, -0.717448473f, -0.0f }, 0.844f } },
};

static int gCornerIdx = 0;
static int gEdgeIdx = 0;

float phi[3] = { 0.0f, 0.0f, 0.0f };      // corner tilt error, body frame
float phi_edge = 0.0f;                     // edge tilt error, about the axis
float om_edge  = 0.0f;
static float gPhiOffset[3] = { 0.0f, 0.0f, 0.0f };   // corner tare, "z1"/"z0"
static float gEdgeOffset = 0.0f;                      // edge trim, "o<deg>"

void resolveCornerCandidate(Print& out) {
  int bestIdx = 0, secondIdx = 0;
  float bestDot = -2.0f, secondDot = -2.0f;
  for (int i = 0; i < 8; ++i) {
    const float d = dot3(ghat, kCorners[i].gB);
    if (d > bestDot) { secondDot = bestDot; secondIdx = bestIdx; bestDot = d; bestIdx = i; }
    else if (d > secondDot) { secondDot = d; secondIdx = i; }
  }
  gCornerIdx = bestIdx;
  out.print("# corner resolved: "); out.print(kCorners[gCornerIdx].name);
  out.print("  place_offset="); out.print(kCorners[gCornerIdx].placeOffsetDeg, 3);
  out.print(" deg  (best_dot="); out.print(bestDot, 4);
  out.print(" runner_up="); out.print(kCorners[secondIdx].name);
  out.print(" dot="); out.print(secondDot, 4); out.println(")");
  if (bestDot - secondDot < 0.2f)
    out.println("# WARNING: best and runner-up corners are close -- verify before arming.");
}

void resolveEdgeCandidate(Print& out) {
  const float d0 = dot3(ghat, kCandidates[kAxis][0].gB);
  const float d1 = dot3(ghat, kCandidates[kAxis][1].gB);
  gEdgeIdx = (d0 >= d1) ? 0 : 1;
  out.print("# axis="); out.print(kAxisName[kAxis]);
  out.print("  edge candidate resolved: "); out.print(kCandidates[kAxis][gEdgeIdx].name);
  out.print("  place_offset="); out.print(kCandidates[kAxis][gEdgeIdx].placeOffsetDeg, 3);
  out.print(" deg  (dot0="); out.print(d0, 4);
  out.print(" dot1="); out.print(d1, 4); out.println(")");
  if (fabsf(d0 - d1) < 0.2f)
    out.println("# WARNING: dot0 and dot1 are close -- verify before arming.");
}

void updateCornerProjection() {
  float t[3];
  cross3(kCorners[gCornerIdx].gB, ghat, t);
  phi[0] = -t[0] - gPhiOffset[0];
  phi[1] = -t[1] - gPhiOffset[1];
  phi[2] = -t[2] - gPhiOffset[2];
}

void updateEdgeProjection() {
  const EdgeCandidate& c = kCandidates[kAxis][gEdgeIdx];
  float p[3];
  { float t[3]; cross3(c.gB, ghat, t); p[0] = -t[0]; p[1] = -t[1]; p[2] = -t[2]; }
  phi_edge = dot3(c.e, p) - gEdgeOffset;
  om_edge  = dot3(c.e, w_b);
}


// ----------------------------------------------------------------------------
// SECTION 4: MODE STATE MACHINE + CONTROL
// ----------------------------------------------------------------------------

enum class Mode : uint8_t { IDLE = 0, EDGE = 1, CORNER = 2 };
static Mode gMode = Mode::IDLE;
static const char* modeName(Mode m) {
  return (m == Mode::IDLE) ? "IDLE" : (m == Mode::EDGE) ? "EDGE" : "CORNER";
}

static bool  gArmed = false;
static bool  gHalted = false;
static float gGainScale = 1.0f;

static float gLastTau[3]    = { 0, 0, 0 };
static float gLastTauCmd[3] = { 0, 0, 0 };
static float gRhoLp[3]      = { 0, 0, 0 };   // standing speed, ~5 s low-pass
static float gWheelLp       = 0.0f;          // edge equivalent

// Health, straight off the CAN replies. moteus_protocol.h:348 puts voltage in
// the DEFAULT query format at kInt8 resolution, so this arrives on every reply
// with no Format change and no extra bus traffic -- it is simply unused by the
// balance builds. That resolution is 0.5 V/LSB, which against a 6S pack
// (25.2 V full, 18 V floor) is ~14 steps across the usable span: coarse, but
// real, and enough to answer "is the pack sagging".
static float gVbat = -1.0f;
static float gTmax = -1.0f;

static void stopAllWheels() {
  moteusX.SetStop();
  moteusY.SetStop();
  moteusZ.SetStop();
}

/** Read wheel speeds (rad/s) out of the last CAN replies. */
static void readWheelSpeeds(float rho[3]) {
  rho[0] = moteusX.last_result().values.velocity * 2.0f * (float)PI;
  rho[1] = moteusY.last_result().values.velocity * 2.0f * (float)PI;
  rho[2] = moteusZ.last_result().values.velocity * 2.0f * (float)PI;
}

static void updateHealth() {
  float v = -1.0f, t = -1.0f;
  for (int i = 0; i < 3; ++i) {
    const auto& r = wheelObj(i).last_result().values;
    if (!isnan(r.voltage) && r.voltage > v) v = (float)r.voltage;
    if (!isnan(r.temperature) && r.temperature > t) t = (float)r.temperature;
  }
  if (v > 0) gVbat = v;
  if (t > -100) gTmax = t;
}

// ===== TRANSPLANT POINT ======================================================
// Both hooks are called once per control cycle with the estimator already
// updated for this tick. They must fill tau[] (N*m, body frame) and command
// the wheels. As shipped they command nothing.
//
// See "TRANSPLANTING A CONTROL LAW" at the end of this file.
// =============================================================================

static void cornerControlStep(const float rho[3]) {
#if CONTROL_LAW_TRANSPLANTED
  // >>> paste the body of commandWheels() from
  // >>> ../CornerBalance_WiFi/CornerBalance_WiFi.ino (SECTION 2e, ~line 633)
  // >>> together with its kCorners[].Kp gain table, kTauMax, kMaxTilt,
  // >>> kMaxOmega, the friction feedforward constants and the trip logic.
  #error "CONTROL_LAW_TRANSPLANTED is 1 but cornerControlStep() is still empty."
#else
  (void)rho;
  // Query-only: keeps velocity, voltage and temperature fresh in
  // last_result() without putting the controllers into any active mode.
  moteusX.SetQuery(); moteusY.SetQuery(); moteusZ.SetQuery();
  gLastTau[0] = gLastTau[1] = gLastTau[2] = 0.0f;
  gLastTauCmd[0] = gLastTauCmd[1] = gLastTauCmd[2] = 0.0f;
#endif
}

static void edgeControlStep(float wheelVel) {
#if CONTROL_LAW_TRANSPLANTED
  // >>> paste the body of commandWheel() from
  // >>> ../EdgeBalance_WiFi/EdgeBalance_WiFi.ino (SECTION 2e) together with
  // >>> its kCandidates[].K gains and trip logic. Only the kAxis wheel is
  // >>> driven; the other two stay stopped.
  #error "CONTROL_LAW_TRANSPLANTED is 1 but edgeControlStep() is still empty."
#else
  (void)wheelVel;
  moteusX.SetQuery(); moteusY.SetQuery(); moteusZ.SetQuery();
  gLastTau[0] = gLastTau[1] = gLastTau[2] = 0.0f;
  gLastTauCmd[0] = gLastTauCmd[1] = gLastTauCmd[2] = 0.0f;
#endif
}

/** Every mode change goes through a full stop. This is the interlock. */
static void setMode(Mode m, Print& out) {
  gArmed = false;
  stopAllWheels();
  gLastTau[0] = gLastTau[1] = gLastTau[2] = 0.0f;
  gLastTauCmd[0] = gLastTauCmd[1] = gLastTauCmd[2] = 0.0f;

  // The tare is a property of one pivot, so carrying it across a mode change
  // would silently apply a corner's COM correction to an edge.
  gPhiOffset[0] = gPhiOffset[1] = gPhiOffset[2] = 0.0f;
  gEdgeOffset = 0.0f;

  gMode = m;
  gHalted = false;

  out.print("# mode = "); out.print(modeName(m));
  out.println("  (disarmed, wheels stopped, tare cleared)");
  if (m == Mode::CORNER) resolveCornerCandidate(out);
  else if (m == Mode::EDGE) resolveEdgeCandidate(out);
}

/** |tilt error| in rad, whichever mode is active -- what the arm gate tests. */
static float tiltError() {
  return (gMode == Mode::EDGE) ? fabsf(phi_edge) : norm3(phi);
}


// ----------------------------------------------------------------------------
// SECTION 5: TELEMETRY
// ----------------------------------------------------------------------------
// Field content and order are identical to the existing builds, so every
// parser in the tree keeps working unchanged: 21 comma-separated fields is
// CornerBalance_WiFi's PLOTMODE row, 10 is EdgeBalance_WiFi's. dashboard/
// schemas.py, plot_session_csv.py, telemetry_python_wifi*.py and the new web
// console all key off exactly that width.
//
// IDLE emits the 21-field corner row against the auto-resolved corner, with
// zero torque and armed = 0. That is a deliberate choice over emitting
// nothing: a silent link is indistinguishable from a dead one on every
// display in the tree, and the tilt and wheel columns are still true.

void telemetryCorner(Stream& out, uint32_t t_ms, const float rho[3]) {
  out.print(t_ms);
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(phi[i] * (float)RAD_TO_DEG, 4); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(w_b[i] * (float)RAD_TO_DEG, 3); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(rho[i], 3); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(gRhoLp[i], 3); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(gLastTau[i], 5); }
  for (int i = 0; i < 3; ++i) { out.print(','); out.print(gLastTauCmd[i], 5); }
  out.print(','); out.print(gArmed ? 1 : 0);
  out.print(','); out.println(gGainScale, 3);
}

void telemetryEdge(Stream& out, uint32_t t_ms, float wheelVel) {
  out.print(t_ms);
  out.print(','); out.print(phi_edge * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(om_edge * (float)RAD_TO_DEG, 3);
  out.print(','); out.print(gLastTau[kAxis], 5);
  out.print(','); out.print(gLastTauCmd[kAxis], 5);
  out.print(','); out.print(gArmed ? 1 : 0);
  out.print(','); out.print(gGainScale, 3);
  out.print(','); out.print(gWheelLp, 3);
  out.print(','); out.print(wheelObj(kAxis).last_result().values.position, 4);
  out.print(','); out.println(wheelVel, 4);
}

/** '#H,vbat,mode,armed,tmax' -- health on a '#' line, deliberately.
 *
 *  Adding columns to the CSV would take corner from 21 to 22 fields and break
 *  the field-count schema detection that every tool in this tree relies on.
 *  '#' lines are already ignored by all of them, so this carries battery and
 *  mode to the console at zero cost to compatibility.
 */
void telemetryHealth(Stream& out) {
  out.print("#H,");
  out.print(gVbat, 2);
  out.print(','); out.print((int)gMode);
  out.print(','); out.print(gArmed ? 1 : 0);
  out.print(','); out.println(gTmax, 1);
}


// ----------------------------------------------------------------------------
// SECTION 6: OTA -- Intel HEX over Serial1 into the FlasherX staging buffer
// ----------------------------------------------------------------------------
// Protocol, driven entirely by the XIAO (see xiao_cubli_console.ino):
//
//   u1   -> "#OTA READY <bufsize>"  or  "#OTA REFUSED <reason>"
//   :... -> hex records, streamed. Errors answer "#OTA ERR <reason>".
//   u?   -> "#OTA ACK <records> <bytes>"   -- the flow-control probe
//   u2   -> validate and flash_move(). Reboots; there is no reply.
//   u0   -> free the buffer and reboot unchanged.
//
// An explicit 'u1' sentinel starts this, rather than sniffing the stream for
// lines beginning with ':'. A telemetry glitch or a stray console byte must
// never be able to put a balancing cube into a firmware-flashing state.

static bool     gOtaMode = false;
static uint32_t gOtaBufAddr = 0, gOtaBufSize = 0;
static uint32_t gOtaBase = 0;          // set by type-04/02 records
static uint32_t gOtaMin = 0xFFFFFFFF, gOtaMax = 0;
static uint32_t gOtaRecords = 0, gOtaBytes = 0;
static bool     gOtaEof = false;
static uint32_t gOtaLastRxMs = 0;

// Once gOtaMode is set, the ONLY ways out are commit and abort, and both end
// in a reboot -- see otaAbort() for why re-entering without one is unsafe. So
// an uploader that dies mid-transfer would otherwise strand the board here
// with its wheels stopped and no control loop, looking bricked.
const uint32_t kOtaIdleTimeoutMs = 30000;

static void rebootTeensy() {
  Serial.flush();
  Serial1.flush();
  delay(20);
  (*(volatile uint32_t*)0xE000ED0C) = 0x5FA0004;   // SCB->AIRCR SYSRESETREQ
  while (1) {}
}

/** Free the staging buffer and reboot. THE REBOOT IS MANDATORY.
 *
 *  flash_write_block() carries static buf_count/next_addr across calls. A
 *  transfer that stops part-way through a 4-byte group leaves buf_count > 0,
 *  and every subsequent attempt then fails with its error 2 (non-sequential
 *  address) until the statics are re-initialised. Only a reboot does that.
 *  FlasherX's own example sketch reboots after a failed update for exactly
 *  this reason.
 */
static void otaAbort(Print& out, const char* why) {
  out.print("#OTA ERR "); out.println(why);
  out.print("#OTA freeing staging buffer and rebooting");
  out.println();
  out.flush();
  if (gOtaBufSize) firmware_buffer_free(gOtaBufAddr, gOtaBufSize);
  rebootTeensy();
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
static int hexByte(const char* p) {
  const int hi = hexNibble(p[0]), lo = hexNibble(p[1]);
  return (hi < 0 || lo < 0) ? -1 : ((hi << 4) | lo);
}

/** Parse one Intel HEX record. Returns 0 on success, non-zero error code.
 *
 *  Layout: ':' LL AAAA TT [DD]*LL CC, all hex ASCII. The checksum CC makes the
 *  8-bit sum of every byte from LL onward come to zero, which is the check the
 *  brief asks for and the one that catches a corrupted record before it
 *  reaches flash.
 */
static int parseHexRecord(const char* line, uint8_t* data, uint8_t* count,
                          uint16_t* addr, uint8_t* type) {
  if (line[0] != ':') return 1;
  const size_t n = strlen(line);
  if (n < 11) return 1;
  const int cnt = hexByte(line + 1);
  if (cnt < 0) return 2;
  if (n != (size_t)(11 + cnt * 2)) return 1;      // declared length vs actual

  uint8_t sum = 0;
  for (size_t i = 1; i + 1 < n; i += 2) {
    const int b = hexByte(line + i);
    if (b < 0) return 2;
    sum += (uint8_t)b;                            // includes CC, so sum -> 0
  }
  if (sum != 0) return 3;

  *count = (uint8_t)cnt;
  *addr  = (uint16_t)((hexByte(line + 3) << 8) | hexByte(line + 5));
  *type  = (uint8_t)hexByte(line + 7);
  for (int i = 0; i < cnt; ++i) data[i] = (uint8_t)hexByte(line + 9 + i * 2);
  return 0;
}

static void otaBegin(Print& out) {
  if (gArmed)            { out.println("#OTA REFUSED armed -- disarm first"); return; }
  if (gMode != Mode::IDLE) { out.println("#OTA REFUSED not IDLE -- send m0 first"); return; }

  stopAllWheels();
  delay(5);
  stopAllWheels();   // twice: a dropped CAN frame must not leave a wheel live

  if (firmware_buffer_init(&gOtaBufAddr, &gOtaBufSize) == 0) {
    out.println("#OTA REFUSED no staging buffer available");
    return;
  }
  gOtaMode = true;
  gOtaBase = 0;
  gOtaMin = 0xFFFFFFFF; gOtaMax = 0;
  gOtaRecords = gOtaBytes = 0;
  gOtaEof = false;
  gOtaLastRxMs = millis();
  out.print("#OTA READY "); out.println(gOtaBufSize);
}

static bool otaRecord(const char* line, Print& out) {
  uint8_t data[256], count, type;
  uint16_t addr;

  const int err = parseHexRecord(line, data, &count, &addr, &type);
  if (err) {
    char why[80];
    snprintf(why, sizeof(why), "record %lu: %s",
             (unsigned long)gOtaRecords + 1,
             err == 1 ? "malformed" : err == 2 ? "bad hex digit" : "BAD CHECKSUM");
    otaAbort(out, why);
    return false;
  }
  gOtaRecords++;

  switch (type) {
    case 0x00: {                                   // data
      const uint32_t absAddr = gOtaBase + addr;
      if (absAddr < FLASH_BASE_ADDR ||
          absAddr + count > FLASH_BASE_ADDR + FLASH_SIZE) {
        otaAbort(out, "record address outside the Teensy 4.1 flash map");
        return false;
      }
      const uint32_t dst = gOtaBufAddr + (absAddr - FLASH_BASE_ADDR);
      if (dst + count > gOtaBufAddr + gOtaBufSize) {
        otaAbort(out, "image larger than the staging buffer");
        return false;
      }
      const int e = flash_write_block(dst, (char*)data, count);
      if (e) {
        char why[64];
        snprintf(why, sizeof(why), "flash_write_block() error %d", e);
        otaAbort(out, why);
        return false;
      }
      if (absAddr < gOtaMin) gOtaMin = absAddr;
      if (absAddr + count > gOtaMax) gOtaMax = absAddr + count;
      gOtaBytes += count;
      break;
    }
    case 0x01: gOtaEof = true; break;                                  // EOF
    case 0x02: gOtaBase = ((uint32_t)((data[0] << 8) | data[1])) << 4;  break;
    case 0x04: gOtaBase = ((uint32_t)((data[0] << 8) | data[1])) << 16; break;
    case 0x03: case 0x05: break;                    // start address -- ignored
    default:
      otaAbort(out, "unknown Intel HEX record type");
      return false;
  }
  return true;
}

static void otaCommit(Print& out) {
  if (!gOtaEof) { otaAbort(out, "no EOF record -- transfer was truncated"); return; }

  const uint32_t size = gOtaMax - gOtaMin;
  if (size < 4096) { otaAbort(out, "staged image implausibly small"); return; }

  // FlasherX's flash_move() copies the buffer to FLASH_BASE_ADDR, so an image
  // that does not start there would land shifted. Teensy hex files always do;
  // check rather than assume, because the failure is a bricked board.
  if (gOtaMin != FLASH_BASE_ADDR) {
    otaAbort(out, "image does not start at 0x60000000");
    return;
  }
  if (!check_flash_id(gOtaBufAddr, size)) {
    otaAbort(out, "no \"" FLASH_ID "\" marker -- wrong board, or the image was "
                  "built without FlasherX (it would not be re-flashable)");
    return;
  }

  out.print("#OTA COMMIT "); out.print(size); out.println(" bytes -- flash_move(), rebooting");
  out.flush();
  Serial.flush();

  stopAllWheels();
  delay(20);

  // Never returns: overwrites the running program and reboots.
  flash_move(FLASH_BASE_ADDR, gOtaBufAddr, size);
}

/** Lines arriving while gOtaMode is set. */
static void otaHandleLine(char* line, Print& out) {
  gOtaLastRxMs = millis();
  if (line[0] == ':') { otaRecord(line, out); return; }
  if (strcmp(line, "u?") == 0) {
    out.print("#OTA ACK "); out.print(gOtaRecords);
    out.print(' '); out.println(gOtaBytes);
    return;
  }
  if (strcmp(line, "u2") == 0) { otaCommit(out); return; }
  if (strcmp(line, "u0") == 0) { otaAbort(out, "aborted by host"); return; }
  // Anything else -- a stray keepalive, a late command -- is dropped in
  // silence. Answering would interleave text with the record stream.
}


// ----------------------------------------------------------------------------
// SECTION 7: COMMANDS
// ----------------------------------------------------------------------------

void handleCommandLine(const char* line, Stream& echo) {
  if (line[0] == '\0') return;

  const char cmd = line[0];
  const float val = atof(line + 1);

  if (cmd == 'u') {
    if (line[1] == '1') otaBegin(echo);
    else echo.println("# OTA not active. Send u1 first (requires IDLE + disarmed).");

  } else if (cmd == 'm') {
    const int m = (int)val;
    if (m < 0 || m > 2) { echo.println("# mode must be 0 (IDLE), 1 (EDGE) or 2 (CORNER)"); return; }
    setMode((Mode)m, echo);

  } else if (cmd == 'a') {
    if (val != 0.0f) {
      if (gMode == Mode::IDLE) {
        echo.println("# ARM REFUSED: mode is IDLE. Select EDGE or CORNER first.");
      } else if (tiltError() < kArmGate) {
        gArmed = true;
        echo.println("# gArmed = TRUE");
      } else {
        gArmed = false;
        echo.print("# ARM REFUSED: |phi|="); echo.print(tiltError() * (float)RAD_TO_DEG, 3);
        echo.print(" deg exceeds ARM_GATE="); echo.print(kArmGate * (float)RAD_TO_DEG, 2);
        echo.println(" deg. Get closer to the resolved equilibrium and retry.");
      }
    } else {
      gArmed = false;
      echo.println("# gArmed = FALSE");
    }

  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
    echo.print("# gGainScale = "); echo.println(gGainScale, 3);

  } else if (cmd == 'c') {
    resolveCornerCandidate(echo);

  } else if (cmd == 'e') {
    resolveEdgeCandidate(echo);

  } else if (cmd == 'z') {
    if (val != 0.0f) {
      for (int i = 0; i < 3; ++i) gPhiOffset[i] += phi[i];
      echo.print("# gPhiOffset TARED to: ");
      for (int i = 0; i < 3; ++i) { echo.print(gPhiOffset[i] * (float)RAD_TO_DEG, 3); echo.print(i < 2 ? "," : " deg\n"); }
    } else {
      gPhiOffset[0] = gPhiOffset[1] = gPhiOffset[2] = 0.0f;
      echo.println("# gPhiOffset cleared to 0,0,0");
    }

  } else if (cmd == 'o') {
    gEdgeOffset = val * (float)DEG_TO_RAD;
    echo.print("# edge phi offset = "); echo.print(val, 3); echo.println(" deg");

  } else if (cmd == 'p') {
    gHalted = (val != 0.0f);
    if (gHalted && gArmed) { gArmed = false; echo.println("# disarmed by halt"); }
    if (gHalted) stopAllWheels();
    echo.print("# gHalted = "); echo.println(gHalted ? "TRUE" : "FALSE");

  } else if (cmd == 'h' || cmd == 'k') {
    // keepalive no-op -- receipt alone is the point, see pollCommands()

  } else {
    echo.println("# unknown. use: m<0/1/2> a<0/1> g<0..1> c e z<0/1> o<deg> "
                 "p<0/1> t<0/1> h|k u1");
  }
}

void pollCommands() {
  static char lineBuf[200];

  if (gUsbReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    if (gOtaMode) otaHandleLine(lineBuf, Serial);
    else if (lineBuf[0] == 't') gLinkMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
    else handleCommandLine(lineBuf, Serial);
  }

  if (gLinkReader.poll(Serial1, lineBuf, sizeof(lineBuf))) {
    gLastSerial1RxMillis = millis();      // any line at all counts as link-alive
    if (gOtaMode) otaHandleLine(lineBuf, Serial1);
    else if (lineBuf[0] == 't') gLinkMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
    else handleCommandLine(lineBuf, Serial1);
  }
}


// ----------------------------------------------------------------------------
// SECTION 8: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  const uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) delay(10);

  Serial1.begin(kLinkBaud);
  Serial1.addMemoryForRead(gSerial1RxBuf, sizeof(gSerial1RxBuf));
  gLastSerial1RxMillis = millis();

  gBoot.println("\nstarted - CUBLI OTA TEMPLATE (mode state machine + FlasherX)");
#if !CONTROL_LAW_TRANSPLANTED
  gBoot.println("# CONTROL_LAW_TRANSPLANTED = 0 -- no torque will ever be");
  gBoot.println("# commanded. Telemetry, modes, E-STOP and OTA are all live.");
#endif

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    gBoot.print("CAN error 0x"); gBoot.println(errorCode, HEX);
    delay(1000);
  }

  stopAllWheels();
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
  resolveCornerCandidate(gBoot);

  gBoot.println("# STARTS IN IDLE, DISARMED. m1 = edge, m2 = corner, m0 = idle.");
  gBoot.println("# Every mode change disarms, stops the wheels and clears the tare.");
  gBoot.println("# WiFi mode auto-disarms if no line arrives on Serial1 for 300 ms.");
  gBoot.println("# u1 begins a wireless firmware update (IDLE + disarmed only).");
}


// ----------------------------------------------------------------------------
// SECTION 9: loop()
// ----------------------------------------------------------------------------

void loop() {
  pollCommands();

  // While staging firmware there is no control loop and no telemetry: the
  // wheels are stopped, the cube is disarmed, and every spare cycle belongs to
  // draining Serial1 before the RX buffer fills behind a sector erase.
  if (gOtaMode) {
    if (millis() - gOtaLastRxMs > kOtaIdleTimeoutMs) {
      otaAbort(outStream(), "no records for 30 s -- uploader gone");   // reboots
    }
    return;
  }

  if (gLinkMode == LinkMode::WIFI && gArmed &&
      millis() - gLastSerial1RxMillis > kLinkTimeoutMs) {
    gArmed = false;
  }

  if (gHalted) return;

  static uint32_t nextSendMillis = 0;
  if (nextSendMillis == 0) nextSendMillis = millis();
  if (static_cast<int32_t>(millis() - nextSendMillis) < 0) return;
  nextSendMillis += kPeriodMs;

  const uint32_t time = millis();

  static uint32_t lastMicros = 0;
  static bool dtInitialized = false;
  const uint32_t nowMicros = micros();
  float dt = dtInitialized ? (nowMicros - lastMicros) * 1e-6f : kPeriodMs * 1e-3f;
  lastMicros = nowMicros;
  dtInitialized = true;
  if (dt <= 0.0f || dt > 0.5f) dt = kPeriodMs * 1e-3f;

  float aImu[3], wImu[3];
  imu.getSensorData();
  readIMURaw(imu, aImu, wImu);
  attitudeUpdate(aImu, wImu, dt);

  if (gMode == Mode::EDGE) updateEdgeProjection();
  else                     updateCornerProjection();

  float rho[3];
  readWheelSpeeds(rho);

  // ~5 s low-pass on wheel speed -- "standing speed", the number that says
  // whether the wheels are unwinding after each correction.
  const float alpha = dt / (5.0f + dt);
  for (int i = 0; i < 3; ++i) gRhoLp[i] += alpha * (rho[i] - gRhoLp[i]);
  gWheelLp += alpha * (rho[kAxis] - gWheelLp);

  switch (gMode) {
    case Mode::CORNER: cornerControlStep(rho); break;
    case Mode::EDGE:   edgeControlStep(rho[kAxis]); break;
    case Mode::IDLE:
      // Query-only keeps velocity, voltage and temperature live so the battery
      // gauge and wheel chart still read true while parked.
      moteusX.SetQuery(); moteusY.SetQuery(); moteusZ.SetQuery();
      break;
  }

  updateHealth();

  static uint8_t telemetryCount = 0;
  if (++telemetryCount >= kTelemetryDecim) {
    telemetryCount = 0;
    Stream& out = outStream();
    if (gMode == Mode::EDGE) telemetryEdge(out, time, rho[kAxis]);
    else                     telemetryCorner(out, time, rho);
  }

  static uint32_t lastHealthMs = 0;
  if (time - lastHealthMs >= kHealthIntervalMs) {
    lastHealthMs = time;
    telemetryHealth(outStream());
  }
}


// ============================================================================
// TRANSPLANTING A CONTROL LAW
// ----------------------------------------------------------------------------
// The two hooks in SECTION 4 are the only things standing between this file
// and a working balance build. What has to move, and from where:
//
//   CORNER  ../CornerBalance_WiFi/CornerBalance_WiFi.ino
//     - the full CornerCandidate struct WITH its float Kp[3][9] member and the
//       kCorners[8] table at line 382 (this file keeps only name/gB/offset);
//     - SECTION 2e in its entirety: kMaxTilt, kMaxOmega, kTauMax, kTaperStart,
//       the friction feedforward constants, kAxisWheelSign, kTorqueFormat and
//       the body of commandWheels() at line 633.
//
//   EDGE    ../EdgeBalance_WiFi/EdgeBalance_WiFi.ino
//     - EdgeCandidate WITH its float K[3] member and the kCandidates[3][2]
//       table at line 348;
//     - its SECTION 2e control body, driving only the kAxis wheel.
//
// Then set CONTROL_LAW_TRANSPLANTED to 1. The #error guards in both hooks fire
// if you flip the flag with a hook still empty, so the two cannot drift apart.
//
// THREE THINGS THAT WILL BITE
//
//   1. Both laws own gLastTau/gLastTauCmd, and both were written assuming they
//      are the only law in the binary. Confirm on a bench that a mode change
//      really does zero them -- setMode() does, but a law that caches its own
//      integrator state will need that cleared too.
//
//   2. The corner law's trips (overtilt, overspeed, NaN) clear gArmed by
//      themselves and never announce it. The telemetry `armed` column is
//      therefore the authority, not the '#' echoes -- which is exactly the
//      assumption dashboard/schemas.py:82 already documents. Keep it true.
//
//   3. Arm gate. This file states ONE value (kArmGate, 1.0 deg) and reports it
//      in its refusal line, which is how the console learns the real threshold.
//      Do not let a transplanted law reintroduce EdgeBalance_WiFi.ino:484's
//      0.1672664619 rad (9.58 deg) -- that value lets the edge build arm with
//      the cube leaning nearly ten degrees, and its own comment still claims
//      0.5 deg.
//
// None of the above is validated. Re-run the bring-up ladder, hand-held, with
// the console's E-STOP in reach.
// ============================================================================
