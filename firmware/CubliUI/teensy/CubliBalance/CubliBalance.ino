// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) --
// CUBLI BALANCE: EDGE + CORNER IN ONE BINARY, SWITCHED AT RUNTIME
//                                                              [WiFi build]
// ============================================================================
// This file is the FUSION of the two hardware-validated WiFi builds:
//
//   corner: 3D model/Gam/FINAL/WIFIMODE/corner-bringup/Stage4_FixedOffset/
//             Stage4_FixedOffset.ino        -- full law, fixed offset, no arm gate
//   edge:   3D model/Gam/PreFINAL/WIFIMODE/EdgeBalance_WiFi/
//             EdgeBalance_WiFi.ino          -- single-axis full law, arm gate
//
// NEITHER CONTROL PATH IS RETUNED HERE. Both laws, both candidate tables, both
// sets of gains, trips and saturation limits are copied VERBATIM from those
// files. What this file adds is the state machine that lets one binary be
// either of them, plus the collision fixes that copying two files into one
// translation unit forces (see SECTION 0b). Retune in the reference sketches
// and re-copy; do not tune here.
//
// WHY THIS EXISTS
// ---------------
// Until now edge and corner were separate binaries, so changing which feature
// the cube balances on meant re-flashing over USB. The wireless-flash console
// (MODE B) existed largely to work around that, and it was abandoned as
// unreliable. Rather than re-add OTA, the mode became a runtime property: the
// laptop sends m0/m1 over the link that already carries every other command.
//
// ---------------------------- STATE MACHINE ---------------------------------
//
//   gMode   in {EDGE, CORNER}                    -- which law is STAGED
//   gState  in {DISARMED, EDGE_BALANCE, CORNER_BALANCE}  -- what is RUNNING
//
// gMode is always defined, including while disarmed: it selects the estimator
// projection, the wheel set and the telemetry format. gState is the
// authoritative armed status -- isArmed() is (gState != DISARMED), and every
// trip in either control law routes through disarm() rather than poking a
// flag, so there is exactly one place that can stop the torque.
//
//   FROM              EVENT                       TO               NOTE
//   DISARMED          a1, gMode=CORNER            CORNER_BALANCE   no arm gate
//   DISARMED          a1, gMode=EDGE, in gate     EDGE_BALANCE
//   DISARMED          a1, gMode=EDGE, out of gate DISARMED         refuses, says so
//   any               a0                          DISARMED
//   any               m0 / m1                     DISARMED         see below
//   any               h1                          DISARMED         halt idles the loop
//   EDGE/CORNER_BAL.  tilt / omega / NaN trip     DISARMED
//   EDGE/CORNER_BAL.  link quiet kLinkTimeoutMs   DISARMED
//
// THE MODE-SWITCH GUARD, which is the whole safety story of this file:
// m<n> NEVER switches while armed. It disarms FIRST, SetStop()s all three
// wheels, re-initialises the new law's filters, re-resolves its candidate, and
// lands in DISARMED -- never in the new mode's armed state. Arming again is a
// separate, deliberate a1. So the sequence is always DISARM -> SWITCH -> ARM
// even when the operator only pressed one button, and if it had to force the
// disarm it prints "# MODE SWITCH: force-disarmed first" rather than doing it
// quietly.
//
// Every transition prints a machine-readable line, also emitted at boot and on
// demand with "s", so the laptop's view of the cube comes from the cube:
//
//     # state=CORNER_BALANCE mode=CORNER armed=1 halted=0
//
// ---------------------------- COMMAND GRAMMAR -------------------------------
//
//   a<0/1>    arm / disarm -- arms into whichever mode is staged
//   m<0/1>    MODE: m0 = EDGE, m1 = CORNER (force-disarms, see above)
//   g<0..1>   gain scale
//   c         re-resolve corner candidate      (corner mode)
//   e         re-resolve edge candidate        (edge mode)
//   o<deg>    edge tilt trim, ON TOP of kEdgePhiOffsetDefaultDeg (edge mode)
//   p<0..3>   LQR tilt-term gain scale  (this mode's own copy, see below)
//   r<0..3>   LQR rate-term gain scale  (   "      "    "     "    "   )
//   w<0..3>   LQR wheel-term gain scale (   "      "    "     "    "   )
//   h<0/1>    HALT / resume
//   d<N>      telemetry decimation, 1..100, applies to the current mode
//   l<0/1>    link mode: l0 = telemetry on USB, l1 = telemetry on Serial1
//   k         keepalive no-op (feeds the link watchdog)
//   s         print the state line
//
// p/r/w EACH MODE KEEPS ITS OWN THREE SCALES, default 1.0 (unscaled hardcoded
// law), applied at the point of use in commandWheelsCorner() / commandWheel
// Edge() -- the underlying gain tables are never modified. Setting one while
// the OTHER mode is staged is not refused, unlike c/e/o: it just stores the
// value for when you switch there, same as "o<deg>" already persists across
// a mode switch. A reflash always starts every scale back at 1.0.
//
// TWO GRAMMAR COLLISIONS WERE RESOLVED, AND BOTH CAN BITE:
//
//  1. 'h' IS HALT HERE. On EdgeBalance_WiFi.ino 'h' was a KEEPALIVE, accepted
//     alongside 'k' specifically so the old telemetry_python_wifi.py could run
//     unmodified. In this file a bare "h" parses as atof("") = 0.0 -> h0 ->
//     RESUME. Anything still sending "h" ten times a second therefore holds
//     the HALT button down in the released position. 'k' IS THE ONLY
//     KEEPALIVE. ../tools/telemetry_python_wifi.py and
//     ../tools/telemetry_python_wifi_corner.py were changed to send 'k' for
//     exactly this reason -- if you resurrect an older copy of either, check
//     its heartbeat before arming.
//
//  2. 't' IS GONE. The edge build used t<0/1> for link mode and the corner
//     build used l<0/1>. The corner letter won; 't' is simply not in the
//     grammar any more.
//
//  'x' IS NEVER USED and never can be: the XIAO bridge consumes x0/x1 as ITS
//  OWN mode switch and never relays them to the Teensy
//  (../../xiao/xiao_teensy_bridge/xiao_teensy_bridge.ino).
//
// ------------------------ LINK LOSS AUTO-DISARMS ----------------------------
// In WIFI mode, no line on Serial1 for kLinkTimeoutMs (3 s) force-disarms.
// Over the radio the laptop is the operator's only disarm button, so a dead
// link must not leave a law running. This uses the CORNER build's stricter
// form: enforced whenever WIFI mode is selected, even if the laptop has never
// spoken. (The edge build only armed this check once already armed, which
// would leave a USB "a1" with no laptop running unattended.)
//
// Note what that means in CORNER mode specifically: that build has NO arm gate
// (see SECTION 5), so the link watchdog and kMaxTilt's 25 deg trip are the
// only automatic protections left.
//
// ------------------------------- WIRING -------------------------------------
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   common GND, 1 Mbaud.  Full detail in ../../docs/WIRING.md.
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"


// ----------------------------------------------------------------------------
// SECTION 1b: STATE MACHINE TYPES
// ----------------------------------------------------------------------------
// These live UP HERE, above every function, on purpose. The Arduino .ino
// preprocessor generates forward prototypes for the file's functions and
// inserts them immediately before the FIRST function definition -- which is
// wheelObj() in SECTION 2. Declared any later, switchMode(BalanceMode, Print&)
// gets a generated prototype that mentions a type not yet declared, and the
// sketch fails to compile with "'BalanceMode' was not declared in this scope"
// pointing at a line that looks perfectly fine. Moving these back down will
// reintroduce that.
//
// NOT `Mode` and `State`: MoteusTeensy.h pulls in mjbots::moteus::Mode, and a
// bare `Mode` here is ambiguous against it at every use. The prefix is not
// decoration -- dropping it does not compile either.
//
// The semantics, and the transition table, are in SECTION 2a.

enum class BalanceMode  : uint8_t { EDGE = 0, CORNER = 1 };
enum class BalanceState : uint8_t { DISARMED = 0, EDGE_BALANCE = 1, CORNER_BALANCE = 2 };

static BalanceMode  gMode  = BalanceMode::CORNER;       // what a1 arms into
static BalanceState gState = BalanceState::DISARMED;    // boot state, always

static const char* kModeName[2]  = { "EDGE", "CORNER" };
static const char* kStateName[3] = { "DISARMED", "EDGE_BALANCE", "CORNER_BALANCE" };

static inline bool isArmed() { return gState != BalanceState::DISARMED; }


// ----------------------------------------------------------------------------
// SECTION 0b: NAMING -- what the fusion had to rename, and why
// ----------------------------------------------------------------------------
// The two reference files independently used the same identifiers for
// different things. Merging them into one translation unit would either fail
// to compile or, worse, compile and be wrong. The renames are mechanical, and
// they are listed here so a diff against either reference stays readable:
//
//   corner ref            edge ref                 here
//   --------------------  -----------------------  --------------------------
//   kPhiOffset[3] (const) kPhiOffset (live scalar) kCornerPhiOffset[3] /
//                                                    gEdgePhiOffsetRad
//   phi[3]                phi_edge, om_edge        gPhi[3] / gPhiEdge, gOmEdge
//   commandWheels()       commandWheel()           commandWheelsCorner() /
//                                                    commandWheelEdge()
//   printState()          printState()/telemetryPlot()  printCornerState() /
//                                                    printEdgePlot()
//   kMaxOmega = 209.4     kMaxOmega = 1e6          kCornerMaxOmega /
//                                                    kEdgeMaxOmega
//   gLastTau[3]           gLastTau                 gCornerTau[3] / gEdgeTau
//   gLastTauCmd[3]        gLastTauCmd              gCornerTauCmd[3] / gEdgeTauCmd
//   gRhoLp[3]             gWheelOmegaLp            gRhoLp[3] / gEdgeOmegaLp
//   moteusX/Y/Z           moteusActive             wheelObj(i) for both
//   -                     kAxis (compile-time)     gEdgeAxis (runtime)
//   -                     kArmGate                 kEdgeArmGate
//
// The kPhiOffset pair is the one that would have been genuinely dangerous: in
// the corner build it is a compile-time 3-vector of validated equilibrium
// offsets, in the edge build it is a live scalar the operator sets with "o".
// Same name, different type, different meaning, different lifetime.
//
// Everything NOT in that table -- the estimator, the mount DCM, the IMU
// calibration constants, the vector helpers, LineReader, DualPrint, the link
// layer, kMaxTilt, kTauMax, the friction feedforward, kAxisWheelSign,
// kTorqueFormat -- was byte-identical in both references and appears here
// exactly once.


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// Confirmed on bench, physically verified: id 2 -> X, id 3 -> Y, id 1 -> Z.
// Both reference files agree on this mapping.
Moteus moteusX(canBus, []() { Moteus::Options options; options.id = 2; return options; }());
Moteus moteusY(canBus, []() { Moteus::Options options; options.id = 3; return options; }());
Moteus moteusZ(canBus, []() { Moteus::Options options; options.id = 1; return options; }());

Moteus& wheelObj(int i) {
  return i == 0 ? moteusX : (i == 1 ? moteusY : moteusZ);
}

enum Axis { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };
static const char* kAxisName[3] = { "X", "Y", "Z" };
static const int8_t kAxisMoteusId[3] = { 2, 3, 1 };

// Which single wheel edge mode drives. Was a compile-time `kAxis` in
// EdgeBalance_WiFi.ino; it is a runtime global here only because the edge law
// now shares wheelObj() with the corner law and kWheelSign can no longer be a
// const initialised from it. Default AXIS_Y matches the reference -- Y[+1,+1]
// and Y[-1,-1] have by far the smallest place offsets (0.006/0.007 deg) and
// are the pair actually flown.
static Axis gEdgeAxis = AXIS_Y;

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz control loop -- NOT the telemetry rate

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Halting force-disarms (strict superset of "a0"), resuming does NOT re-arm.


// ----------------------------------------------------------------------------
// SECTION 2a: THE STATE MACHINE
// ----------------------------------------------------------------------------
// The types themselves are declared in SECTION 1b, above the first function --
// see the note there for why they cannot live here.
//
// gMode is the STAGED law (EDGE or CORNER): it selects the estimator
// projection, the wheel set and the telemetry format, and is defined at all
// times including while disarmed. gState is what is actually RUNNING.
//
// See the file header for the full transition table. The invariant that
// matters: nothing outside disarm() / armRequest() / switchMode() ever assigns
// gState, and every trip in either control law routes through disarm() rather
// than poking a flag -- which is what makes "what stopped the cube" answerable
// from the log.


// ---------------- Link mode + command channels ----------------
// Identical in both reference files -- fix a bug here and it is fixed for both
// laws.

// Emit telemetry every Nth control cycle, PER MODE. The defaults are each
// reference's own rate: edge ran undecimated at 500 Hz (10 short CSV fields),
// corner decimated by 2 to 250 Hz (26 fields, ~200 B -- ~50 kB/s, comfortable
// against the 1 Mbaud link, though the UDP hop is the real ceiling at ~105
// lines/s measured in link-bringup Stage 6). "d<N>" writes the CURRENT mode's
// entry, so setting a readable rate for one does not disturb the other.
static uint8_t gDecim[2] = { 1, 2 };   // [EDGE, CORNER]
static uint8_t gTelemetryCount = 0;
static const uint8_t kDecimMin = 1;
static const uint8_t kDecimMax = 100;

enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;   // boot default

const uint32_t kLinkBaud      = 1000000;   // must match TEENSY_LINK_BAUD in
                                            // the XIAO sketch
const uint32_t kLinkTimeoutMs = 3000;       // auto-disarm if WIFI mode and no
                                            // Serial1 line in this long
static uint32_t gLastSerial1RxMillis = 0;
static bool     gLinkAlive           = false;   // false at boot so the first
                                                 // packet from the laptop
                                                 // re-prints the header

// Small non-blocking line accumulator -- used for BOTH Serial and Serial1 so
// neither channel can ever stall loop() waiting on bytes that haven't
// arrived yet. Replaces the USB builds' blocking readStringUntil('\n').
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
// Only used outside the control path -- per-cycle telemetry goes to exactly
// one stream, chosen by gLinkMode.
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

// Where asynchronous console messages go -- the channel the operator is
// actually on.
static inline Stream& activeStream() {
  return (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
}


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (shared; identical in both references)
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
// SECTION 3: CORNER CANDIDATE TABLE AND PROJECTION
// ----------------------------------------------------------------------------
// Verbatim from Stage4_FixedOffset.ino.

struct CornerCandidate {
  const char* name;
  float gB[3];
  float Kp[3][9];   // rows = wheel X,Y,Z; cols = [phi(3) om(3) rho(3)]
  float placeOffsetDeg;
  float ellM;       // m, contact-to-COM lever arm -- used only to convert the
                     // fixed offset into an equivalent COM offset in mm for
                     // telemetry.
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
                  // is unaffected, multi-corner locomotion is not, until
                  // the pole is rebalanced or a counterweight is added.
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

// Same measurement-based resolution as the USB build; only the print sink is
// parameterised, so the 'c' command's answer comes back on whichever channel
// asked for it.
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
  if (bestDot - secondDot < 0.2f) {
    out.println("# WARNING: best and runner-up corners are close -- verify before arming.");
  }
}

float gPhi[3] = { 0.0f, 0.0f, 0.0f };

// FIXED OFFSET -- from perfect_equilibrium_2.log, mean over a 132.41 s
// continuous armed hold (2026-08-20), per-axis std < 0.0012 deg. That is the
// signature of a genuine fixed point, not a value still drifting: whatever
// residual COM/mounting error automatic trim was correcting for, it found it
// and stopped moving. ADDED to the raw measurement below, same sign/convention
// Stage4_AutoTrim.ino's live gTrim used (verified on the bench,
// cos(trim,wheels) = -1.0000).
//
// If the cube's mounting changes (battery moved, a cable re-routed, a bolt
// re-torqued) this number goes stale -- that is the tradeoff for dropping live
// adaptation. Re-run Stage4_AutoTrim.ino, let it re-converge, and update from
// the new result rather than assuming this one still holds. This file has no
// way to detect that its offset has gone stale on its own.
static const float kCornerPhiOffset[3] = {
  0.004580f,   // rad =  0.2624 deg
  0.003952f,   // rad =  0.2265 deg
  -0.008430f,  // rad = -0.4830 deg
};

void updateCornerProjection() {
  float t[3];
  cross3(kCorners[gCornerIdx].gB, ghat, t);
  gPhi[0] = -t[0] + kCornerPhiOffset[0];
  gPhi[1] = -t[1] + kCornerPhiOffset[1];
  gPhi[2] = -t[2] + kCornerPhiOffset[2];
}


// ----------------------------------------------------------------------------
// SECTION 4: EDGE CANDIDATE TABLE AND PROJECTION
// ----------------------------------------------------------------------------
// Verbatim from EdgeBalance_WiFi.ino.

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

void resolveEdgeCandidate(Print& out) {
  const float d0 = dot3(ghat, kCandidates[gEdgeAxis][0].gB);
  const float d1 = dot3(ghat, kCandidates[gEdgeAxis][1].gB);
  gEdgeIdx = (d0 >= d1) ? 0 : 1;
  out.print("# axis="); out.print(kAxisName[gEdgeAxis]);
  out.print("  edge candidate resolved: "); out.print(kCandidates[gEdgeAxis][gEdgeIdx].name);
  out.print("  K="); out.print(kCandidates[gEdgeAxis][gEdgeIdx].K[0], 4);
  out.print(","); out.print(kCandidates[gEdgeAxis][gEdgeIdx].K[1], 4);
  out.print(","); out.print(kCandidates[gEdgeAxis][gEdgeIdx].K[2], 5);
  out.print("  (dot0="); out.print(d0, 4);
  out.print(" dot1="); out.print(d1, 4); out.println(")");
  if (fabsf(d0 - d1) < 0.2f) {
    out.println("# WARNING: dot0 and dot1 are close -- verify before arming.");
  }
}

float gPhiEdge = 0.0f;
float gOmEdge  = 0.0f;

// Live-settable, NOT a sensor/mount calibration (that's already handled by
// gMountDCM and the accel/gyro constants above) -- this corrects for the
// cube's CURRENT physical resting point being genuinely offset from the
// designed equilibrium. PER AXIS: re-measure and re-set for whichever edge is
// currently active. Set live with "o<deg>", subtracted from gPhiEdge
// everywhere it is used (control, trips, arm gate, telemetry). Reset to 0 once
// the missing mass (battery cable, DC-DC) is mounted.
//
// This is the edge counterpart of kCornerPhiOffset above and they are NOT
// interchangeable -- one is a live scalar the operator tunes, the other a
// compile-time 3-vector. Both were called kPhiOffset in their source files.
//
// HARDCODED DEFAULT: -0.7 deg, bench-measured resting offset for the Y-axis
// edge (gEdgeAxis's default). Unlike kCornerPhiOffset, this is NOT the product
// of a long converged auto-trim hold -- there is no edge auto-trim mechanism
// yet (deliberately not built -- see the project notes). It is just the
// number that used to get typed in by hand with "o-0.7" every session, now
// baked in as the boot default instead. Re-measure and update this constant
// if the mount changes; "o<deg>" still works live on top of it for the same
// reason it always did (per-axis drift, a knob you shouldn't need every
// session but might occasionally).
//
// dashboard/schemas.py mirrors this value (EDGE_PHI_OFFSET_DEFAULT_DEG) for
// its "reset to default" button -- there is no protocol that carries this
// constant from firmware to dashboard, so if you change the number here,
// change it there too or the button will send the wrong value.
static const float kEdgePhiOffsetDefaultDeg = -0.7f;

// enterMode() deliberately does NOT reset this on a mode switch (see its body
// below) -- it is meant to survive m0/m1 within one boot session exactly like
// it survived nothing before. Only a reflash (back to the constant above) or
// an explicit "o<deg>" changes it.
static float gEdgePhiOffsetRad = kEdgePhiOffsetDefaultDeg * (float)DEG_TO_RAD;

void updateEdgeProjection() {
  const EdgeCandidate& c = kCandidates[gEdgeAxis][gEdgeIdx];
  float phi[3];
  { float t[3]; cross3(c.gB, ghat, t); phi[0]=-t[0]; phi[1]=-t[1]; phi[2]=-t[2]; }
  gPhiEdge = dot3(c.e, phi) - gEdgePhiOffsetRad;
  gOmEdge  = dot3(c.e, w_b);
}


// ----------------------------------------------------------------------------
// SECTION 5: CONTROL -- shared limits, then the two laws
// ----------------------------------------------------------------------------

static float gGainScale = 1.0f;   // already validated through Stage 3's ramp

// Shared, identical in both references. Loose: hand-held, watching combined-
// term behavior. Stage 5 tightens to the real DISARM/OMEGA_CAP policy from
// ../cubli_gains.h.
static const float kMaxTilt    = 0.4363f;   // rad, 25 deg
static const float kTauMax     = 0.12f;     // N*m, TAU_MAX -- the real physical
                                             // torque saturation, untouched
static const float kTaperStart = 36.0f;     // rad/s -- irrelevant in both modes
                                             // with the velocity caps below

// The one shared constant the two references DISAGREED on, kept split:
//   corner: loosened from the 40 rad/s policy value to the motor's real
//           mechanical speed rating, so there is still a genuine hardware
//           ceiling behind it rather than "effectively infinite".
//   edge:   the taper was choking off spin-up torque before the wheel reached
//           enough momentum to actually recover, so it was removed outright.
// Neither is the Stage 5 policy value. Do NOT carry either into Stage 5.
static const float kCornerMaxOmega = 209.43951f;   // rad/s (2000 RPM, motor rating)
static const float kEdgeMaxOmega   = 1.0e6f;       // rad/s -- effectively uncapped

// Friction feedforward -- Firmware Lessons: "not optional". PLACEHOLDER values
// from cubli_gains.h pending the real spin-down/breakaway test. Same values on
// all three wheels (no reason to expect them to differ between wheels of the
// same motor/mount design, but that is a PLACEHOLDER assumption too -- flag it
// if one wheel's standing speed clearly misbehaves relative to the other two).
static const float kTauCw  = 0.008f;   // N*m, PLACEHOLDER
static const float kBw     = 0.0f;     // N*m*s, PLACEHOLDER
static const float kEpsFf  = 0.05f;    // rad/s, tanh width

// ARM GATES DIFFER BY MODE, and this is a real safety asymmetry worth being
// explicit about rather than smoothing over:
//
// CORNER HAS NO ARM GATE. "a1" arms unconditionally regardless of tilt. The Kp
// control law was derived by linearizing around the corner's equilibrium and
// is only VALIDATED for small angles (the simulation study's own recovery
// envelope tops out around 2.7-3.1 deg worst case, see Cube-Performance-
// Envelope-Results.md). Arming from a large initial tilt commands full-
// authority torque from OUTSIDE that validated envelope. The ONLY remaining
// backstops are kMaxTilt (25 deg, an automatic disarm-on-trip) and the link
// watchdog -- both of which stop a runaway AFTER it has started, not before.
// Placing the cube near the resolved equilibrium before "a1" is OPERATOR
// DISCIPLINE here, not a code-enforced rule. The dashboard warns and asks for
// a second click; it deliberately does not block, because blocking would
// contradict the firmware and train you to ignore refusals.
//
// EDGE HAS A GATE, and its value is KNOWN SUSPECT. 0.1672664619 rad is
// 9.58 deg, but EdgeBalance_WiFi.ino's own comment beside it says 0.5 deg and
// cubli_gains.h's ARM_GATE is 0.00872664619 rad (0.50 deg) -- note the shared
// "72664619" tail. It looks exactly like a decimal-point typo. It is
// nonetheless COPIED VERBATIM here, deliberately, so this fusion stays a
// faithful port and the fix is a separate, deliberate change with its own
// bench check. Until then the dashboard assumes the conservative 0.50 deg and
// is the tighter of the two.
static const float kEdgeArmGate = 0.1672664619f;   // rad, 9.58 deg -- SEE ABOVE

// ----------------------------------------------------------------------------
// LIVE LQR GAIN SCALES -- tuning knobs, not calibration
// ----------------------------------------------------------------------------
// Three multiplicative scale factors per mode, grouped by physical term
// (tilt / rate / wheel-speed) rather than exposed per matrix element -- the
// corner law's Kp is a full 3x9 matrix PER CORNER (216 numbers across all
// eight), so per-element control was never a serious option. A scale per
// term, applied uniformly to whichever corner/edge candidate is currently
// resolved, is. The underlying kCorners[...].Kp / kCandidates[...].K tables
// are read-only and UNTOUCHED -- these scales are applied at the point of use
// in commandWheelsCorner() / commandWheelEdge(), not baked back into the
// tables. That is also why nothing special is needed to make "reflash resets
// to hardcoded" true: these are fresh globals, default 1.0, so a fresh boot
// IS the unscaled hardcoded law with no extra logic required.
//
// Commands (mode-gated like c/e/o, refused on the wrong mode): p<val> tilt,
// r<val> rate, w<val> wheel. Same three letters in both modes since only one
// mode is ever active. Clamped to kGainScaleTermMax so a typo can't silently
// multiply torque output by an order of magnitude.
static float gCornerKPhiScale = 1.0f, gCornerKOmScale = 1.0f, gCornerKRhoScale = 1.0f;
static float gEdgeKPhiScale   = 1.0f, gEdgeKOmScale   = 1.0f, gEdgeKRhoScale   = 1.0f;
static const float kGainScaleTermMax = 3.0f;   // sanity clamp, not a validated bound

// Per-axis: all three confirmed via each axis's own Stage 1 pulse test.
// Re-verify per axis rather than assuming if the mount or wiring changes
// (Firmware Lessons S4).
static const float kAxisWheelSign[3] = {
  1.0f,   // X -- CONFIRMED
  1.0f,   // Y -- CONFIRMED
  1.0f,   // Z -- CONFIRMED
};

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

// Corner telemetry state
static float gCornerTau[3]    = { 0.0f, 0.0f, 0.0f };
static float gCornerTauCmd[3] = { 0.0f, 0.0f, 0.0f };
static float gRhoLp[3]        = { 0.0f, 0.0f, 0.0f };   // standing speed, tau = 5 s

// Edge telemetry state
static float gEdgeTau      = 0.0f;
static float gEdgeTauCmd   = 0.0f;
static float gEdgeOmegaLp  = 0.0f;   // standing wheel speed, tau = 5 s


// ----------------------------------------------------------------------------
// SECTION 5a: STATE TRANSITIONS
// ----------------------------------------------------------------------------
// The only three functions permitted to assign gState. Everything else -- the
// trips inside both control laws, the link watchdog, the halt command -- calls
// disarm() rather than clearing a flag, which is what makes "what stopped the
// cube" answerable from the log.

static void printStateLine(Print& out) {
  out.print("# state="); out.print(kStateName[(uint8_t)gState]);
  out.print(" mode=");   out.print(kModeName[(uint8_t)gMode]);
  out.print(" armed=");  out.print(isArmed() ? 1 : 0);
  out.print(" halted="); out.println(gHalted ? 1 : 0);
}

/** Stop the law. Idempotent: safe to call from the 500 Hz control path, where
 *  it fires exactly once per event because an already-DISARMED cube returns
 *  immediately instead of re-printing at loop rate. */
static void disarm(const char* why) {
  if (gState == BalanceState::DISARMED) { return; }
  gState = BalanceState::DISARMED;
  Stream& out = activeStream();
  out.print("# DISARMED: "); out.println(why);
  printStateLine(out);
}

// Print&, not Stream&: these only ever print, and setup() calls enterMode()
// with gBoot, which is a DualPrint (a Print, not a Stream). Stream& would
// compile everywhere except that one call.
static void armRequest(Print& echo) {
  if (gHalted) {
    echo.println("# ARM REFUSED: halted -- send h0 first");
    return;
  }
  if (gMode == BalanceMode::EDGE) {
    // Written as !(x < gate) rather than (x >= gate) so a NaN phi refuses
    // instead of arming.
    if (!(fabsf(gPhiEdge) < kEdgeArmGate)) {
      echo.print("# ARM REFUSED: |phi_edge|="); echo.print(fabsf(gPhiEdge) * (float)RAD_TO_DEG, 3);
      echo.print(" deg exceeds ARM_GATE="); echo.print(kEdgeArmGate * (float)RAD_TO_DEG, 2);
      echo.println(" deg. Get closer to vertical and retry.");
      return;
    }
    gState = BalanceState::EDGE_BALANCE;
  } else {
    // NO ARM GATE -- see the long note in SECTION 5. Operator's call.
    gState = BalanceState::CORNER_BALANCE;
    echo.println("# armed with no arm-gate check -- operator's call");
  }
  printStateLine(echo);
}

/** Re-initialise whichever law is now staged. Always leaves the cube DISARMED.
 *
 *  SetStop() on ALL THREE wheels, not just the ones the outgoing law drove:
 *  in edge mode the two inactive wheels are never commanded again, so they
 *  must be left explicitly stopped rather than relying on a moteus watchdog
 *  lapse to get there. The per-mode low-pass filters and last-torque values
 *  are cleared too, so the new law's first telemetry line is its own reading
 *  and not a stale echo of the previous mode. */
static void enterMode(Print& out) {
  moteusX.SetStop();
  moteusY.SetStop();
  moteusZ.SetStop();

  gRhoLp[0] = gRhoLp[1] = gRhoLp[2] = 0.0f;
  gCornerTau[0] = gCornerTau[1] = gCornerTau[2] = 0.0f;
  gCornerTauCmd[0] = gCornerTauCmd[1] = gCornerTauCmd[2] = 0.0f;
  gEdgeOmegaLp = gEdgeTau = gEdgeTauCmd = 0.0f;
  gTelemetryCount = 0;

  if (gMode == BalanceMode::CORNER) { resolveCornerCandidate(out); }
  else                        { resolveEdgeCandidate(out); }
  printStateLine(out);
}

/** m0 / m1. Disarms FIRST -- this never switches a running law out from under
 *  itself, and never lands in the new mode's armed state. See the header. */
static void switchMode(BalanceMode m, Print& echo) {
  if (isArmed()) {
    gState = BalanceState::DISARMED;
    echo.println("# MODE SWITCH: force-disarmed first");
  }
  gMode = m;
  echo.print("# MODE = "); echo.println(kModeName[(uint8_t)gMode]);
  enterMode(echo);
  echo.println("# still DISARMED -- send a1 to arm into the new mode");
}


// ----------------------------------------------------------------------------
// SECTION 5b: CORNER LAW -- full law: phi + om + rho, plus friction FF
// ----------------------------------------------------------------------------

void commandWheelsCorner(const float rho[3]) {
  // x = [phi(3); om(3); rho(3)] -- full state, all nine columns of each
  // wheel's Kp row contribute.
  const float xVec[9] = {
    gPhi[0], gPhi[1], gPhi[2],
    w_b[0], w_b[1], w_b[2],
    rho[0], rho[1], rho[2],
  };

  // Per-term live scale, applied at the point of use -- see the "LIVE LQR
  // GAIN SCALES" note in SECTION 5. kCorners[...].Kp itself is never modified.
  const float termScale[9] = {
    gCornerKPhiScale, gCornerKPhiScale, gCornerKPhiScale,
    gCornerKOmScale,  gCornerKOmScale,  gCornerKOmScale,
    gCornerKRhoScale, gCornerKRhoScale, gCornerKRhoScale,
  };

  float tau[3];
  for (int i = 0; i < 3; ++i) {
    const float* row = kCorners[gCornerIdx].Kp[i];
    float u = 0.0f;
    for (int j = 0; j < 9; ++j) { u -= row[j] * termScale[j] * xVec[j]; }
    u *= gGainScale;
    u += kTauCw * tanhf(rho[i] / kEpsFf) + kBw * rho[i];

    const bool spinning_up = (u >= 0.0f) == (rho[i] >= 0.0f);
    if (spinning_up) {
      float s = (kCornerMaxOmega - fabsf(rho[i])) / (kCornerMaxOmega - kTaperStart);
      s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
      u *= s;
    }

    if (!isfinite(u)) { u = 0.0f; disarm("nonfinite torque"); }
    u = u >  kTauMax ?  kTauMax : u;
    u = u < -kTauMax ? -kTauMax : u;
    tau[i] = u;
  }

  if (norm3(gPhi) > kMaxTilt) { disarm("tilt > kMaxTilt"); }
  for (int i = 0; i < 3; ++i) {
    if (fabsf(rho[i]) > kCornerMaxOmega) { disarm("wheel over speed"); }
  }
  if (!isfinite(gPhi[0]) || !isfinite(gPhi[1]) || !isfinite(gPhi[2]) ||
      !isfinite(w_b[0]) || !isfinite(w_b[1]) || !isfinite(w_b[2])) {
    disarm("nonfinite state");
  }

  static const float kLpTau = 5.0f;
  const float dtNom = kPeriodMs * 1e-3f;
  const float alphaLp = dtNom / (kLpTau + dtNom);

  for (int i = 0; i < 3; ++i) {
    const float tau_cmd = isArmed() ? tau[i] : 0.0f;
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
    gCornerTau[i]    = tau[i];
    gCornerTauCmd[i] = tau_cmd;
  }
}


// ----------------------------------------------------------------------------
// SECTION 5c: EDGE LAW -- full law: K_phi + K_om + K_rho, plus friction FF
// ----------------------------------------------------------------------------

void commandWheelEdge(float wheel_omega) {
  const EdgeCandidate& c = kCandidates[gEdgeAxis][gEdgeIdx];
  const float wheelSign = kAxisWheelSign[gEdgeAxis];

  // Per-term live scale, applied at the point of use, same pattern as
  // commandWheelsCorner() -- c.K itself is never modified.
  float tau = -(c.K[0] * gEdgeKPhiScale * gPhiEdge
              + c.K[1] * gEdgeKOmScale  * gOmEdge
              + c.K[2] * gEdgeKRhoScale * wheel_omega) * gGainScale;
  tau += kTauCw * tanhf(wheel_omega / kEpsFf) + kBw * wheel_omega;

  const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
  if (spinning_up) {
    float s = (kEdgeMaxOmega - fabsf(wheel_omega)) / (kEdgeMaxOmega - kTaperStart);
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    tau *= s;
  }

  if (!isfinite(tau)) { tau = 0.0f; disarm("nonfinite torque"); }

  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  if (fabsf(gPhiEdge) > kMaxTilt)          { disarm("tilt > kMaxTilt"); }
  if (fabsf(wheel_omega) > kEdgeMaxOmega)  { disarm("wheel over speed"); }
  if (!isfinite(gPhiEdge) || !isfinite(gOmEdge)) { disarm("nonfinite state"); }

  const float tau_cmd = isArmed() ? tau : 0.0f;

  Moteus::PositionMode::Command cmd;
  cmd.position               = NaN;
  cmd.velocity               = 0.0f;
  cmd.kp_scale                = 0.0f;
  cmd.kd_scale                 = 0.0f;
  cmd.feedforward_torque       = wheelSign * tau_cmd;
  cmd.maximum_torque           = kTauMax;
  cmd.watchdog_timeout          = 0.10f;
  cmd.ignore_position_bounds    = 1.0f;
  wheelObj(gEdgeAxis).SetPosition(cmd, &kTorqueFormat);

  static const float kLpTau = 5.0f;
  const float dtNom = kPeriodMs * 1e-3f;
  const float alphaLp = dtNom / (kLpTau + dtNom);
  gEdgeOmegaLp += alphaLp * (wheel_omega - gEdgeOmegaLp);

  gEdgeTau    = tau;
  gEdgeTauCmd = tau_cmd;
}


// ----------------------------------------------------------------------------
// SECTION 6: TELEMETRY
// ----------------------------------------------------------------------------
// BOTH WIRE FORMATS ARE COPIED VERBATIM AND MUST STAY THAT WAY. Everything on
// the laptop side -- dashboard/schemas.py, tools/analysis/plot_session_csv.py,
// fft_tilt_analysis.py, every recording already on disk -- identifies the
// build purely by FIELD COUNT. Adding a "mode" column would have been the
// obvious way to announce a mode switch and it would have broken all of them;
// the "# state=" console line carries that instead.
//
//   CORNER  26 fields, TAB-delimited, ~250 Hz
//     t_ms  phi_x_deg phi_y_deg phi_z_deg  om_x_dps om_y_dps om_z_dps
//     rho_x rho_y rho_z  rho_x_lp rho_y_lp rho_z_lp
//     tau_x tau_y tau_z  tau_cmd_x tau_cmd_y tau_cmd_z  armed gain_scale
//     trim_x_deg trim_y_deg trim_z_deg trim_com_mm trim_enabled
//
//   EDGE    10 fields, COMMA-delimited, ~500 Hz
//     t_ms  phi_edge_deg om_edge_dps  tau_Nm tau_cmd_Nm
//     armed gain_scale wheel_omega_lp wheel_pos wheel_vel
//
// So a mode switch changes the packet width mid-session. That is expected and
// already handled: the dashboard logs the change, re-sends the resolve command
// and rolls the CSV to a new file rather than mixing two widths in one.

// The corner header goes out bare (no '#'), byte-identical to the reference,
// because the analysis tools recognise this exact 26-column shape. It has the
// right width and non-numeric cells, so parsers skip it as a header row.
void printCornerTelemetryHeader(Print& out) {
  out.println("t_ms\tphi_x_deg\tphi_y_deg\tphi_z_deg\t"
               "om_x_dps\tom_y_dps\tom_z_dps\trho_x\trho_y\trho_z\t"
               "rho_x_lp\trho_y_lp\trho_z_lp\t"
               "tau_x\ttau_y\ttau_z\ttau_cmd_x\ttau_cmd_y\ttau_cmd_z\t"
               "armed\tgain_scale\t"
               "trim_x_deg\ttrim_y_deg\ttrim_z_deg\ttrim_com_mm\ttrim_enabled");
}

// The edge stream is plain CSV with NO header by design -- the plotters parse
// every non-'#' line as data. So its column list is emitted as a comment,
// which is informative in a terminal and invisible to a parser.
void printEdgeTelemetryHeader(Print& out) {
  out.println("# columns: t_ms,phi_edge_deg,om_edge_dps,tau_Nm,tau_cmd_Nm,"
               "armed,gain_scale,wheel_omega_lp,wheel_pos,wheel_vel");
}

void printTelemetryHeader(Print& out) {
  if (gMode == BalanceMode::CORNER) { printCornerTelemetryHeader(out); }
  else                        { printEdgeTelemetryHeader(out); }
}

void printCornerState(Stream& out, uint32_t t_ms, const float rho[3]) {
  out.print(t_ms);
  out.print('\t'); out.print(gPhi[0] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(gPhi[1] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(gPhi[2] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(w_b[0] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(w_b[1] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(w_b[2] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(rho[0], 3);
  out.print('\t'); out.print(rho[1], 3);
  out.print('\t'); out.print(rho[2], 3);
  out.print('\t'); out.print(gRhoLp[0], 3);
  out.print('\t'); out.print(gRhoLp[1], 3);
  out.print('\t'); out.print(gRhoLp[2], 3);
  out.print('\t'); out.print(gCornerTau[0], 4);
  out.print('\t'); out.print(gCornerTau[1], 4);
  out.print('\t'); out.print(gCornerTau[2], 4);
  out.print('\t'); out.print(gCornerTauCmd[0], 4);
  out.print('\t'); out.print(gCornerTauCmd[1], 4);
  out.print('\t'); out.print(gCornerTauCmd[2], 4);
  out.print('\t'); out.print(isArmed() ? 1 : 0);
  out.print('\t'); out.print(gGainScale, 2);
  // The trim five are CONSTANTS in this build (kCornerPhiOffset, and
  // trim_enabled = 0 always), kept only so the line stays the 26-column shape
  // the existing analysis tools already recognize. trim_enabled = 0 signals
  // "fixed, not adapting" to anyone comparing logs across files.
  out.print('\t'); out.print(kCornerPhiOffset[0] * (float)RAD_TO_DEG, 4);
  out.print('\t'); out.print(kCornerPhiOffset[1] * (float)RAD_TO_DEG, 4);
  out.print('\t'); out.print(kCornerPhiOffset[2] * (float)RAD_TO_DEG, 4);
  out.print('\t'); out.print(norm3(kCornerPhiOffset) * kCorners[gCornerIdx].ellM * 1000.0f, 3);
  out.print('\t'); out.println(0);
}

void printEdgePlot(Stream& out, uint32_t t_ms, const Moteus::Query::Result& v) {
  out.print(t_ms);
  out.print(','); out.print(gPhiEdge * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(gOmEdge  * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(gEdgeTau, 6);
  out.print(','); out.print(gEdgeTauCmd, 6);
  out.print(','); out.print(isArmed() ? 1 : 0);
  out.print(','); out.print(gGainScale, 4);
  out.print(','); out.print(gEdgeOmegaLp, 6);
  out.print(','); out.print(v.position, 6);
  out.print(','); out.println(v.velocity, 6);
}


// ----------------------------------------------------------------------------
// SECTION 7: COMMANDS (from either Serial or Serial1)
// ----------------------------------------------------------------------------
// `echo` is whichever stream the line arrived on, so an ack goes back down the
// channel it came in on. Mode-specific letters answer on the wrong mode rather
// than being silently swallowed -- a command that appears to work and does
// nothing is worse than a refusal.

void handleCommandLine(const char* line, Stream& echo) {
  if (line[0] == '\0') { return; }

  const char cmd = line[0];
  const float val = atof(line + 1);

  if (cmd == 'a') {
    if (val != 0.0f) {
      armRequest(echo);
    } else {
      if (isArmed()) { disarm("a0"); }
      else           { printStateLine(echo); }
    }

  } else if (cmd == 'm') {
    switchMode(val != 0.0f ? BalanceMode::CORNER : BalanceMode::EDGE, echo);

  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
    echo.print("# gGainScale = "); echo.println(gGainScale, 3);

  } else if (cmd == 'c') {
    if (gMode == BalanceMode::CORNER) { resolveCornerCandidate(echo); }
    else { echo.println("# 'c' is corner-only -- send m1 first (this is EDGE mode)"); }

  } else if (cmd == 'e') {
    if (gMode == BalanceMode::EDGE) { resolveEdgeCandidate(echo); }
    else { echo.println("# 'e' is edge-only -- send m0 first (this is CORNER mode)"); }

  } else if (cmd == 'o') {
    if (gMode == BalanceMode::EDGE) {
      gEdgePhiOffsetRad = val * (float)DEG_TO_RAD;
      echo.print("# edge phi offset = "); echo.print(val, 4); echo.println(" deg");
    } else {
      echo.println("# 'o' is edge-only -- corner uses a compile-time offset");
    }

  } else if (cmd == 'p' || cmd == 'r' || cmd == 'w') {
    // Live LQR gain-term scales -- see the SECTION 5 note. Unlike c/e/o these
    // have NO wrong mode: corner and edge each keep their own three scales at
    // all times, so setting one while the other mode is staged just stores it
    // for when you switch there, the same way "o<deg>" already persists
    // across a mode switch. Nothing to refuse.
    const float scale = val < 0.0f ? 0.0f
                       : (val > kGainScaleTermMax ? kGainScaleTermMax : val);
    float* target;
    const char* label;
    if (gMode == BalanceMode::CORNER) {
      target = (cmd == 'p') ? &gCornerKPhiScale
              : (cmd == 'r') ? &gCornerKOmScale : &gCornerKRhoScale;
      label  = (cmd == 'p') ? "corner tilt" : (cmd == 'r') ? "corner rate" : "corner wheel";
    } else {
      target = (cmd == 'p') ? &gEdgeKPhiScale
              : (cmd == 'r') ? &gEdgeKOmScale : &gEdgeKRhoScale;
      label  = (cmd == 'p') ? "edge tilt" : (cmd == 'r') ? "edge rate" : "edge wheel";
    }
    *target = scale;
    echo.print("# "); echo.print(label); echo.print(" gain scale = "); echo.println(scale, 3);

  } else if (cmd == 'h') {
    gHalted = (val != 0.0f);
    if (gHalted) { disarm("halt"); }
    echo.print("# gHalted = ");
    echo.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                          : "FALSE (resumed, still DISARMED -- send a1)");

  } else if (cmd == 'd') {
    const int n = atoi(line + 1);
    const uint8_t d = (uint8_t)(n < kDecimMin ? kDecimMin
                                : (n > kDecimMax ? kDecimMax : n));
    gDecim[(uint8_t)gMode] = d;
    gTelemetryCount = 0;
    echo.print("# decim["); echo.print(kModeName[(uint8_t)gMode]);
    echo.print("] = "); echo.print(d);
    echo.print("  ("); echo.print(500.0f / (float)d, 1);
    echo.println(" Hz)");
    printTelemetryHeader(echo);   // rate changed -- restate what the columns are

  } else if (cmd == 's') {
    printStateLine(echo);

  } else if (cmd == 'k') {
    // keepalive / no-op -- receipt alone is enough (see pollCommands()).

  } else {
    echo.println("# unknown. use: a<0/1> (arm)  m<0/1> (mode edge/corner)  "
                  "g<0..1> (gain)  c (corner resolve)  e (edge resolve)  "
                  "o<deg> (edge trim)  p/r/w<0..3> (LQR tilt/rate/wheel scale)  "
                  "h<0/1> (halt)  d<N> (decim)  l<0/1> (link)  s (state)  "
                  "k (keepalive)");
  }
}

// Polls BOTH Serial and Serial1 every loop() iteration -- see LineReader above
// for why this can never block. 'l'-prefixed lines switch gLinkMode locally
// and are NOT passed to handleCommandLine().
void pollCommands() {
  static char lineBuf[64];

  if (gUsbReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    if (lineBuf[0] == 'l') {
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
    if (!gLinkAlive) {
      // The laptop just showed up -- it missed the boot header, so restate the
      // columns and the current state.
      gLinkAlive = true;
      printTelemetryHeader(Serial1);
      printStateLine(Serial1);
    }
    if (lineBuf[0] == 'l') {
      gLinkMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
    } else {
      handleCommandLine(lineBuf, Serial1);
    }
  }
}


// ----------------------------------------------------------------------------
// SECTION 8: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // Bounded wait, NOT while(!Serial){} -- this build must still boot into WIFI
  // mode with nothing attached to USB at all.
  uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) { delay(10); }

  Serial1.begin(kLinkBaud);
  gLastSerial1RxMillis = millis();

  gBoot.println("\nstarted - CUBLI BALANCE: EDGE + CORNER, runtime mode switch (WiFi build)");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    gBoot.print("CAN error 0x");
    gBoot.println(errorCode, HEX);
    delay(1000);
  }

  moteusX.SetStop();
  moteusY.SetStop();
  moteusZ.SetStop();
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

  // 800 Hz, not 400 -- and this is a DELIBERATE DIVERGENCE from the edge
  // reference, which used 400. cube-bringup/Stage0c_IMUJitter.ino already
  // found and documented the problem (Phase 0.4): this loop runs at 500 Hz
  // (kPeriodMs = 2), so an IMU output rate of 400 Hz means the loop polls
  // FASTER than the sensor produces new samples -- roughly 1 in 5 reads is a
  // stale repeat of the previous one, not new information. Stage0c tested and
  // validated 800 Hz specifically to fix this (bwp left at default there too
  // -- see that file's own TODO on re-deriving group delay for the real
  // (ODR, bwp) pair if it ever matters). The corner reference already carried
  // the fix; the edge one predates it.
  if (imu.setAccelODR(BMI2_ACC_ODR_800HZ) != BMI2_OK ||
      imu.setGyroODR(BMI2_GYR_ODR_800HZ)  != BMI2_OK) {
    gBoot.println("Warning: could not raise BMI270 ODR to 800 Hz");
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

  // Resolve the boot mode's candidate and announce the state. enterMode()
  // re-stops the wheels, which is redundant right after the SetStop() above --
  // harmless, and it keeps "every mode entry stops every wheel" true without
  // an exception for the first one.
  enterMode(gBoot);
  printTelemetryHeader(gBoot);

  gBoot.println("# STARTS DISARMED, in CORNER mode. m0 = EDGE, m1 = CORNER.");
  gBoot.println("# A mode switch always force-disarms first and leaves the cube");
  gBoot.println("# DISARMED -- arming again is a separate a1.");
  gBoot.println("# CORNER HAS NO ARM GATE: a1 arms at any tilt. Place the cube");
  gBoot.println("#   near the resolved corner's equilibrium before arming anyway.");
  gBoot.print(  "#   Fixed offset (not adapting): ");
  gBoot.print(kCornerPhiOffset[0]*(float)RAD_TO_DEG, 4);
  gBoot.print(", "); gBoot.print(kCornerPhiOffset[1]*(float)RAD_TO_DEG, 4);
  gBoot.print(", "); gBoot.print(kCornerPhiOffset[2]*(float)RAD_TO_DEG, 4);
  gBoot.println(" deg (from perfect_equilibrium_2.log).");
  gBoot.print(  "# EDGE HAS A GATE at ");
  gBoot.print(kEdgeArmGate * (float)RAD_TO_DEG, 2);
  gBoot.println(" deg -- KNOWN SUSPECT, see SECTION 5.");
  gBoot.print(  "#   Edge offset boots at kEdgePhiOffsetDefaultDeg = ");
  gBoot.print(kEdgePhiOffsetDefaultDeg, 2);
  gBoot.println(" deg; o<deg> still overrides it live.");
  gBoot.println("# p/r/w<0..3> live-scale this mode's LQR tilt/rate/wheel gain");
  gBoot.println("#   terms (default 1.0 = unscaled hardcoded law, per mode).");
  gBoot.println("# Velocity caps are loose in BOTH modes -- a0 (disarm) is the");
  gBoot.println("#   real safety net, along with kMaxTilt's 25 deg auto-trip.");
  gBoot.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");
  gBoot.println("# d<N> sets THIS mode's telemetry decimation, l<0/1> picks");
  gBoot.println("#   USB/WiFi telemetry, s prints the state line.");
  gBoot.println("# 'k' IS THE ONLY KEEPALIVE -- 'h' is HALT here, not a no-op.");
  gBoot.println("#   AUTO-DISARMS if no line arrives on Serial1 for 3 s.");

  gNextSendMillis = millis();
}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 9: loop()
// ----------------------------------------------------------------------------

void loop() {
  pollCommands();

  // Link watchdog -- the laptop is the operator's only disarm button in WiFi
  // mode. Enforced whenever WIFI mode is selected, even if the laptop has
  // never spoken (gLinkAlive still false); otherwise a USB "a1" with no laptop
  // would leave a law running unattended. In CORNER mode this and kMaxTilt are
  // the only automatic protections, because that mode has no arm gate.
  if (gLinkMode == LinkMode::WIFI &&
      millis() - gLastSerial1RxMillis > kLinkTimeoutMs) {
    disarm("link lost");
    gLinkAlive = false;
  }

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

  // --- shared: one IMU read, one estimator update, whatever the mode ---
  float aImu[3], wImu[3];
  imu.getSensorData();
  readIMURaw(imu, aImu, wImu);
  attitudeUpdate(aImu, wImu, dt);

  const bool emit = (++gTelemetryCount >= gDecim[(uint8_t)gMode]);
  if (emit) { gTelemetryCount = 0; }

  if (gMode == BalanceMode::CORNER) {
    updateCornerProjection();

    const float rho[3] = {
      kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI,
      kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI,
      kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI,
    };

    commandWheelsCorner(rho);
    if (emit) { printCornerState(activeStream(), time, rho); }

  } else {
    updateEdgeProjection();

    // Only the active wheel is read AND only the active wheel is commanded --
    // the other two were SetStop()ed on mode entry and are deliberately left
    // alone, so their last_result() is stale and must not be used.
    const auto& v = wheelObj(gEdgeAxis).last_result().values;
    const float wheel_omega = kAxisWheelSign[gEdgeAxis] * v.velocity * 2.0f * (float)PI;

    commandWheelEdge(wheel_omega);
    if (emit) { printEdgePlot(activeStream(), time, v); }
  }
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// This file vs its two references: the control paths are copied, not rewritten.
// If something looks wrong in the corner law, check Stage4_FixedOffset.ino has
// the same problem before assuming the fusion caused it; likewise
// EdgeBalance_WiFi.ino for the edge law. What IS new here, and therefore where
// a fusion-specific bug would live:
//
//   - the state machine (SECTION 5a) and everything that calls disarm()
//   - the per-mode decimation split (gDecim[2])
//   - the renames in SECTION 0b
//   - the mode dispatch in loop() and the mode guards in handleCommandLine()
//
// Known follow-ups, in the order they are worth doing:
//   - kEdgeArmGate is 9.58 deg and almost certainly wants to be 0.50. Fix it
//     deliberately, with a bench check, not as a drive-by.
//   - gEdgeAxis has no command letter. It is a runtime global but nothing can
//     change it at runtime, so edge mode is still Y-only in practice. Adding
//     a letter means adding it to dashboard/schemas.py's whitelist too.
//   - Both velocity caps are bring-up values. Stage 5 tightens to the real
//     DISARM/OMEGA_CAP policy from ../cubli_gains.h.
//   - kCornerPhiOffset goes stale after ANY mechanical change to that corner.
// ============================================================================
