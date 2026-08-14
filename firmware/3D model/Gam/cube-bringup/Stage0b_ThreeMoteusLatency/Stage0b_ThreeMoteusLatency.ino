// ============================================================================
// TEENSY 4.1 + 3x moteus-n1 (CAN3) — PHASE 0.3: THREE-MOTEUS 400 Hz LATENCY
// ============================================================================
// Run this only after Stage0_SingleMoteusQuery has passed on ALL THREE
// wheel IDs individually. This sketch adds the thing a single moteus
// cannot show you: CAN-FD bus contention from three nodes queried in the
// same 2.5 ms slot.
//
// PASS CRITERION (Phase 0.3): p99.9 round-trip latency under 2.0 ms, ZERO
// dropped cycles over 10 minutes. Per the brief: "the tail matters, not
// the mean -- one dropped cycle is a full sample of extra delay." So a
// timed-out SetPosition() call is NOT excluded from the latency stats --
// its measured wall-clock time (however long the blocking call actually
// took before giving up) goes into the histogram like any other sample.
// Silently dropping timeouts from the distribution would hide exactly the
// tail behavior this test exists to catch.
//
// No IMU here either -- SPI and CAN-FD share the Teensy's DMA/interrupt
// budget, and this test isolates CAN timing from that interaction. Stage0c
// covers the IMU side alone; the two are combined only once both pass
// independently (that combination is the real 400 Hz Gam loop, not a
// separate bring-up stage).
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// Widened for the same reason as Stage0 -- we want a slow reply MEASURED,
// not folded into "no reply" right at the boundary we're testing.
static Moteus::Options MakeOptions(int8_t id) {
  Moteus::Options options;
  options.id = id;
  options.min_rcv_wait_us = 5000;
  return options;
}

// Confirmed on bench, physically verified per wheel, via
// Stage0_SingleMoteusQuery: id 2 -> X, id 3 -> Y, id 1 -> Z.
Moteus moteusX(canBus, MakeOptions(2));
Moteus moteusY(canBus, MakeOptions(3));
Moteus moteusZ(canBus, MakeOptions(1));
Moteus* const gWheels[3] = { &moteusX, &moteusY, &moteusZ };

// DIAGNOSTIC ONLY -- not a fix, and not meant to ship. Second run (X=100%,
// Y=Z=0%, every cycle) is consistent with EITHER the shared-receive-queue
// discard theory (see NOTES) OR contention on ACAN_T4's single dedicated
// TX mailbox for CAN-FD data frames (tryToSendDataFrameFD uses exactly one
// hardware mailbox, not one per node, backed by a 16-deep software buffer).
// Source reading alone can't tell which -- this is the cheap test. If a
// gap here changes the 100/0/0 pattern at all, that implicates timing/
// mailbox contention; if it makes no difference, that points back at the
// receive-discard theory instead. Set to 0 to restore the original
// back-to-back behavior once this has told us something.
static const uint32_t kInterQueryDelayUs = 300;

static const uint32_t kPeriodUs   = 2500;          // 400 Hz, matches FS_HZ in cubli_gains.h
static const uint32_t kDurationUs = 600000000UL;   // 10 minutes
static const uint32_t kStatusEveryUs = 5000000UL;  // live status line every 5 s

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

static bool gHalted = false;


// ----------------------------------------------------------------------------
// SECTION 2b: PER-CALL LATENCY HISTOGRAM (pooled across all 3 wheels)
// ----------------------------------------------------------------------------
// 20 bins x 100 us covering 0-2000 us (the region the pass criterion cares
// about), plus one overflow bin for >=2000 us -- that bin alone already
// means p99.9 has failed if it holds more than 0.1% of samples, so finer
// resolution above 2 ms buys nothing.

static const int kNumBins = 21;              // 20 fine + 1 overflow
static const uint32_t kBinWidthUs = 100;
static uint32_t gHist[kNumBins] = {0};
static uint64_t gLatSamples = 0;
static uint32_t gLatMinUs = 0xFFFFFFFFUL;
static uint32_t gLatMaxUs = 0;
static uint64_t gLatSumUs = 0;
static uint32_t gNoReplyCount = 0;

// Per-wheel breakdown -- added after a run where noReply was consistently
// exactly 2x cycles (not ~2x, EXACTLY, for 80+ seconds): too regular to be
// bus loading, and the pooled counters alone can't say whether it's always
// the same wheel losing out or whether it rotates. gWheels[0..2] = X,Y,Z.
static uint32_t gWheelReplies[3] = {0, 0, 0};
static uint32_t gWheelNoReply[3] = {0, 0, 0};

void recordLatency(int wheelIdx, uint32_t rtUs, bool ok) {
  int bin = (int)(rtUs / kBinWidthUs);
  if (bin >= kNumBins) { bin = kNumBins - 1; }
  gHist[bin]++;
  gLatSamples++;
  gLatSumUs += rtUs;
  if (rtUs < gLatMinUs) { gLatMinUs = rtUs; }
  if (rtUs > gLatMaxUs) { gLatMaxUs = rtUs; }
  if (ok) {
    gWheelReplies[wheelIdx]++;
  } else {
    gNoReplyCount++;
    gWheelNoReply[wheelIdx]++;
  }
}

// Bucketed percentile: walk bins bottom-up, return the upper edge of the
// bin containing the target rank. Approximate (bin-resolution), which is
// exactly the resolution the pass criterion is stated at (0.1 ms/us granularity, 2.0 ms threshold).
uint32_t percentile(double frac) {
  const uint64_t targetRank = (uint64_t)(frac * (double)gLatSamples);
  uint64_t cum = 0;
  for (int i = 0; i < kNumBins; ++i) {
    cum += gHist[i];
    if (cum >= targetRank) {
      return (i == kNumBins - 1) ? gLatMaxUs : (uint32_t)((i + 1) * kBinWidthUs);
    }
  }
  return gLatMaxUs;
}

void printHistogram() {
  Serial.println("# --- latency histogram (us bucket-upper-edge : count) ---");
  for (int i = 0; i < kNumBins; ++i) {
    if (gHist[i] == 0) { continue; }
    Serial.print("#   ");
    if (i == kNumBins - 1) {
      Serial.print(">=2000");
    } else {
      Serial.print((i + 1) * kBinWidthUs);
    }
    Serial.print(" : ");
    Serial.println(gHist[i]);
  }
}


// ----------------------------------------------------------------------------
// SECTION 2c: DROPPED-CYCLE DETECTION
// ----------------------------------------------------------------------------
// A cycle is "dropped" if servicing all three wheels took longer than the
// 2.5 ms slot -- i.e. the loop could not keep up with 400 Hz for that
// iteration, independent of whether any individual SetPosition() call
// itself timed out.

static uint32_t gTotalCycles   = 0;
static uint32_t gDroppedCycles = 0;
static uint32_t gWorstCycleUs  = 0;


// ----------------------------------------------------------------------------
// SECTION 2d: CAN BUS DIAGNOSTICS
// ----------------------------------------------------------------------------
// Same as Stage0_SingleMoteusQuery -- kBusOff/climbing error counters would
// mean electrical contention with 3 nodes talking; staying ACTIVE with 0
// errors while noReply is still ~2x cycles would rule that out and point at
// something in the shared-bus receive path instead (see the NOTES at the
// bottom of this file).

const char* canStateName(tControllerState s) {
  switch (s) {
    case kActive:  return "ACTIVE";
    case kPassive: return "PASSIVE";
    case kBusOff:  return "BUS_OFF";
  }
  return "?";
}

void printCanDiagnostics() {
  Serial.print("# CAN state: "); Serial.print(canStateName(ACAN_T4::can3.controllerState()));
  Serial.print("  rxErr="); Serial.print(ACAN_T4::can3.receiveErrorCounter());
  Serial.print("  txErr="); Serial.println(ACAN_T4::can3.transmitErrorCounter());
}


// ----------------------------------------------------------------------------
// SECTION 2e: SERIAL COMMANDS
// ----------------------------------------------------------------------------

void handleSerialCommands() {
  if (!Serial.available()) { return; }
  const String line = Serial.readStringUntil('\n');
  if (line.length() == 0) { return; }
  if (line[0] == 'h') {
    gHalted = !gHalted;
    Serial.print("# gHalted = "); Serial.println(gHalted ? "TRUE" : "FALSE");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - PHASE 0.3: THREE-MOTEUS 400 Hz LATENCY (10 min)");
  Serial.println("# run Stage0_SingleMoteusQuery on all 3 IDs first -- this");
  Serial.println("# sketch will not tell you WHICH wheel is the problem.");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  for (int i = 0; i < 3; ++i) { gWheels[i]->SetStop(); }
  Serial.println("# all stopped, starting 400 Hz loop with zero torque on all 3");
}


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  handleSerialCommands();
  if (gHalted) { return; }

  static uint32_t nextCycleUs   = micros();
  static uint32_t startUs       = micros();
  static uint32_t nextStatusUs  = micros() + kStatusEveryUs;
  static bool done = false;

  if (done) { return; }

  const uint32_t now = micros();
  if ((int32_t)(now - nextCycleUs) < 0) { return; }

  const uint32_t cycleStart = now;

  for (int i = 0; i < 3; ++i) {
    Moteus::PositionMode::Command cmd;
    cmd.position               = NaN;
    cmd.velocity               = 0.0f;
    cmd.kp_scale                = 0.0f;
    cmd.kd_scale                 = 0.0f;
    cmd.feedforward_torque       = 0.0f;
    cmd.maximum_torque           = 0.12f;
    cmd.watchdog_timeout          = 0.10f;
    cmd.ignore_position_bounds    = 1.0f;

    const uint32_t t0 = micros();
    const bool ok = gWheels[i]->SetPosition(cmd, &kTorqueFormat);
    const uint32_t rt = micros() - t0;
    recordLatency(i, rt, ok);
    if (kInterQueryDelayUs > 0 && i < 2) { delayMicroseconds(kInterQueryDelayUs); }
  }

  const uint32_t cycleDur = micros() - cycleStart;
  gTotalCycles++;
  if (cycleDur > kPeriodUs) {
    gDroppedCycles++;
    if (cycleDur > gWorstCycleUs) { gWorstCycleUs = cycleDur; }
  }

  nextCycleUs += kPeriodUs;

  const uint32_t elapsed = now - startUs;

  if ((int32_t)(now - nextStatusUs) >= 0) {
    nextStatusUs += kStatusEveryUs;
    Serial.print("# t="); Serial.print(elapsed / 1000000UL);
    Serial.print("s cycles="); Serial.print(gTotalCycles);
    Serial.print(" dropped="); Serial.print(gDroppedCycles);
    Serial.print(" worstCycle_us="); Serial.print(gWorstCycleUs);
    Serial.print(" latMax_us="); Serial.print(gLatMaxUs);
    Serial.print(" noReply="); Serial.println(gNoReplyCount);
    Serial.print("#   per-wheel replies/noReply  X="); Serial.print(gWheelReplies[0]);
    Serial.print("/"); Serial.print(gWheelNoReply[0]);
    Serial.print("  Y="); Serial.print(gWheelReplies[1]);
    Serial.print("/"); Serial.print(gWheelNoReply[1]);
    Serial.print("  Z="); Serial.print(gWheelReplies[2]);
    Serial.print("/"); Serial.println(gWheelNoReply[2]);
    printCanDiagnostics();
  }

  if (elapsed >= kDurationUs) {
    done = true;
    Serial.println();
    Serial.println("# ================ 10 MINUTES COMPLETE ================");
    printHistogram();
    Serial.print("# samples="); Serial.println((uint32_t)gLatSamples);
    Serial.print("# min_us="); Serial.println(gLatMinUs);
    Serial.print("# mean_us="); Serial.println((uint32_t)(gLatSumUs / gLatSamples));
    Serial.print("# p50_us="); Serial.println(percentile(0.50));
    Serial.print("# p99_us="); Serial.println(percentile(0.99));
    Serial.print("# p999_us="); Serial.println(percentile(0.999));
    Serial.print("# max_us="); Serial.println(gLatMaxUs);
    Serial.print("# noReply_total="); Serial.println(gNoReplyCount);
    Serial.print("# totalCycles="); Serial.println(gTotalCycles);
    Serial.print("# droppedCycles="); Serial.println(gDroppedCycles);
    Serial.print("# worstCycle_us="); Serial.println(gWorstCycleUs);
    Serial.print("# per-wheel replies/noReply  X="); Serial.print(gWheelReplies[0]);
    Serial.print("/"); Serial.print(gWheelNoReply[0]);
    Serial.print("  Y="); Serial.print(gWheelReplies[1]);
    Serial.print("/"); Serial.print(gWheelNoReply[1]);
    Serial.print("  Z="); Serial.print(gWheelReplies[2]);
    Serial.print("/"); Serial.println(gWheelNoReply[2]);
    printCanDiagnostics();

    const bool pass = (percentile(0.999) < 2000) && (gDroppedCycles == 0);
    Serial.print("# ================ RESULT: ");
    Serial.print(pass ? "PASS" : "FAIL");
    Serial.println(" ================");
  }
}


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// If this fails but Stage0_SingleMoteusQuery passed on all 3 IDs
// individually, the problem is bus contention/arbitration or CPU time
// spent servicing 3x the interrupts, not any single node's link quality --
// look at bus loading (1 Mbps arbitration + BRS data rate, payload size x
// 3 nodes x 400 Hz) before suspecting hardware.
//
// FIRST RUN, 80s: noReply was EXACTLY 2x cycles every single status line
// (not approximately -- exactly), worstCycle_us ~11.4ms (~2 full 5ms
// timeouts + one fast success), CAN state ACTIVE with 0 errors throughout
// on the Stage0_SingleMoteusQuery run that preceded it. That regularity
// argues against random bus contention and toward something structural.
//
// Working hypothesis, from reading Moteus.h's Poll(): it pulls exactly one
// frame off the CanBus's shared receive queue per call and DISCARDS it
// (does not requeue, does not hand it to another Moteus instance) if the
// frame's source doesn't match that specific object's own options_.id.
// With gWheels[0..2] sharing one canBus and polled sequentially inside
// ExecuteSingleCommand()'s blocking wait loop, a reply meant for wheel B
// arriving while wheel A is still polling can be dequeued and thrown away
// by A's own Poll() before B ever sees it -- consistent with "whichever
// wheel is serviced first in the loop tends to win, the other two starve."
// NOT independently confirmed yet -- the per-wheel counters and CAN
// diagnostics added above are the direct test: if it's the same wheel(s)
// losing every cycle, this holds; if it rotates, it's something else
// (check bus loading / ACAN_T4 RX FIFO depth next instead).
// ============================================================================
