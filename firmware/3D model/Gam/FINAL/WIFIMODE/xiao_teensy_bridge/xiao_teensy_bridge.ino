/*
  XIAO ESP32C6 — Teensy <-> WiFi/UDP Bridge (+ standalone WiFi test mode)
  ---------------------------------------------------------------------
  Companion to firmware/2D model/panel-bringup/Stage4_FullLaw_WiFi/
  Stage4_FullLaw_WiFi.ino. Relays that sketch's existing Serial1
  telemetry/command protocol over WiFi/UDP to a laptop, so the panel can
  run untethered from USB.

  This does NOT modify or replace xiao_esp32c6_wifi_test1.ino -- that
  standalone WiFi/UDP smoke test (+ its xiao_laptop_listener1.py
  companion) is untouched and still works on its own. This is a new,
  separate sketch.

  Flash once -- it bundles two modes, switched at RUNTIME (no reflash) by
  a UDP command from the laptop, so you don't need to keep re-flashing to
  change what the board is doing:

    x0  -> WIFI_TEST mode     -- standalone ad-hoc PING/PONG + synthetic
                                  telemetry, no Teensy required. Useful for
                                  bench-checking the WiFi link by itself.
    x1  -> TEENSY_BRIDGE mode -- real relay: Serial1 <-> UDP.  [boot default]

  'x' packets are handled locally and NEVER relayed to the Teensy. Every
  other packet (t/h/a/g/o -- Stage4_FullLaw_WiFi.ino's command grammar) is
  relayed to Serial1 verbatim, and every line the Teensy sends back on
  Serial1 is forwarded to the laptop as one UDP packet -- regardless of
  which mode is active, so the bridge itself is always live. loop() is
  fully non-blocking (no delay() in the hot path) so this UDP command
  check runs every single iteration and mode-switch commands are never
  missed, no matter what else the board is doing.

  Wiring to the Teensy (see Stage4_FullLaw_WiFi.ino header):
    XIAO D7 (RX) <- Teensy Serial1 TX1 (pin 1)
    XIAO D6 (TX) -> Teensy Serial1 RX1 (pin 0)
    Common GND.

  ---------------------------------------------------------
  ARDUINO IDE SETUP (one-time) -- same as xiao_esp32c6_wifi_test1.ino:
    1. File > Preferences > Additional Board Manager URLs, add:
       https://espressif.github.io/arduino-esp32/package_esp32_index.json
    2. Tools > Board > Boards Manager > install "esp32" (Espressif).
    3. Tools > Board > select "XIAO_ESP32C6".
    4. Tools > USB CDC On Boot > "Enabled" (boot logs only -- not part of
       the operational bridge, which runs entirely over Serial1 + WiFi).
    5. Fill in WIFI_SSID / WIFI_PASSWORD / XIAO's static IP block below.
  ---------------------------------------------------------
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------- USER CONFIG ----------------
// Phone hotspot. ES-Guest was abandoned: it runs CLIENT ISOLATION, so the
// laptop and the XIAO could both reach the internet but never each other --
// proven with link_check.py, which showed the XIAO's own udp_in counter
// sitting at 0 while the laptop sent 10 packets/s.
//
// Keep the SSID PLAIN ASCII. The default iPhone name ("Pablo's Iphone")
// contains U+2019, a curly apostrophe that is 3 UTF-8 bytes -- typing a
// normal ' here would not match the real SSID and the join would fail
// silently. Renaming the phone is the fix; do not go back to a fancy name.
const char* WIFI_SSID     = "cubli1";
const char* WIFI_PASSWORD = "12345678";   // <<< SET THIS

// Static IP for the XIAO itself, so the PC script always has one fixed
// address to send to (no discovery handshake needed). Pick a free address
// on your actual subnet -- these are placeholders matching the subnet
// already used by xiao_esp32c6_wifi_test1.ino's LAPTOP_IP.
// iPhone hotspot is a /28: network 172.20.10.0, usable .1-.14, broadcast
// .15, gateway .1. The laptop takes .4 from DHCP, so .14 sits at the top of
// the range well clear of it. THE MASK IS 240, NOT 0 -- only 14 addresses.
IPAddress XIAO_IP     (172, 20, 10, 14);
IPAddress XIAO_GATEWAY(172, 20, 10,  1);
IPAddress XIAO_SUBNET (255, 255, 255, 240);

const uint16_t UDP_PORT             = 4210;
const uint32_t TEENSY_LINK_BAUD     = 1000000;   // matches Serial1 on the Teensy
const uint32_t TELEMETRY_INTERVAL_MS = 500;      // WIFI_TEST synthetic telemetry rate
// ----------------------------------------------

WiFiUDP udp;

enum class XiaoMode : uint8_t { WIFI_TEST = 0, TEENSY_BRIDGE = 1 };
static XiaoMode gMode = XiaoMode::TEENSY_BRIDGE;   // boot default -- see header

static IPAddress gLaptopIP;
static uint16_t  gLaptopPort = 0;
static bool      gHaveLaptop = false;

static uint32_t gLastTelemetryMs = 0;
static uint32_t gTelemetrySeq    = 0;

char udpBuf[256];      // inbound UDP payload
char serialBuf[256];   // Serial1 line accumulator
uint8_t serialLen = 0;

// Set to 0 if you must leave the USB cable attached during a telemetry run
// with nothing reading the port -- see the long note in printLinkStatus().
// Leaving it at 1 is fine when the cable is unplugged, or when a serial
// monitor is actually open and draining the buffer.
#define ENABLE_LINK_STATUS 1

// ---------- TEMP DEBUG: link diagnostics ----------
// Prints a one-line status summary every second over USB so you can watch,
// in real time, exactly where the pipeline is breaking: no Serial1 lines
// at all -> wiring/Teensy-side problem; Serial1 lines seen but no UDP-out
// -> laptop never reached the XIAO (firewall/client isolation); both
// counts moving but nothing shows up on the PC -> look there instead.
// Safe to delete once the link is confirmed working end-to-end.
static uint32_t gSerial1LinesSeen = 0;
static uint32_t gUdpPacketsIn     = 0;
static uint32_t gUdpPacketsOut    = 0;
static uint32_t gLastStatusMs     = 0;

void printLinkStatus() {
  const uint32_t now = millis();
  if (now - gLastStatusMs < 1000) { return; }
  gLastStatusMs = now;

  // Native USB CDC (this chip's `Serial`) can BLOCK on print/printf when no
  // host has the port open -- unlike Teensy's USB serial, which never
  // blocks regardless of connection state.
  //
  // `if (Serial)` ALONE IS NOT ENOUGH, and this was measured, not guessed:
  // with the cable plugged into a laptop but NO application reading the
  // port, Serial still evaluates true and printf() blocks ~2 s waiting for
  // the TX buffer to drain. Because the 1 s gate above has always elapsed by
  // then, it re-blocks on the very next iteration. Measured result:
  //
  //     loop() collapses from kHz to 0.49 iterations/s
  //
  // and since pollUdpIn() and pollSerial1Out() each handle exactly ONE item
  // per iteration, the bridge then relays 0.5 lines/s -- against the 250/s
  // the corner build emits and 500/s from the edge build. Telemetry looks
  // dead for reasons that appear to be nothing to do with this debug aid.
  //
  // availableForWrite() makes it skip itself instead of stalling the relay.
  // ENABLE_LINK_STATUS is the hard escape hatch if a core revision reports
  // free space that does not reflect a stalled host.
  //
  // OPERATIONALLY: unplug the XIAO's USB for real telemetry runs and power
  // it from the 5 V rail. With no cable there is no CDC to stall on.
  if (ENABLE_LINK_STATUS && Serial && Serial.availableForWrite() >= 160) {
    Serial.printf("[status] wifi=%s rssi=%ddBm laptop=%s serial1_lines=%lu udp_in=%lu udp_out=%lu mode=%s\n",
                  WiFi.status() == WL_CONNECTED ? "up" : "DOWN",
                  WiFi.RSSI(),
                  gHaveLaptop ? (gLaptopIP.toString() + ":" + String(gLaptopPort)).c_str() : "none yet",
                  (unsigned long)gSerial1LinesSeen,
                  (unsigned long)gUdpPacketsIn,
                  (unsigned long)gUdpPacketsOut,
                  gMode == XiaoMode::TEENSY_BRIDGE ? "TEENSY_BRIDGE" : "WIFI_TEST");
  }

  gSerial1LinesSeen = 0;
  gUdpPacketsIn = 0;
  gUdpPacketsOut = 0;
}

// ---------- WiFi connect (same pattern as xiao_esp32c6_wifi_test1.ino) ----------

const char* wifiStatusToString(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL (network not found / out of range)";
    case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED (likely wrong password, or WPA2-Enterprise/captive portal network)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    default:                 return "UNKNOWN";
  }
}

bool connectWiFi() {
  // Same non-blocking-CDC concern as printLinkStatus() above -- every
  // Serial call in here is guarded so a disconnected/absent USB host can
  // never stall a (re)connect attempt, which loop() can trigger at any
  // time if WiFi drops mid-run, not just at boot.
  if (Serial) { Serial.printf("\nConnecting to WiFi \"%s\" ", WIFI_SSID); }
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_STA);

  // WIFI MODEM SLEEP OFF. This is not an optimisation, it is required.
  //
  // The esp32 core defaults a station to WIFI_PS_MIN_MODEM: the radio parks
  // between the AP's beacons and only wakes to collect buffered downlink
  // traffic. Everything the laptop sends therefore waits for the next
  // beacon. Measured on this hardware at -32 dBm (link-bringup Stage 3):
  //
  //     min 3.9 ms   median 75.5 ms   p95/max 93.5 ms
  //
  // The 3.9 ms is what the RF path actually costs; the rest is sleep. The
  // ~100 ms ceiling is the iPhone hotspot's beacon interval, which is what
  // gives the distribution its shape. Consequences for this bridge:
  //   - every command (including a0) is delayed by up to a beacon interval;
  //   - the PC keepalive arrives in bursts, so the Teensy's 300 ms link
  //     watchdog can time out and auto-disarm mid-balance for no real
  //     reason -- a phantom fault that looks exactly like a link failure;
  //   - packets buffered for a sleeping station get dropped when the AP's
  //     queue for it fills, at telemetry rates.
  //
  // Cost: the radio stays awake, ~100 mA continuous instead of a lower
  // average. Electrical-Design-Guide.md §2.2 already budgets the ESP32-C6 at
  // ~100 mA typical / 300 mA peak, so this is inside the existing budget.
  WiFi.setSleep(false);

  WiFi.config(XIAO_IP, XIAO_GATEWAY, XIAO_SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  wl_status_t lastStatus = WL_IDLE_STATUS;
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    if (Serial) { Serial.print("."); }
    wl_status_t status = WiFi.status();
    if (status != lastStatus) {
      if (Serial) { Serial.printf("\n  status: %s", wifiStatusToString(status)); }
      lastStatus = status;
    }
    if (millis() - start > 20000) {
      if (Serial) {
        Serial.printf("\nWiFi connect TIMED OUT. Last status: %s\n", wifiStatusToString(WiFi.status()));
        Serial.println("If this is a guest/campus/office network, it likely needs a");
        Serial.println("captive-portal browser login and/or blocks device-to-device");
        Serial.println("traffic (client isolation), which would break this bridge too --");
        Serial.println("try a home network or phone hotspot instead.");
      }
      return false;
    }
  }
  if (Serial) {
    Serial.println(" connected!");
    Serial.print("  IP address:  "); Serial.println(WiFi.localIP());
    Serial.print("  Signal RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  }
  return true;
}

// ---------- UDP in: mode-switch, Teensy-command relay, WIFI_TEST ping ----------

void pollUdpIn() {
  const int size = udp.parsePacket();
  if (size <= 0) { return; }

  int len = udp.read(udpBuf, sizeof(udpBuf) - 1);
  if (len < 0) { len = 0; }
  udpBuf[len] = '\0';

  gUdpPacketsIn++;
  gLaptopIP    = udp.remoteIP();
  gLaptopPort  = udp.remotePort();
  gHaveLaptop  = true;

  if (udpBuf[0] == 'x') {
    // x0 -> WIFI_TEST, x1 -> TEENSY_BRIDGE, matching this file's header, the
    // boot message below, and both telemetry scripts' docs. This condition
    // used to be reversed, so every documented x0/x1 did the opposite of
    // what it said -- notably making PING/PONG unreachable via the
    // documented command, since PING is only answered in WIFI_TEST.
    gMode = (udpBuf[1] != '0') ? XiaoMode::TEENSY_BRIDGE : XiaoMode::WIFI_TEST;
    return;   // XIAO-local mode switch -- never relayed to the Teensy
  }

  // Relay everything else to the Teensy verbatim (t/h/a/g/o), regardless
  // of current mode -- the bridge itself is always live.
  Serial1.write((const uint8_t*)udpBuf, len);
  if (len == 0 || udpBuf[len - 1] != '\n') { Serial1.write('\n'); }

  // WIFI_TEST extra: also answer ad-hoc pings directly, for a Teensy-free
  // bench check of the WiFi link (harmless no-op relay above either way).
  if (gMode == XiaoMode::WIFI_TEST && strncmp(udpBuf, "PING:", 5) == 0) {
    char reply[32];
    snprintf(reply, sizeof(reply), "PONG:%s", udpBuf + 5);
    udp.beginPacket(gLaptopIP, gLaptopPort);
    udp.write((const uint8_t*)reply, strlen(reply));
    udp.endPacket();
  }
}

// ---------- Serial1 (Teensy) -> UDP (laptop): always active ----------

void pollSerial1Out() {
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (serialLen == 0) { continue; }
      gSerial1LinesSeen++;
      if (gHaveLaptop) {
        udp.beginPacket(gLaptopIP, gLaptopPort);
        udp.write((const uint8_t*)serialBuf, serialLen);
        udp.endPacket();
        gUdpPacketsOut++;
      }
      serialLen = 0;
      return;   // one line per call -- bounded work per loop() iteration
    }
    if (serialLen < sizeof(serialBuf) - 1) { serialBuf[serialLen++] = c; }
  }
}

// ---------- WIFI_TEST: synthetic telemetry, same shape as xiao_esp32c6_wifi_test1.ino ----------

void maybeSendSyntheticTelemetry() {
  if (!gHaveLaptop) { return; }
  const uint32_t now = millis();
  if (now - gLastTelemetryMs < TELEMETRY_INTERVAL_MS) { return; }
  gLastTelemetryMs = now;

  char msg[192];
  snprintf(msg, sizeof(msg),
    "{\"type\":\"telemetry\",\"seq\":%lu,\"uptime_ms\":%lu,\"rssi_dbm\":%d,\"free_heap\":%lu,\"ip\":\"%s\"}",
    (unsigned long)gTelemetrySeq++,
    (unsigned long)now,
    WiFi.RSSI(),
    (unsigned long)ESP.getFreeHeap(),
    WiFi.localIP().toString().c_str()
  );

  udp.beginPacket(gLaptopIP, gLaptopPort);
  udp.write((const uint8_t*)msg, strlen(msg));
  udp.endPacket();
}

// ---------- Arduino entry points ----------

void setup() {
  Serial.begin(115200);
  uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) { delay(10); }

  Serial1.begin(TEENSY_LINK_BAUD, SERIAL_8N1, D7, D6);   // D7=RX, D6=TX

  if (Serial) { Serial.println("\n=== XIAO ESP32C6 Teensy Bridge ==="); }
  connectWiFi();

  udp.begin(UDP_PORT);
  if (Serial) {
    Serial.printf("UDP listening on port %u (mode: %s)\n", UDP_PORT,
                  gMode == XiaoMode::TEENSY_BRIDGE ? "TEENSY_BRIDGE" : "WIFI_TEST");
    Serial.println("Send \"x0\" (WIFI_TEST) / \"x1\" (TEENSY_BRIDGE) via UDP to switch modes live.");
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (Serial) { Serial.println("WiFi dropped -- reconnecting..."); }
    connectWiFi();
  }

  pollUdpIn();
  pollSerial1Out();
  printLinkStatus();

  if (gMode == XiaoMode::WIFI_TEST) {
    maybeSendSyntheticTelemetry();
  }
}
