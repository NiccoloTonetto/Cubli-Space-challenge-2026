/*
  Stage 3 -- laptop <-> XIAO over UDP.  Still NO Teensy, still no wires.
  ---------------------------------------------------------------------------
  This is the stage that decides whether WiFi is your problem at all. It
  answers each of the two directions SEPARATELY, which is the thing a silent
  telemetry window can never tell you:

    laptop -> XIAO   proven by the board's own udp_in counter, printed over
                     USB. Nothing on the network path can fake this number.
    XIAO -> laptop   proven by ECHO packets arriving at stage3_udp_echo.py.

  If udp_in climbs and no echo arrives, the fault is on the LAPTOP side
  (Windows Firewall inbound UDP for python.exe, or a VPN capturing the route).
  If udp_in stays at 0 while the script is sending 20 packets/s, your packets
  never crossed the network: that is CLIENT ISOLATION on the access point, and
  no firmware change fixes it -- use a phone hotspot or a network you control.

  Run with cube power OFF, XIAO USB only. Serial Monitor at 115200 in one
  window; `python stage3_udp_echo.py` in another. It is fine and expected to
  have both open at once.

  WHAT IT SENDS BACK:
    "ECHO:<payload>"  immediately, for every packet received  -> round-trip
    "BEAT:<n>,<ms>"   every 500 ms once it has heard from the laptop, i.e.
                      unprompted -> proves the XIAO->laptop direction even if
                      your echo request is what got lost.

  The XIAO learns the laptop's address from the first packet it receives; it
  never needs to be told. That is why the script must speak first, and why
  nothing arrives until it does. Same behaviour as the real bridge.
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------- USER CONFIG -- must match xiao_teensy_bridge.ino ----------
const char* WIFI_SSID     = "cubli1";
const char* WIFI_PASSWORD = "12345678";   // <<< SET THIS

IPAddress XIAO_IP     (172, 20, 10, 14);
IPAddress XIAO_GATEWAY(172, 20, 10,  1);
IPAddress XIAO_SUBNET (255, 255, 255, 240);

const uint16_t UDP_PORT = 4210;
// ----------------------------------------------------------------------------

WiFiUDP udp;

static IPAddress gLaptopIP;
static uint16_t  gLaptopPort = 0;
static bool      gHaveLaptop = false;

static uint32_t gIn = 0, gOut = 0;          // per-second, zeroed on each print
static uint32_t gInTotal = 0, gOutTotal = 0;
static uint32_t gLastStatusMs = 0, gLastBeatMs = 0, gBeatSeq = 0;

char buf[256];

bool connectWiFi() {
  if (Serial) { Serial.printf("\nConnecting to \"%s\" ", WIFI_SSID); }
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_STA);
  // Modem sleep OFF -- see the long note in xiao_teensy_bridge.ino. Without
  // it this stage measures the ESP32's sleep schedule (median ~75 ms, ceiling
  // at the AP's beacon interval) rather than the network, and reports loss
  // that is really just late arrival.
  WiFi.setSleep(false);
  WiFi.config(XIAO_IP, XIAO_GATEWAY, XIAO_SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    if (Serial) { Serial.print("."); }
    if (millis() - start > 20000) {
      if (Serial) {
        Serial.println("\nTIMED OUT. Go back to Stage 2 -- it explains why.");
      }
      return false;
    }
  }
  if (Serial) {
    Serial.println(" connected!");
    Serial.print("  IP:   "); Serial.println(WiFi.localIP());
    Serial.print("  RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  }
  return true;
}

void pollUdp() {
  const int size = udp.parsePacket();
  if (size <= 0) { return; }

  int len = udp.read(buf, sizeof(buf) - 1);
  if (len < 0) { len = 0; }
  buf[len] = '\0';

  gIn++; gInTotal++;
  gLaptopIP   = udp.remoteIP();
  gLaptopPort = udp.remotePort();
  gHaveLaptop = true;

  // Strip the trailing newline so the echoed token is exactly what was sent.
  while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
    buf[--len] = '\0';
  }

  char reply[280];
  const int n = snprintf(reply, sizeof(reply), "ECHO:%s", buf);
  udp.beginPacket(gLaptopIP, gLaptopPort);
  udp.write((const uint8_t*)reply, n);
  udp.endPacket();
  gOut++; gOutTotal++;
}

void maybeBeat() {
  if (!gHaveLaptop) { return; }
  const uint32_t now = millis();
  if (now - gLastBeatMs < 500) { return; }
  gLastBeatMs = now;

  char msg[64];
  const int n = snprintf(msg, sizeof(msg), "BEAT:%lu,%lu",
                         (unsigned long)gBeatSeq++, (unsigned long)now);
  udp.beginPacket(gLaptopIP, gLaptopPort);
  udp.write((const uint8_t*)msg, n);
  udp.endPacket();
  gOut++; gOutTotal++;
}

void printStatus() {
  const uint32_t now = millis();
  if (now - gLastStatusMs < 1000) { return; }
  gLastStatusMs = now;

  // Same availableForWrite() guard as the real bridge: without it, a plugged-in
  // cable with no monitor reading collapses this loop to ~0.5 Hz and the echo
  // test measures the USB stall instead of the network.
  if (Serial && Serial.availableForWrite() >= 160) {
    Serial.printf("[status] wifi=%s rssi=%ddBm laptop=%s udp_in=%lu/s "
                  "udp_out=%lu/s totals=%lu/%lu\n",
                  WiFi.status() == WL_CONNECTED ? "up" : "DOWN",
                  WiFi.RSSI(),
                  gHaveLaptop
                    ? (gLaptopIP.toString() + ":" + String(gLaptopPort)).c_str()
                    : "none yet -- the laptop has never been heard from",
                  (unsigned long)gIn, (unsigned long)gOut,
                  (unsigned long)gInTotal, (unsigned long)gOutTotal);
  }
  gIn = gOut = 0;
}

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) { delay(10); }

  if (Serial) { Serial.println("\n=== Stage 3: XIAO UDP echo ==="); }
  connectWiFi();
  udp.begin(UDP_PORT);
  if (Serial) {
    Serial.printf("UDP listening on %u. Now run stage3_udp_echo.py.\n", UDP_PORT);
    Serial.println("Watch udp_in: it is the laptop->XIAO direction, measured");
    Serial.println("by the board itself. Nothing on the network can fake it.");
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (Serial) { Serial.println("WiFi dropped -- reconnecting..."); }
    connectWiFi();
  }
  pollUdp();
  maybeBeat();
  printStatus();
}
