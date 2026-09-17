/*
  XIAO ESP32C6 - WiFi + Telemetry + Link Calibration Test
  ---------------------------------------------------------
  Purpose: Standalone test of the XIAO ESP32C6's WiFi and telemetry
  capabilities BEFORE it's wired to the Teensy 4.1. This does not touch
  the D6/D7 (TX/RX) pins that will later go to the Teensy — it only
  uses WiFi + the USB serial connection for debugging.

  What it does:
    1. Connects to your WiFi network.
    2. Runs a "calibration" pass: sends N pings over UDP and measures
       round-trip latency to a listener running on your laptop, then
       reports min/avg/max/jitter and packet loss.
    3. After calibration, continuously broadcasts telemetry packets
       (JSON) over UDP so you can confirm sustained throughput/stability.
    4. Also answers ad-hoc PING requests at any time, so you can re-run
       calibration from the laptop without resetting the board.

  Companion file: xiao_laptop_listener.py (run on your laptop first)

  ---------------------------------------------------------
  ARDUINO IDE SETUP (one-time):
    1. File > Preferences > Additional Board Manager URLs, add:
       https://espressif.github.io/arduino-esp32/package_esp32_index.json
    2. Tools > Board > Boards Manager > install "esp32" (Espressif).
    3. Tools > Board > select "XIAO_ESP32C6".
    4. Tools > USB CDC On Boot > "Enabled"
       (REQUIRED on this board — it has no USB-serial chip, so without
       this the Serial Monitor will show nothing.)
    5. Fill in WIFI_SSID / WIFI_PASSWORD and LAPTOP_IP below.
  ---------------------------------------------------------
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------- USER CONFIG ----------------
const char* WIFI_SSID     = "ES-Guest";
const char* WIFI_PASSWORD = "Welc0me@ES2026";

// Set this to the fixed/static IP of the laptop running the listener
// script. Find it with `ipconfig` (Windows) or `ifconfig`/`ip addr` (Mac/Linux).
IPAddress LAPTOP_IP(10, 11, 250, 88);

const uint16_t UDP_PORT              = 4210;
const int      CALIBRATION_PING_COUNT = 20;
const uint32_t CALIBRATION_TIMEOUT_MS = 300;   // per-ping wait
const uint32_t TELEMETRY_INTERVAL_MS  = 500;   // steady-state telemetry rate
// ----------------------------------------------

WiFiUDP udp;
uint32_t telemetrySeq = 0;
uint32_t lastTelemetryMs = 0;
char packetBuf[256];

// ---------- helpers ----------

// Human-readable WiFi status codes, since "timed out" alone doesn't say why.
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
  Serial.printf("\nConnecting to WiFi \"%s\" ", WIFI_SSID);
  WiFi.disconnect(true, true);   // clear any stale state before retrying
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  wl_status_t lastStatus = WL_IDLE_STATUS;
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    wl_status_t status = WiFi.status();
    if (status != lastStatus) {
      Serial.printf("\n  status: %s", wifiStatusToString(status));
      lastStatus = status;
    }
    if (millis() - start > 20000) {
      Serial.printf("\nWiFi connect TIMED OUT. Last status: %s\n", wifiStatusToString(WiFi.status()));
      Serial.println("If this is a guest/campus/office network, it likely needs a");
      Serial.println("captive-portal browser login and/or blocks device-to-device");
      Serial.println("traffic (client isolation) — try a home network or phone hotspot instead.");
      return false;
    }
  }
  Serial.println(" connected!");
  Serial.print("  IP address:  "); Serial.println(WiFi.localIP());
  Serial.print("  Signal RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  return true;
}

// Sends "PING:<id>" to LAPTOP_IP and waits for "PONG:<id>" back.
// Returns round-trip time in ms, or -1 on timeout.
long sendPingAndWait(uint32_t id) {
  char msg[32];
  snprintf(msg, sizeof(msg), "PING:%lu", (unsigned long)id);

  uint32_t t0 = millis();
  udp.beginPacket(LAPTOP_IP, UDP_PORT);
  udp.write((const uint8_t*)msg, strlen(msg));
  udp.endPacket();

  while (millis() - t0 < CALIBRATION_TIMEOUT_MS) {
    int size = udp.parsePacket();
    if (size > 0) {
      int len = udp.read(packetBuf, sizeof(packetBuf) - 1);
      packetBuf[len] = '\0';
      char expected[32];
      snprintf(expected, sizeof(expected), "PONG:%lu", (unsigned long)id);
      if (strncmp(packetBuf, expected, strlen(expected)) == 0) {
        return millis() - t0;
      }
      // not our packet (maybe a stray) — keep waiting within the timeout window
    }
  }
  return -1;
}

void runCalibration() {
  Serial.println("\n--- Starting link calibration ---");
  Serial.printf("Target listener: %s:%u\n", LAPTOP_IP.toString().c_str(), UDP_PORT);
  Serial.println("(Make sure xiao_laptop_listener.py is running on the laptop.)");

  long rtts[CALIBRATION_PING_COUNT];
  int received = 0;
  long sum = 0, minRtt = LONG_MAX, maxRtt = LONG_MIN;

  for (int i = 0; i < CALIBRATION_PING_COUNT; i++) {
    long rtt = sendPingAndWait(i);
    if (rtt >= 0) {
      rtts[received++] = rtt;
      sum += rtt;
      if (rtt < minRtt) minRtt = rtt;
      if (rtt > maxRtt) maxRtt = rtt;
      Serial.printf("  ping %2d: %ld ms\n", i, rtt);
    } else {
      Serial.printf("  ping %2d: timeout\n", i);
    }
    delay(50);
  }

  Serial.println("--- Calibration results ---");
  if (received == 0) {
    Serial.println("  No responses received. Check that:");
    Serial.println("   - xiao_laptop_listener.py is running");
    Serial.println("   - LAPTOP_IP in this sketch matches the laptop's actual IP");
    Serial.println("   - Both devices are on the same WiFi network/subnet");
    Serial.println("   - No firewall is blocking UDP port 4210");
  } else {
    long avg = sum / received;
    // simple jitter estimate: average deviation from mean
    long jitterSum = 0;
    for (int i = 0; i < received; i++) jitterSum += abs(rtts[i] - avg);
    long jitter = jitterSum / received;

    int lossPct = 100 - (received * 100 / CALIBRATION_PING_COUNT);
    Serial.printf("  Packets: %d/%d received (%d%% loss)\n", received, CALIBRATION_PING_COUNT, lossPct);
    Serial.printf("  RTT min/avg/max: %ld / %ld / %ld ms\n", minRtt, avg, maxRtt);
    Serial.printf("  Jitter (avg deviation): %ld ms\n", jitter);
  }
  Serial.println("----------------------------\n");
}

void sendTelemetry() {
  snprintf(packetBuf, sizeof(packetBuf),
    "{\"type\":\"telemetry\",\"seq\":%lu,\"uptime_ms\":%lu,\"rssi_dbm\":%d,\"free_heap\":%lu,\"ip\":\"%s\"}",
    (unsigned long)telemetrySeq,
    (unsigned long)millis(),
    WiFi.RSSI(),
    (unsigned long)ESP.getFreeHeap(),
    WiFi.localIP().toString().c_str()
  );

  udp.beginPacket(LAPTOP_IP, UDP_PORT);
  udp.write((const uint8_t*)packetBuf, strlen(packetBuf));
  udp.endPacket();

  telemetrySeq++;
}

// Handles any inbound PING that arrives outside the calibration routine,
// so you can trigger fresh calibration checks from the laptop side later
// without resetting the board.
void handleIncomingPackets() {
  int size = udp.parsePacket();
  if (size <= 0) return;

  int len = udp.read(packetBuf, sizeof(packetBuf) - 1);
  packetBuf[len] = '\0';

  if (strncmp(packetBuf, "PING:", 5) == 0) {
    char reply[32];
    snprintf(reply, sizeof(reply), "PONG:%s", packetBuf + 5);
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((const uint8_t*)reply, strlen(reply));
    udp.endPacket();
  }
}

// ---------- Arduino entry points ----------

void setup() {
  Serial.begin(115200);
  uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) { delay(10); }

  Serial.println("\n=== XIAO ESP32C6 WiFi/Telemetry Test ===");

  if (!connectWiFi()) {
    Serial.println("Halting — fix WiFi credentials/signal and reset the board.");
    while (true) { delay(1000); }
  }

  udp.begin(UDP_PORT);
  Serial.printf("UDP listening on port %u\n", UDP_PORT);

  runCalibration();

  Serial.println("Entering steady-state telemetry broadcast...");
  Serial.println("(Watch xiao_laptop_listener.py output on your laptop.)\n");
}

void loop() {
  // Reconnect handling — keep the test resilient if WiFi drops
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi dropped — reconnecting...");
    connectWiFi();
  }

  handleIncomingPackets();

  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }
}

/*
  ---------------------------------------------------------
  NOTE FOR THE FINAL SYSTEM (Teensy 4.1 link):
  When you're ready to add the Teensy connection, D6 (GPIO16/TX) and
  D7 (GPIO17/RX) are the hardware UART0 pins on this board. Example
  init (not used in this WiFi-only test):

    #define TEENSY_RX_PIN D7
    #define TEENSY_TX_PIN D6
    Serial1.begin(115200, SERIAL_8N1, TEENSY_RX_PIN, TEENSY_TX_PIN);

  Wire it cross-connected: XIAO D6(TX) -> Teensy RX, XIAO D7(RX) <- Teensy TX,
  and make sure both boards share a common GND.
  ---------------------------------------------------------
*/
