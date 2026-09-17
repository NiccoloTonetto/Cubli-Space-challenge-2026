/*
  Stage 2 -- XIAO joins the network.  No UDP, no Teensy, no wires needed.
  ---------------------------------------------------------------------------
  Isolates exactly one question: can this board associate with your access
  point and get the static address you assigned it?

  Stage 3 cannot pass until this does, and "no telemetry" caused by a failed
  join looks identical from the laptop to "no telemetry" caused by a firewall.
  Separate them here, with the board's own report as evidence.

  Run with cube power OFF, XIAO USB only, Serial Monitor at 115200.

  WHAT IT DOES BEYOND A PLAIN JOIN:
    * on failure it SCANS and lists every SSID it can hear, with RSSI and
      encryption, so you can see whether your network is even in range and
      whether its name is spelled the way you typed it;
    * it flags non-ASCII characters in visible SSIDs. The default iPhone
      hotspot name contains U+2019, a curly apostrophe -- typing a normal '
      in WIFI_SSID silently never matches. This has cost this project real
      hours; rename the phone to plain ASCII (this repo uses "cubli1");
    * it checks the static IP against the gateway and mask and refuses to
      pretend a mismatched subnet is fine;
    * it warns about 5 GHz-only networks: the ESP32-C6 radio here is used in
      2.4 GHz mode, and an AP left on 5 GHz-only will never appear in the
      scan at all.

  KEEP THESE FOUR VALUES IDENTICAL in every sketch of this ladder and in
  ../../../xiao_teensy_bridge/xiao_teensy_bridge.ino. A mismatch between the
  test sketch that passed and the bridge that "doesn't work" is a trap.
*/

#include <WiFi.h>

// ---------------- USER CONFIG -- must match xiao_teensy_bridge.ino ----------
const char* WIFI_SSID     = "cubli1";
const char* WIFI_PASSWORD = "12345678";   // <<< SET THIS

// iPhone hotspot is a /28: network 172.20.10.0, usable .1-.14, gateway .1,
// broadcast .15. THE MASK IS 240, NOT 0 -- only 14 addresses exist.
IPAddress XIAO_IP     (172, 20, 10, 14);
IPAddress XIAO_GATEWAY(172, 20, 10,  1);
IPAddress XIAO_SUBNET (255, 255, 255, 240);
// ----------------------------------------------------------------------------

static uint32_t gLastPrintMs = 0;

const char* statusName(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL (name not found / out of range / 5 GHz-only AP)";
    case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED (wrong password, or WPA2-Enterprise/captive portal)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    default:                 return "UNKNOWN";
  }
}

// A static address only works if it is inside the gateway's subnet. Getting
// this wrong gives a board that reports "connected" and is unreachable --
// the most confusing failure in the whole ladder, because every LED and every
// status line looks healthy.
void checkSubnetSanity() {
  const uint32_t ip = (uint32_t)XIAO_IP;
  const uint32_t gw = (uint32_t)XIAO_GATEWAY;
  const uint32_t mask = (uint32_t)XIAO_SUBNET;

  Serial.println("  --- static address sanity ---");
  if ((ip & mask) != (gw & mask)) {
    Serial.println("  !! XIAO_IP and XIAO_GATEWAY are NOT in the same subnet.");
    Serial.println("     The board will associate and then be unreachable.");
    Serial.println("     Fix XIAO_IP/XIAO_GATEWAY/XIAO_SUBNET before going on.");
  } else {
    Serial.println("  ok: XIAO_IP is inside the gateway's subnet");
  }
  if (ip == gw) {
    Serial.println("  !! XIAO_IP equals the GATEWAY address. Pick another.");
  }
  // Host part all-ones = the broadcast address; all-zeros = the network
  // address. Neither is assignable, and on a /28 hotspot .15 and .0 are much
  // easier to hit by accident than on a /24.
  //
  // (IPAddress casts to a uint32_t in network byte order, so these words are
  // byte-reversed on this little-endian chip. That is a uniform permutation
  // of the bits, applied to the address and the mask alike, so the masking
  // and the comparisons below are still exactly right.)
  if ((ip | ~mask) == ip) { Serial.println("  !! XIAO_IP is the BROADCAST address of this subnet."); }
  if ((ip & ~mask) == 0)  { Serial.println("  !! XIAO_IP is the NETWORK address of this subnet."); }
  Serial.println();
}

void scanAndReport() {
  Serial.println("\n  --- scanning for visible networks (2.4 GHz) ---");
  const int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("  nothing heard at all. Antenna, or the AP is off / 5 GHz-only.");
    return;
  }
  bool sawTarget = false;
  for (int i = 0; i < n; i++) {
    const String ssid = WiFi.SSID(i);
    bool nonAscii = false;
    for (unsigned k = 0; k < ssid.length(); k++) {
      if ((uint8_t)ssid[k] < 32 || (uint8_t)ssid[k] > 126) { nonAscii = true; }
    }
    Serial.printf("   %2d) %-32s %4d dBm ch%-3d %s%s\n", i + 1, ssid.c_str(),
                  WiFi.RSSI(i), WiFi.channel(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OPEN " : "enc  ",
                  nonAscii ? "  <-- CONTAINS NON-ASCII CHARACTERS" : "");
    if (ssid == String(WIFI_SSID)) { sawTarget = true; }
  }
  Serial.println();
  if (sawTarget) {
    Serial.printf("  \"%s\" IS visible -- so the name is right and the failure is\n", WIFI_SSID);
    Serial.println("  the PASSWORD, or the network needs a captive-portal login,");
    Serial.println("  or it is WPA2-Enterprise (which this board cannot join).");
  } else {
    Serial.printf("  \"%s\" is NOT in that list. Compare it character by\n", WIFI_SSID);
    Serial.println("  character against the names above -- especially any line");
    Serial.println("  flagged NON-ASCII. Also check the AP is not 5 GHz-only.");
  }
  WiFi.scanDelete();
}

bool connectWiFi() {
  Serial.printf("\nConnecting to \"%s\" ", WIFI_SSID);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_STA);
  // Modem sleep OFF, to match the bridge exactly -- see the long note in
  // xiao_teensy_bridge.ino. Keep every sketch in this ladder identical to the
  // bridge in this respect, or a stage can pass under settings the real
  // firmware does not use.
  WiFi.setSleep(false);
  WiFi.config(XIAO_IP, XIAO_GATEWAY, XIAO_SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t start = millis();
  wl_status_t last = WL_IDLE_STATUS;
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    const wl_status_t s = WiFi.status();
    if (s != last) { Serial.printf("\n  status: %s\n", statusName(s)); last = s; }
    if (millis() - start > 20000) {
      Serial.printf("\nTIMED OUT after 20 s. Last status: %s\n",
                    statusName(WiFi.status()));
      scanAndReport();
      return false;
    }
  }

  Serial.println(" connected!");
  Serial.print("  IP:      "); Serial.println(WiFi.localIP());
  Serial.print("  gateway: "); Serial.println(WiFi.gatewayIP());
  Serial.print("  mask:    "); Serial.println(WiFi.subnetMask());
  Serial.print("  RSSI:    "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  Serial.print("  MAC:     "); Serial.println(WiFi.macAddress());

  if (WiFi.localIP() != XIAO_IP) {
    Serial.println("  !! The board did NOT get the static IP you asked for.");
    Serial.println("     WiFi.config() was rejected or overridden by DHCP.");
    Serial.println("     Use the address printed above, or pick a free one.");
  }
  if (WiFi.RSSI() < -75) {
    Serial.println("  !! RSSI is weak. Expect packet loss at telemetry rates.");
    Serial.println("     Move the AP closer before blaming the bridge later.");
  }
  Serial.println("\n  NEXT: from the laptop, on the SAME network, run");
  Serial.print  ("     ping "); Serial.println(WiFi.localIP());
  Serial.println("  A reply here is good news but NOT proof -- some APs pass");
  Serial.println("  ICMP and still block UDP between clients. Stage 3 decides.");
  return true;
}

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) { delay(10); }

  Serial.println();
  Serial.println("=== Stage 2: XIAO WiFi join ===");
  checkSubnetSanity();
  connectWiFi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n!! link DROPPED -- reconnecting");
    connectWiFi();
    return;
  }
  const uint32_t now = millis();
  if (now - gLastPrintMs < 1000) { return; }
  gLastPrintMs = now;

  // A join that holds for 60 s is a join. One that flaps every few seconds
  // will show up here as repeated "link DROPPED" and would otherwise look
  // like random telemetry dropouts three stages later.
  Serial.printf("[wifi] up ip=%s rssi=%ddBm uptime=%lus\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI(),
                (unsigned long)(now / 1000));
}
