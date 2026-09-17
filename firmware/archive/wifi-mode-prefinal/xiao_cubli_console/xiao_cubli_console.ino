/*
  XIAO ESP32C6 -- Cubli console host: HTTP + WebSocket + UDP bridge + OTA
  ======================================================================

  A superset of ../xiao_teensy_bridge/xiao_teensy_bridge.ino. That sketch is
  LEFT IN PLACE and still works; this one keeps its entire UDP behaviour
  byte-for-byte (same port, same x0/x1 mode switch, same verbatim relay) and
  adds three things on top:

    1. an HTTP server on :80 serving the single-page console from PROGMEM;
    2. a WebSocket server on :81 for live telemetry, commands and status;
    3. a wireless flashing pipeline that streams an Intel .hex to the Teensy.

  Keeping UDP means telemetry_python_wifi*.py, terminal_wifi.py, link_check.py
  and dashboard/app.py all keep working unchanged, and can even run at the
  same time as a browser -- the two paths are independent.

  ---------------------------------------------------------------------------
  THE DEADMAN, WHICH IS THE WHOLE SAFETY STORY
  ---------------------------------------------------------------------------
  CornerBalance_WiFi.ino:170 sets kLinkTimeoutMs = 300: an armed cube disarms
  itself if no line arrives on Serial1 for 300 ms. Under the Python dashboard
  the laptop's heartbeat thread was what kept that timer fed.

  With a browser talking straight to this board, THIS board has to feed it --
  and the naive version is a safety regression. If the XIAO sent 'k'
  unconditionally, closing the browser tab would no longer disarm the cube:
  the keepalive would sail on with nobody watching.

  So the chain is deliberately end-to-end:

      browser ping every 250 ms
        -> XIAO emits 'k' every 100 ms ONLY while a ping is < 500 ms old
          -> Teensy disarms 300 ms after 'k' stops

  Close the tab, drop WiFi, walk out of range, or crash the browser, and the
  cube is disarmed inside ~800 ms. Do not "fix" the gate by making the
  keepalive unconditional.

  ---------------------------------------------------------------------------
  WIRELESS FLASHING
  ---------------------------------------------------------------------------
  Two phases, matching what the hardware can actually promise:

    POST /ota/upload   the browser posts the .hex; this sketch splits it into
                       records and streams them to the Teensy, which checksums
                       each one and writes it into its FlasherX staging buffer
                       (upper flash). Nothing is committed.
    POST /ota/commit   the Teensy validates the staged image and calls
                       flash_move(), which reboots into the new firmware.
    POST /ota/abort    the Teensy frees the buffer and reboots unchanged.

  FLOW CONTROL IS NOT OPTIONAL. A 4 KB flash sector erase on the Teensy stalls
  for tens of milliseconds, and at 1 Mbaud that is several KB of hex arriving
  with nowhere to go. So this sketch sends at most kOtaChunkBytes between
  acknowledgements: it emits a "u?" probe, then blocks until the Teensy answers
  "#OTA ACK". Because that block happens inside the WebServer's upload
  callback, TCP's own window does the rest -- the browser is throttled to
  exactly the rate the Teensy can absorb, with no application-level rate
  guessing anywhere.

  The Teensy's Serial1 RX buffer is enlarged to 8 KB on its side (see
  CubliOta_Teensy.ino), which is 4x the chunk size, so even a worst-case erase
  stall mid-chunk cannot overflow it.

  ---------------------------------------------------------------------------
  ARDUINO IDE SETUP
  ---------------------------------------------------------------------------
    Board:    XIAO_ESP32C6   (esp32 core by Espressif, 3.x)
    Library:  "WebSockets" by Markus Sattler (Links2004), >= 2.4
              Sketch > Include Library > Manage Libraries > "WebSockets"
    USB CDC On Boot: Enabled  (boot logs only -- see the note in status())

  web_index.h is GENERATED. After editing ../console/index.html run:
      cd ../console && python build_web.py

  Wiring is unchanged from the old bridge:
      XIAO D7 (RX) <- Teensy Serial1 TX1 (pin 1)
      XIAO D6 (TX) -> Teensy Serial1 RX1 (pin 0)
      Common GND.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#include "web_index.h"

// ---------------- USER CONFIG ----------------
// Identical to xiao_teensy_bridge.ino -- see its header for why the SSID must
// stay plain ASCII and why ES-Guest (client isolation) was abandoned.
const char* WIFI_SSID     = "cubli1";
const char* WIFI_PASSWORD = "12345678";   // <<< SET THIS

// iPhone hotspot is a /28: usable .1-.14, gateway .1, MASK IS 240.
IPAddress XIAO_IP     (172, 20, 10, 14);
IPAddress XIAO_GATEWAY(172, 20, 10,  1);
IPAddress XIAO_SUBNET (255, 255, 255, 240);

const uint16_t UDP_PORT  = 4210;
const uint16_t HTTP_PORT = 80;
const uint16_t WS_PORT   = 81;    // the sync HTTP server owns :80 and cannot
                                  // share it, so the socket is a second listener

// Serial1 to the Teensy. MUST match kLinkBaud in the Teensy sketch.
//
// This is 1 Mbaud, not the 115200 a flashing pipeline would normally use, and
// that is forced by telemetry rather than by OTA: a 21-field corner line is
// ~170 B at 250 Hz = ~340 kbit/s, which does not fit in 115200 (see the
// TELEMETRY RATE note in CornerBalance_WiFi.ino:33). Dropping to 115200 would
// make Serial1 writes block inside the Teensy's 2 ms control cycle. It also
// happens to make OTA ~8x faster: a ~570 KB hex takes ~6 s instead of ~50 s.
// If you ever do need 115200, change it HERE and in the Teensy sketch
// together -- a mismatch is silent, and looks exactly like a dead link.
const uint32_t TEENSY_LINK_BAUD = 1000000;
// ----------------------------------------------

// ---- timings ----
const uint32_t kKeepaliveMs    = 100;   // 'k' to the Teensy while a client is live
const uint32_t kPingStaleMs    = 500;   // browser ping older than this = no client
const uint32_t kWsTelemetryMs  = 40;    // 25 Hz to the browser; the Teensy sends 250
const uint32_t kWsStatusMs     = 250;
const uint32_t kUartStaleMs    = 400;   // no Teensy line for this long = UART down

// ---- OTA ----
const uint32_t kOtaChunkBytes  = 2048;  // bytes between "u?" acknowledgements
const uint32_t kOtaAckTimeout  = 8000;  // a sector erase can be slow; be generous
const uint32_t kOtaReadyTimeout= 3000;

// Relay at most this many Teensy lines per loop() iteration. The old bridge
// did exactly one, which was fine when loop() did nothing else; with an HTTP
// server and a WebSocket in the same loop that is not enough headroom to keep
// up with a 250 Hz corner stream. Still bounded, so one busy stream can never
// starve the UDP/HTTP/WS polls below it.
const uint8_t kMaxLinesPerLoop = 8;

WiFiUDP udp;
WebServer server(HTTP_PORT);
WebSocketsServer webSocket(WS_PORT);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

enum class XiaoMode : uint8_t { WIFI_TEST = 0, TEENSY_BRIDGE = 1 };
static XiaoMode gMode = XiaoMode::TEENSY_BRIDGE;

static IPAddress gLaptopIP;
static uint16_t  gLaptopPort = 0;
static bool      gHaveLaptop = false;

static char   gSerialBuf[512];   // one Teensy line
static size_t gSerialLen = 0;

static char     gLastTlm[512];   // newest telemetry line, awaiting decimation
static bool     gTlmPending = false;
static uint32_t gLastTeensyMs = 0;
static uint32_t gLineCount = 0;      // lines since the last rate calculation
static uint32_t gRateWindowMs = 0;
static float    gRateHz = 0.0f;

static uint32_t gLastPingMs = 0;     // browser deadman
static uint32_t gLastKeepaliveMs = 0;
static uint32_t gLastWsTlmMs = 0;
static uint32_t gLastWsStatusMs = 0;

// From the Teensy's "#H,..." health line. -1 means "never reported".
static float   gVbat  = -1.0f;
static float   gTmax  = -1.0f;
static int8_t  gTeensyMode  = -1;
static int8_t  gTeensyArmed = -1;

enum class Ota : uint8_t { IDLE, STAGING, STAGED, COMMITTING, FAILED };
static Ota      gOta = Ota::IDLE;
// gOtaRecv counts bytes taken off the HTTP body and drives the progress bar;
// gOtaSent counts what actually reached the Teensy (blank and non-':' lines in
// the .hex are skipped, so the two differ slightly). gOtaLines is the record
// count the Teensy itself acknowledges -- the only one of the three that is
// evidence the far end is keeping up.
static uint32_t gOtaRecv = 0, gOtaSent = 0, gOtaTotal = 0, gOtaLines = 0;
static bool     gOtaAck = false;       // set when "#OTA ACK" arrives
static bool     gOtaReady = false;     // set when "#OTA READY" arrives
static char     gOtaErr[96] = {0};
static char     gOtaLine[160];         // partial hex record across HTTP chunks
static size_t   gOtaLineLen = 0;
static uint32_t gOtaChunkAcc = 0;
static uint32_t gOtaCommitMs = 0;      // when u2 went out; the Teensy then reboots

static bool     gClientLive = false;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

static bool clientLive() { return (millis() - gLastPingMs) < kPingStaleMs && gLastPingMs != 0; }
static bool uartLive()   { return gLastTeensyMs != 0 && (millis() - gLastTeensyMs) < kUartStaleMs; }

/** Escape a firmware line so it can be embedded in a JSON string. */
static void jsonEscape(const char* in, char* out, size_t outSize) {
  size_t o = 0;
  for (size_t i = 0; in[i] && o + 7 < outSize; i++) {
    const unsigned char c = (unsigned char)in[i];
    if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = c; }
    else if (c < 0x20 || c > 0x7e) { o += snprintf(out + o, outSize - o, "\\u%04x", c); }
    else out[o++] = c;
  }
  out[o] = '\0';
}

/** Pull a bare string value out of a flat JSON object: {"c":"a0"} -> a0. */
static bool jsonStr(const char* json, const char* key, char* out, size_t outSize) {
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p + strlen(pat), ':');
  if (!p) return false;
  p++;
  while (*p == ' ') p++;
  if (*p != '"') return false;
  p++;
  size_t o = 0;
  while (*p && *p != '"' && o + 1 < outSize) out[o++] = *p++;
  out[o] = '\0';
  return true;
}

/** Pull an integer value: {"m":2} -> 2. */
static bool jsonInt(const char* json, const char* key, int* out) {
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p + strlen(pat), ':');
  if (!p) return false;
  *out = atoi(p + 1);
  return true;
}

static void wsLog(const char* line, const char* level) {
  if (webSocket.connectedClients() == 0) return;
  char esc[420], msg[512];
  jsonEscape(line, esc, sizeof(esc));
  snprintf(msg, sizeof(msg), "{\"t\":\"log\",\"lvl\":\"%s\",\"s\":\"%s\"}", level, esc);
  webSocket.broadcastTXT(msg);
}

static const char* otaPhaseName() {
  switch (gOta) {
    case Ota::IDLE:       return "idle";
    case Ota::STAGING:    return "staging";
    case Ota::STAGED:     return "staged";
    case Ota::COMMITTING: return "committing";
    default:              return "error";
  }
}

static void wsOta() {
  if (webSocket.connectedClients() == 0) return;
  char esc[128], msg[256];
  jsonEscape(gOtaErr, esc, sizeof(esc));
  snprintf(msg, sizeof(msg),
           "{\"t\":\"ota\",\"phase\":\"%s\",\"sent\":%lu,\"total\":%lu,"
           "\"lines\":%lu,\"err\":\"%s\"}",
           otaPhaseName(), (unsigned long)gOtaRecv, (unsigned long)gOtaTotal,
           (unsigned long)gOtaLines, esc);
  webSocket.broadcastTXT(msg);
}

// ---------------------------------------------------------------------------
// Command whitelist
//
// The browser is an untrusted input to a machine that spins three flywheels,
// so this mirrors dashboard/schemas.py's CMD_OK rather than trusting the page.
// Notable exclusions:
//   'u' -- OTA. Driven only by the HTTP endpoints, never by a socket message,
//          so a stray command can never interleave with a hex stream.
//   'h' -- ambiguous across builds ('h' is HALT on the AutoTrim grammar and a
//          keepalive on the legacy one). The keepalive is generated here, and
//          halt is reached through 'p' or a mode change.
// ---------------------------------------------------------------------------
static bool commandAllowed(const char* c) {
  const size_t n = strlen(c);
  if (n == 0 || n > 12) return false;

  switch (c[0]) {
    case 'a': case 'z': case 'p': case 't': case 'l': case 'y':
      return n == 2 && (c[1] == '0' || c[1] == '1');
    case 'm':
      return n == 2 && c[1] >= '0' && c[1] <= '2';
    case 'c': case 'e': case 'k':
      return n == 1;
    case 'g': {
      if (n < 2) return false;
      const float v = atof(c + 1);
      return v >= 0.0f && v <= 1.0f;
    }
    case 'o': case 'r': {          // edge trim / log marker: signed decimal
      if (n < 2) return false;
      for (size_t i = 1; i < n; i++)
        if (!isdigit((unsigned char)c[i]) && c[i] != '.' && c[i] != '-' && c[i] != '+') return false;
      return true;
    }
    default:
      return false;
  }
}

static void sendToTeensy(const char* line) {
  Serial1.print(line);
  Serial1.print('\n');
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------

bool connectWiFi() {
  if (Serial) Serial.printf("\nConnecting to WiFi \"%s\" ", WIFI_SSID);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_STA);

  // Modem sleep OFF -- required, not an optimisation. See the long measurement
  // note in xiao_teensy_bridge.ino: with the default WIFI_PS_MIN_MODEM the
  // median round trip was 75 ms against 3.9 ms of real RF time, which alone
  // can trip the Teensy's 300 ms link watchdog mid-balance.
  WiFi.setSleep(false);

  WiFi.config(XIAO_IP, XIAO_GATEWAY, XIAO_SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    if (Serial) Serial.print(".");
    if (millis() - start > 20000) {
      if (Serial) Serial.println("\nWiFi connect TIMED OUT.");
      return false;
    }
  }
  if (Serial) {
    Serial.println(" connected!");
    Serial.print("  console: http://"); Serial.println(WiFi.localIP());
  }
  return true;
}

// ---------------------------------------------------------------------------
// Teensy line handling
// ---------------------------------------------------------------------------

/** True if the line is a numeric CSV/TSV telemetry row and not text.
 *
 *  SERIALMONITORMODE reprints a non-numeric header row that has the right
 *  field count, so width alone is not enough of a test -- and a header row
 *  pasted into a JSON array would produce a payload the browser cannot parse.
 */
static bool looksNumeric(const char* s) {
  bool sawDigit = false;
  for (size_t i = 0; s[i]; i++) {
    // via unsigned char: isdigit() on a negative char is undefined, and a
    // corrupted byte on a 1 Mbaud line is exactly how one gets here.
    const char c = s[i];
    if (isdigit((unsigned char)c)) { sawDigit = true; continue; }
    if (c == ',' || c == '\t' || c == '.' || c == '-' || c == '+' ||
        c == 'e' || c == 'E' || c == ' ') continue;
    return false;
  }
  return sawDigit;
}

/** "#H,24.5,2,0,41.0" -> battery / mode / armed / hottest controller. */
static void parseHealth(const char* line) {
  float v = -1, t = -1;
  int mode = -1, armed = -1;
  if (sscanf(line + 3, "%f,%d,%d,%f", &v, &mode, &armed, &t) >= 1) {
    gVbat = v;
    if (mode >= 0) gTeensyMode = (int8_t)mode;
    if (armed >= 0) gTeensyArmed = (int8_t)armed;
    if (t > -100) gTmax = t;
  }
}

static void handleTeensyLine(char* line, size_t len) {
  gLastTeensyMs = millis();
  gLineCount++;

  // flash_move() reboots the Teensy without replying, so the boot banner
  // reappearing is the only positive evidence the new firmware is running.
  if (gOta == Ota::COMMITTING && strncmp(line, "started -", 9) == 0) {
    gOta = Ota::IDLE;
    gOtaErr[0] = '\0';
    gOtaRecv = gOtaSent = gOtaLines = 0;
    wsOta();
    wsLog("# [xiao] Teensy rebooted -- new firmware is running", "info");
  }

  // --- OTA protocol replies, consumed here and not forwarded as telemetry ---
  if (strncmp(line, "#OTA ", 5) == 0) {
    if (strncmp(line + 5, "ACK", 3) == 0) {
      gOtaAck = true;
      // "#OTA ACK <lines> <bytes>"
      unsigned long l = 0;
      if (sscanf(line + 8, "%lu", &l) == 1) gOtaLines = l;
    } else if (strncmp(line + 5, "READY", 5) == 0) {
      gOtaReady = true;
    } else if (strncmp(line + 5, "REFUSED", 7) == 0 ||
               strncmp(line + 5, "ERR", 3) == 0) {
      snprintf(gOtaErr, sizeof(gOtaErr), "%s", line + 5);
      gOta = Ota::FAILED;
      gOtaAck = true;            // unblock any wait -- the error is the answer
    }
    wsLog(line, "warn");
    return;
  }

  if (strncmp(line, "#H,", 3) == 0) {
    parseHealth(line);
    return;                       // health is folded into the status message
  }

  if (line[0] == '#') {
    // Firmware console output. Surface refusals and warnings; the rest is
    // boot chatter that the simplified console deliberately does not show.
    const bool warn = strstr(line, "REFUSED") || strstr(line, "WARNING") ||
                      strstr(line, "disarmed") || strstr(line, "Error") ||
                      strstr(line, "error");
    if (warn) wsLog(line, "warn");
  } else if (looksNumeric(line)) {
    // Newest wins: the browser gets 25 Hz, the Teensy sends 250-500, and a
    // queue would only add latency to a live instrument. The UDP path below
    // still gets every single line, so recordings stay full-rate.
    memcpy(gLastTlm, line, len + 1);
    gTlmPending = true;
  }

  // --- UDP relay, unchanged from the old bridge: every line, verbatim ---
  if (gHaveLaptop) {
    udp.beginPacket(gLaptopIP, gLaptopPort);
    udp.write((const uint8_t*)line, len);
    udp.endPacket();
  }
}

/** Drain Serial1, at most kMaxLinesPerLoop complete lines per call. */
static void pollSerial1() {
  uint8_t handled = 0;
  while (Serial1.available() && handled < kMaxLinesPerLoop) {
    const char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (gSerialLen == 0) continue;
      gSerialBuf[gSerialLen] = '\0';
      handleTeensyLine(gSerialBuf, gSerialLen);
      gSerialLen = 0;
      handled++;
      continue;
    }
    if (gSerialLen < sizeof(gSerialBuf) - 1) gSerialBuf[gSerialLen++] = c;
    // else: overlong line, drop the overflow and resync at the terminator
  }
}

// ---------------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------------

static void pushTelemetry() {
  if (!gTlmPending || webSocket.connectedClients() == 0) return;
  gTlmPending = false;

  // The line is already comma- or tab-separated numbers, so wrapping it in
  // brackets IS the JSON array -- no parse, no float round trip, no loss of
  // the precision the firmware chose to print.
  char msg[600];
  size_t o = snprintf(msg, sizeof(msg), "{\"t\":\"tlm\",\"v\":[");
  for (size_t i = 0; gLastTlm[i] && o + 2 < sizeof(msg); i++)
    msg[o++] = (gLastTlm[i] == '\t') ? ',' : gLastTlm[i];
  snprintf(msg + o, sizeof(msg) - o, "]}");
  webSocket.broadcastTXT(msg);
}

static void pushStatus() {
  if (webSocket.connectedClients() == 0) return;
  // Optional fields are rendered into their own buffers and pasted into ONE
  // snprintf with a literal format. Walking an offset across several snprintf
  // calls invites the classic bug -- snprintf returns what it WOULD have
  // written, so one truncation makes every later `sizeof(msg) - o` underflow
  // into a huge size_t.
  char vbat[32] = "", tmax[32] = "", mode[24] = "", armed[24] = "";
  if (gVbat >= 0)        snprintf(vbat,  sizeof(vbat),  ",\"vbat\":%.2f", gVbat);
  if (gTmax >= 0)        snprintf(tmax,  sizeof(tmax),  ",\"tmax\":%.1f", gTmax);
  if (gTeensyMode >= 0)  snprintf(mode,  sizeof(mode),  ",\"mode\":%d",  (int)gTeensyMode);
  if (gTeensyArmed >= 0) snprintf(armed, sizeof(armed), ",\"armed\":%d", (int)gTeensyArmed);

  char msg[320];
  snprintf(msg, sizeof(msg),
    "{\"t\":\"sys\",\"uart\":%d,\"rate\":%d,\"rssi\":%d,\"ota\":\"%s\"%s%s%s%s}",
    uartLive() ? 1 : 0, (int)(gRateHz + 0.5f), (int)WiFi.RSSI(), otaPhaseName(),
    vbat, tmax, mode, armed);
  webSocket.broadcastTXT(msg);
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) {
    gLastPingMs = millis();       // a fresh connection counts as alive
    wsOta();
    return;
  }
  if (type == WStype_DISCONNECTED) {
    // Belt and braces on top of the deadman: if that was the last client, say
    // so immediately rather than waiting out the 500 ms stale window.
    if (webSocket.connectedClients() <= 1) {
      sendToTeensy("a0");
      gLastPingMs = 0;
    }
    return;
  }
  if (type != WStype_TEXT || len == 0) return;

  char json[256];
  const size_t n = len < sizeof(json) - 1 ? len : sizeof(json) - 1;
  memcpy(json, payload, n);
  json[n] = '\0';

  char kind[16];
  if (!jsonStr(json, "t", kind, sizeof(kind))) return;

  if (strcmp(kind, "ping") == 0) {
    gLastPingMs = millis();
    return;
  }

  // A command must never interleave with a hex record on the wire.
  if (gOta == Ota::STAGING || gOta == Ota::COMMITTING) {
    wsLog("# [xiao] command ignored: OTA in progress", "warn");
    return;
  }

  if (strcmp(kind, "cmd") == 0) {
    char c[16];
    if (!jsonStr(json, "c", c, sizeof(c))) return;
    if (!commandAllowed(c)) {
      char m[96];
      snprintf(m, sizeof(m), "# [xiao] REJECTED '%s': not in the command grammar", c);
      wsLog(m, "warn");
      return;
    }
    sendToTeensy(c);
  } else if (strcmp(kind, "mode") == 0) {
    int m;
    if (!jsonInt(json, "m", &m) || m < 0 || m > 2) return;
    // Disarm first, unconditionally. The Teensy does this too on a mode
    // change, but the command that stops the wheels should not depend on the
    // command that changes the mode arriving intact.
    sendToTeensy("a0");
    char c[4]; snprintf(c, sizeof(c), "m%d", m);
    sendToTeensy(c);
  }
}

// ---------------------------------------------------------------------------
// OTA
// ---------------------------------------------------------------------------

/** Pump the socket and the UART while blocked, so the UI stays live. */
static void otaPump() {
  webSocket.loop();
  pollSerial1();
}

static bool otaWait(volatile bool* flag, uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!*flag) {
    // "#OTA ERR"/"#OTA REFUSED" is a definitive answer, so stop waiting for
    // the affirmative one. Without this a refused handshake sat here for the
    // whole timeout before reporting a reason it already had.
    if (gOta == Ota::FAILED) return false;
    if (millis() - start > timeoutMs) return false;
    otaPump();
    delay(1);
  }
  return true;
}

static void otaFail(const char* why) {
  if (gOta != Ota::FAILED) snprintf(gOtaErr, sizeof(gOtaErr), "%s", why);
  gOta = Ota::FAILED;
  wsOta();
}

static bool otaBegin() {
  gOtaRecv = gOtaSent = gOtaLines = gOtaChunkAcc = 0;
  gOtaLineLen = 0;
  gOtaErr[0] = '\0';
  gOtaReady = gOtaAck = false;
  gOta = Ota::STAGING;

  // The Teensy refuses this unless it is IDLE, disarmed and its wheels are
  // stopped -- the interlock lives there, next to the thing it protects.
  sendToTeensy("u1");
  if (!otaWait(&gOtaReady, kOtaReadyTimeout)) {
    if (gOta != Ota::FAILED) otaFail("Teensy did not answer u1 (link down?)");
    return false;
  }
  if (gOta == Ota::FAILED) return false;
  wsOta();
  return true;
}

/** Send one complete hex record, and acknowledge-gate every chunk. */
static bool otaEmitLine(const char* line, size_t len) {
  if (len == 0) return true;
  if (line[0] != ':') return true;      // comments/blank lines in a .hex: skip

  Serial1.write((const uint8_t*)line, len);
  Serial1.write('\n');
  gOtaSent += len + 1;
  gOtaChunkAcc += len + 1;

  if (gOtaChunkAcc >= kOtaChunkBytes) {
    gOtaChunkAcc = 0;
    gOtaAck = false;
    sendToTeensy("u?");
    if (!otaWait(&gOtaAck, kOtaAckTimeout)) {
      otaFail("Teensy stopped acknowledging (flash write stalled?)");
      return false;
    }
    if (gOta == Ota::FAILED) return false;
    wsOta();
  }
  return true;
}

static bool otaFeed(const uint8_t* data, size_t len) {
  if (gOta != Ota::STAGING) return false;
  gOtaRecv += len;
  for (size_t i = 0; i < len; i++) {
    const char c = (char)data[i];
    if (c == '\n' || c == '\r') {
      if (gOtaLineLen == 0) continue;
      gOtaLine[gOtaLineLen] = '\0';
      const bool ok = otaEmitLine(gOtaLine, gOtaLineLen);
      gOtaLineLen = 0;
      if (!ok) return false;
      continue;
    }
    if (gOtaLineLen < sizeof(gOtaLine) - 1) {
      gOtaLine[gOtaLineLen++] = c;
    } else {
      otaFail("hex record longer than 158 chars -- not an Intel HEX file?");
      return false;
    }
  }
  return true;
}

static bool otaFinish() {
  if (gOta != Ota::STAGING) return false;
  if (gOtaLineLen > 0) {                       // file without a trailing newline
    gOtaLine[gOtaLineLen] = '\0';
    if (!otaEmitLine(gOtaLine, gOtaLineLen)) return false;
    gOtaLineLen = 0;
  }
  gOtaAck = false;
  sendToTeensy("u?");
  if (!otaWait(&gOtaAck, kOtaAckTimeout)) {
    otaFail("no final acknowledgement from the Teensy");
    return false;
  }
  if (gOta == Ota::FAILED) return false;
  gOta = Ota::STAGED;
  wsOta();
  return true;
}

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------

// The page may be opened straight off disk for UI work, in which case its
// Origin is "null" and every fetch/XHR here is cross-origin. Allowing it costs
// nothing that matters: this board is on a private hotspot, and anything that
// can reach :80 can already reach :81, which has no origin check at all.
static void cors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static void handleRoot() {
  cors();
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", (PGM_P)kWebIndexGz, kWebIndexGzLen);
}

static void handleHealth() {
  cors();
  char msg[256];
  snprintf(msg, sizeof(msg),
    "{\"uart\":%d,\"rate\":%d,\"rssi\":%d,\"vbat\":%.2f,\"mode\":%d,"
    "\"armed\":%d,\"ota\":\"%s\",\"clients\":%d,\"page\":\"%s\"}",
    uartLive() ? 1 : 0, (int)(gRateHz + 0.5f), (int)WiFi.RSSI(), gVbat,
    gTeensyMode, gTeensyArmed, otaPhaseName(),
    webSocket.connectedClients(), WEB_INDEX_SHA);
  server.send(200, "application/json", msg);
}

static void handleOtaUploadDone() {
  cors();
  if (gOta == Ota::STAGED) {
    char msg[160];
    snprintf(msg, sizeof(msg),
             "{\"ok\":true,\"records\":%lu,\"bytes\":%lu}",
             (unsigned long)gOtaLines, (unsigned long)gOtaSent);
    server.send(200, "application/json", msg);
  } else {
    // Leave the Teensy in a known state rather than parked in its OTA loop
    // holding a half-written buffer. It frees the buffer and reboots.
    sendToTeensy("u0");
    server.send(500, "text/plain", gOtaErr[0] ? gOtaErr : "upload failed");
  }
}

static void handleOtaUpload() {
  HTTPUpload& up = server.upload();
  switch (up.status) {
    case UPLOAD_FILE_START:
      // contentLength is the whole multipart body, so it overshoots the file
      // by the couple of hundred bytes of MIME headers. That is the right
      // denominator anyway: gOtaRecv counts body bytes, not file bytes.
      gOtaTotal = up.contentLength;
      otaBegin();
      break;
    case UPLOAD_FILE_WRITE:
      otaFeed(up.buf, up.currentSize);
      break;
    case UPLOAD_FILE_END:
      if (gOtaTotal == 0) gOtaTotal = gOtaRecv;   // chunked body: no length up front
      otaFinish();
      break;
    default:
      otaFail("upload aborted by the client");
      sendToTeensy("u0");
      break;
  }
}

static void handleOtaCommit() {
  cors();
  if (gOta != Ota::STAGED) {
    server.send(409, "text/plain", "nothing staged -- upload a .hex first");
    return;
  }
  gOta = Ota::COMMITTING;
  gOtaCommitMs = millis();
  wsOta();
  sendToTeensy("u2");
  server.send(200, "text/plain", "committing");
  // flash_move() reboots the Teensy, so there is no reply to wait for. The
  // status settles back to idle when its boot banner reappears on Serial1.
}

static void handleOtaAbort() {
  cors();
  sendToTeensy("u0");
  gOta = Ota::IDLE;
  gOtaErr[0] = '\0';
  gOtaRecv = gOtaSent = gOtaLines = 0;
  wsOta();
  server.send(200, "text/plain", "aborted");
}

// ---------------------------------------------------------------------------
// UDP -- unchanged behaviour from xiao_teensy_bridge.ino
// ---------------------------------------------------------------------------

static void pollUdpIn() {
  const int size = udp.parsePacket();
  if (size <= 0) return;

  char buf[256];
  int len = udp.read(buf, sizeof(buf) - 1);
  if (len < 0) len = 0;
  buf[len] = '\0';

  gLaptopIP = udp.remoteIP();
  gLaptopPort = udp.remotePort();
  gHaveLaptop = true;

  if (buf[0] == 'x') {           // XIAO-local mode switch, never relayed
    gMode = (buf[1] != '0') ? XiaoMode::TEENSY_BRIDGE : XiaoMode::WIFI_TEST;
    return;
  }

  // Never let a UDP client interleave with a hex stream.
  if (gOta == Ota::STAGING || gOta == Ota::COMMITTING) return;

  Serial1.write((const uint8_t*)buf, len);
  if (len == 0 || buf[len - 1] != '\n') Serial1.write('\n');
}

// ---------------------------------------------------------------------------
// Keepalive + rate
// ---------------------------------------------------------------------------

static void pollKeepalive() {
  // No keepalive during OTA: the Teensy is disarmed and in its flashing loop,
  // and a stray 'k' between hex records would be a parse error there.
  if (gOta == Ota::STAGING || gOta == Ota::COMMITTING) return;

  const bool live = clientLive();
  if (live != gClientLive) {
    gClientLive = live;
    if (!live) {
      // The deadman is about to fire on the Teensy side; make it explicit
      // rather than relying on the timeout alone.
      sendToTeensy("a0");
    }
  }
  if (!live) return;

  const uint32_t now = millis();
  if (now - gLastKeepaliveMs < kKeepaliveMs) return;
  gLastKeepaliveMs = now;
  sendToTeensy("k");
}

static void updateRate() {
  const uint32_t now = millis();
  if (now - gRateWindowMs < 1000) return;
  const float span = (now - gRateWindowMs) / 1000.0f;
  gRateHz = span > 0 ? gLineCount / span : 0.0f;
  gLineCount = 0;
  gRateWindowMs = now;
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  const uint32_t wait = millis();
  while (!Serial && millis() - wait < 3000) delay(10);

  Serial1.begin(TEENSY_LINK_BAUD, SERIAL_8N1, D7, D6);   // D7=RX, D6=TX

  if (Serial) Serial.println("\n=== XIAO Cubli Console ===");
  connectWiFi();

  udp.begin(UDP_PORT);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/health", HTTP_GET, handleHealth);
  server.on("/ota/upload", HTTP_POST, handleOtaUploadDone, handleOtaUpload);
  server.on("/ota/commit", HTTP_POST, handleOtaCommit);
  server.on("/ota/abort", HTTP_POST, handleOtaAbort);
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) { cors(); server.send(204); }
    else server.send(404, "text/plain", "not found");
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(onWsEvent);

  gRateWindowMs = millis();

  if (Serial) {
    Serial.printf("HTTP  : http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.printf("WS    : ws://%s:%u/\n", WiFi.localIP().toString().c_str(), WS_PORT);
    Serial.printf("UDP   : :%u (unchanged -- the Python tools still work)\n", UDP_PORT);
    Serial.printf("page  : %s (%u B gzipped)\n", WEB_INDEX_SHA, (unsigned)kWebIndexGzLen);
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    // Never mid-flash: a reconnect takes seconds and would strand the Teensy
    // waiting for records. Fail the transfer instead and let it be retried.
    if (gOta == Ota::STAGING) { otaFail("WiFi dropped during upload"); sendToTeensy("u0"); }
    connectWiFi();
  }

  pollSerial1();
  pollUdpIn();
  server.handleClient();
  webSocket.loop();
  pollKeepalive();
  updateRate();

  const uint32_t now = millis();

  // A commit that produced no boot banner: either the image was rejected on
  // the Teensy side after u2, or the board is not coming back. Say so instead
  // of leaving the console stuck on "flashing" forever.
  if (gOta == Ota::COMMITTING && now - gOtaCommitMs > 15000) {
    otaFail("no boot banner after commit -- check the Teensy over USB");
  }

  if (now - gLastWsTlmMs >= kWsTelemetryMs) { gLastWsTlmMs = now; pushTelemetry(); }
  if (now - gLastWsStatusMs >= kWsStatusMs) { gLastWsStatusMs = now; pushStatus(); }
}
