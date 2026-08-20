/*
  Stage 0a -- XIAO ESP32C6 alive over USB.  NO WiFi join, NO Teensy, NO wires.
  ---------------------------------------------------------------------------
  The only thing this proves, and the only thing it is meant to prove:

    * the esp32 board package is installed and "XIAO_ESP32C6" is selected
    * the board enumerates, you know WHICH COM port it is
    * "USB CDC On Boot" is ENABLED  (if it is disabled you get a COM port but
      never a single character -- the single most common wasted hour on this
      board)
    * the 3.3 V regulator and the chip are running

  If this sketch does not print, NOTHING further up the ladder can work, and
  every WiFi symptom you chase is a guess. Start here even if you are sure.

  RUN IT WITH THE CUBE POWER OFF and only the XIAO's USB-C attached.
  See ../../README.md, "Power rules", for why USB + 5 V rail together is a
  hardware risk on this build.

  Serial Monitor: 115200 baud.  Expect one line per second.

  WRITE DOWN THE MAC ADDRESS it prints. You will want it in Stage 2/3 to pick
  the XIAO out of the hotspot's client list, and in `arp -a` output.
*/

#include <WiFi.h>          // for WiFi.macAddress() only -- no join is attempted

// WiFi.macAddress() returns 00:00:00:00:00:00 if the WiFi driver has never
// been initialised -- the MAC lives in efuse but the Arduino wrapper reads it
// out of the netif, which does not exist yet. Measured on this board: all
// zeros, every boot. WiFi.mode(WIFI_STA) brings the driver up far enough to
// populate it and is NOT a join: no SSID, no password, no connect attempt, so
// this sketch still proves exactly what its header claims and nothing more.

static uint32_t gLastPrintMs = 0;
static uint32_t gSeq = 0;

void setup() {
  Serial.begin(115200);

  // Native USB CDC needs a moment before the host has the port open. 3 s is a
  // bounded wait, not a `while(!Serial)` hang -- the board still runs if you
  // never open a monitor.
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) { delay(10); }

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);   // XIAO C6 user LED is active LOW -- the
                                  // blink phase may look inverted. Cosmetic.
#endif

  Serial.println();
  Serial.println("=== Stage 0a: XIAO ESP32C6 alive ===");
  Serial.print  ("  chip:       "); Serial.println(ESP.getChipModel());
  Serial.print  ("  cores:      "); Serial.println(ESP.getChipCores());
  Serial.print  ("  flash:      "); Serial.print(ESP.getFlashChipSize() / 1024); Serial.println(" KiB");
  WiFi.mode(WIFI_STA);            // driver up, no join -- see note at the top
  Serial.print  ("  MAC (STA):  "); Serial.println(WiFi.macAddress());
  Serial.println("  >>> WRITE THE MAC DOWN -- Stage 2/3 use it to identify");
  Serial.println("      this board in the hotspot client list and in arp -a.");
  Serial.println();
}

void loop() {
  const uint32_t now = millis();
  if (now - gLastPrintMs < 1000) { return; }
  gLastPrintMs = now;

#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, (gSeq & 1) ? HIGH : LOW);
#endif

  Serial.printf("[alive] seq=%lu uptime=%lus free_heap=%lu\n",
                (unsigned long)gSeq++,
                (unsigned long)(now / 1000),
                (unsigned long)ESP.getFreeHeap());
}
