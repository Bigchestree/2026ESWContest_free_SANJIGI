#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// =====================================================
// SANJIGI ANCHOR NODE - COMMON CODE
// Anchor 2 upload: #define ANCHOR_NUM 2
// Anchor 3 upload: #define ANCHOR_NUM 3
// =====================================================
#define ANCHOR_NUM 3

#if (ANCHOR_NUM != 2) && (ANCHOR_NUM != 3)
#error "ANCHOR_NUM must be 2 or 3"
#endif

// ---------- Wio-SX1262 B2B pins ----------
#define LORA_NSS   41
#define LORA_BUSY  40
#define LORA_NRST  42
#define LORA_DIO1  39
#define LORA_SCK   7
#define LORA_MISO  8
#define LORA_MOSI  9

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);

bool loraOK = false;

bool extractPressure(const String& msg, float& pressure) {
  int idx = msg.indexOf("PRESS:");
  if (idx < 0) return false;

  String s = msg.substring(idx + 6);
  int comma = s.indexOf(',');
  if (comma >= 0) s = s.substring(0, comma);

  pressure = s.toFloat();
  return isfinite(pressure) && pressure >= 300.0f && pressure <= 1100.0f;
}

void initLoRa() {
  pinMode(LORA_NRST, OUTPUT);
  digitalWrite(LORA_NRST, LOW);
  delay(20);
  digitalWrite(LORA_NRST, HIGH);
  delay(100);

  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int state = radio.begin(
    923.0, 125.0, 9, 7, 0x12,
    10, 8, 1.6, true
  );

  if (state == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    radio.startReceive();
    loraOK = true;
    Serial.printf("[LORA] Anchor %d OK\n", ANCHOR_NUM);
  } else {
    Serial.printf("[LORA] FAIL code=%d\n", state);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.printf("\n=== SANJIGI ANCHOR %d / PRESS MODE ===\n", ANCHOR_NUM);
  initLoRa();
}

void loop() {
  if (!loraOK) {
    delay(1000);
    return;
  }

  String msg;
  int state = radio.readData(msg);

  if (state != RADIOLIB_ERR_NONE) {
    return;
  }

  int targetRssi = (int)radio.getRSSI();

  if (!msg.startsWith("TARGET:PING")) {
    radio.startReceive();
    return;
  }

  float pressure = NAN;
  bool pressOK = extractPressure(msg, pressure);

  // Leave enough separation between Anchor 2 and Anchor 3 relays.
  // A2: 250 ms, A3: 620 ms.
  delay(ANCHOR_NUM == 2 ? 250 : 620);

  String forwardMsg =
      "ANCHOR" + String(ANCHOR_NUM) +
      ":RSSI:" + String(targetRssi);

  // PRESS missing/invalid -> still relay RSSI only.
  if (pressOK) {
    forwardMsg += ",PRESS:" + String(pressure, 2);
  }

  int txState = radio.transmit(forwardMsg);

  if (txState == RADIOLIB_ERR_NONE) {
    Serial.printf("[RELAY TX] %s\n", forwardMsg.c_str());
  } else {
    Serial.printf("[RELAY FAIL] code=%d\n", txState);
  }

  radio.startReceive();
}
