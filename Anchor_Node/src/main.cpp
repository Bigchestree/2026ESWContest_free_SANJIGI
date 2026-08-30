#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// =====================================================
// SANJIGI ANCHOR NODE - COMMON CODE
//
// Anchor 2 : #define ANCHOR_NUM 2
// Anchor 3 : #define ANCHOR_NUM 3
// =====================================================

// 지금 테스트하는 것은 Anchor 2
#define ANCHOR_NUM 2

#if (ANCHOR_NUM != 2) && (ANCHOR_NUM != 3)
#error "ANCHOR_NUM must be 2 or 3"
#endif

// =====================================================
// Wio-SX1262 B2B pins
// =====================================================

#define LORA_NSS   41
#define LORA_BUSY  40
#define LORA_NRST  42
#define LORA_DIO1  39

#define LORA_SCK   7
#define LORA_MISO  8
#define LORA_MOSI  9

SPIClass loraSPI(FSPI);

SX1262 radio = new Module(
  LORA_NSS,
  LORA_DIO1,
  LORA_NRST,
  LORA_BUSY,
  loraSPI
);

bool loraOK = false;

// =====================================================
// DIO1 RX interrupt flag
// =====================================================

volatile bool receivedFlag = false;

void setFlag(void) {
  receivedFlag = true;
}

// =====================================================
// PRESS parser
// =====================================================

bool extractPressure(
  const String& msg,
  float& pressure
) {
  int idx = msg.indexOf("PRESS:");

  if (idx < 0) {
    return false;
  }

  String s = msg.substring(idx + 6);

  int comma = s.indexOf(',');

  if (comma >= 0) {
    s = s.substring(0, comma);
  }

  pressure = s.toFloat();

  return (
    isfinite(pressure) &&
    pressure >= 300.0f &&
    pressure <= 1100.0f
  );
}

// =====================================================
// LoRa init
// =====================================================

void initLoRa() {

  pinMode(LORA_NRST, OUTPUT);

  digitalWrite(LORA_NRST, LOW);
  delay(20);

  digitalWrite(LORA_NRST, HIGH);
  delay(100);

  loraSPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_NSS
  );

  int state = radio.begin(
    923.0,
    125.0,
    9,
    7,
    0x12,
    10,
    8,
    1.6,
    true
  );

  if (state == RADIOLIB_ERR_NONE) {

    radio.setDio2AsRfSwitch(true);

    // 실제 패킷이 들어왔을 때만 flag 발생
    radio.setDio1Action(setFlag);

    receivedFlag = false;

    int rxState = radio.startReceive();

    if (rxState == RADIOLIB_ERR_NONE) {

      loraOK = true;

      Serial.printf(
        "[LORA] Anchor %d OK / Interrupt RX\n",
        ANCHOR_NUM
      );

    } else {

      Serial.printf(
        "[LORA] startReceive FAIL code=%d\n",
        rxState
      );
    }

  } else {

    Serial.printf(
      "[LORA] FAIL code=%d\n",
      state
    );
  }
}

// =====================================================
// Setup
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(1500);

  Serial.printf(
    "\n=== SANJIGI ANCHOR %d / PRESS / INTERRUPT RX ===\n",
    ANCHOR_NUM
  );

  initLoRa();
}

// =====================================================
// Loop
// =====================================================

void loop() {

  if (!loraOK) {
    delay(1000);
    return;
  }

  // 패킷이 실제로 수신된 경우에만 readData()
  if (!receivedFlag) {
    return;
  }

  receivedFlag = false;

  String msg;

  int state = radio.readData(msg);

  if (state != RADIOLIB_ERR_NONE) {

    Serial.printf(
      "[RX ERROR] code=%d\n",
      state
    );

    radio.startReceive();

    return;
  }

  // Target → Anchor RSSI
  int targetRssi = (int)radio.getRSSI();

  Serial.printf(
    "[RX] %s | TARGET RSSI:%d dBm\n",
    msg.c_str(),
    targetRssi
  );

  // ===================================================
  // Target 메시지만 중계
  // ===================================================

  if (!msg.startsWith("TARGET:PING")) {

    radio.startReceive();

    return;
  }

  float pressure = NAN;

  bool pressOK =
    extractPressure(
      msg,
      pressure
    );

  if (pressOK) {

    Serial.printf(
      "[TARGET] RSSI:%d | PRESS:%.2f hPa\n",
      targetRssi,
      pressure
    );

  } else {

    Serial.printf(
      "[TARGET] RSSI:%d | PRESS:INVALID\n",
      targetRssi
    );
  }

  // ===================================================
  // Anchor 중계 충돌 방지
  //
  // Anchor2 = 250 ms
  // Anchor3 = 620 ms
  // ===================================================

  if (ANCHOR_NUM == 2) {
    delay(250);
  } else {
    delay(620);
  }

  // ===================================================
  // Master로 보낼 메시지
  // ===================================================

  String forwardMsg =
    "ANCHOR" +
    String(ANCHOR_NUM) +
    ":RSSI:" +
    String(targetRssi);

  if (pressOK) {

    forwardMsg +=
      ",PRESS:" +
      String(
        pressure,
        2
      );
  }

  // transmit()의 TX 완료 DIO1 신호 때문에
  // 수신 flag가 잘못 남는 것을 방지
  receivedFlag = false;

  int txState =
    radio.transmit(
      forwardMsg
    );

  if (
    txState ==
    RADIOLIB_ERR_NONE
  ) {

    Serial.printf(
      "[RELAY TX] %s\n",
      forwardMsg.c_str()
    );

  } else {

    Serial.printf(
      "[RELAY FAIL] code=%d\n",
      txState
    );
  }

  // TX 완료 인터럽트가 flag에 남았을 수 있으므로 제거
  receivedFlag = false;

  // 다음 Target 패킷 수신
  int rxState =
    radio.startReceive();

  if (
    rxState !=
    RADIOLIB_ERR_NONE
  ) {

    Serial.printf(
      "[RX RESTART FAIL] code=%d\n",
      rxState
    );
  }
}