#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// Wio-SX1262 소켓 전용 물리 GPIO 번호
#define LORA_NSS   1   // D1
#define LORA_BUSY  2   // D2
#define LORA_NRST  3   // D3
#define LORA_DIO1  4   // D4

#define LORA_SCK   7   // D8
#define LORA_MISO  8   // D9
#define LORA_MOSI  9   // D10

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);

bool loraOK = false;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n==========================================");
    Serial.println("   [+] Wio-SX1262 LDO/TCXO Fix Mode...    ");
    Serial.println("==========================================");

    // 1. 하드웨어 강제 리셋 (BUSY 핀 해제)
    pinMode(LORA_NRST, OUTPUT);
    digitalWrite(LORA_NRST, LOW);
    delay(20);
    digitalWrite(LORA_NRST, HIGH);
    delay(100); // 칩 내부 LDO 안정화 대기

    // 2. SPI 통신 시작
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // 3. SX1262 초기화
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);

    if (state == RADIOLIB_ERR_NONE) {
        // Wio-SX1262 전용 전원/안테나 제어 (BUSY 타임아웃 방지)
        radio.setRegulatorMode(RADIOLIB_SX126X_REGULATOR_LDO);
        radio.setTCXO(1.6);
        radio.setDio2AsRfSwitch(true);

        loraOK = true;
        Serial.println(" [SUCCESS] LoRa Radio Init Success!");
    } else {
        Serial.printf(" [FAIL] Radio Init Failed! Code: %d\n", state);
    }
    Serial.println("==========================================\n");
}

void loop() {
    if (loraOK) {
        String payload = "TARGET:PING,ALT:100.0";
        int txState = radio.transmit(payload);

        if (txState == RADIOLIB_ERR_NONE) {
            Serial.printf("[TX Success] Sent: %s\n", payload.c_str());
        } else {
            Serial.printf("[TX Fail] Code: %d\n", txState);
        }
    } else {
        Serial.println("[Waiting] LoRa module not initialized...");
    }

    delay(1000);
}