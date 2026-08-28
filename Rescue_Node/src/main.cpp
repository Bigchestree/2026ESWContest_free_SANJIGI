#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// XIAO ESP32-S3 + Wio-SX1262 표준 아두이노 핀 정의
#define LORA_NSS   D1   // Chip Select (GPIO 1)
#define LORA_BUSY  D2   // Busy (GPIO 2)
#define LORA_NRST  D3   // Reset (GPIO 3)
#define LORA_DIO1  D4   // DIO1 (GPIO 4)

#define LORA_SCK   D8   // SCK (GPIO 7)
#define LORA_MISO  D9   // MISO (GPIO 8)
#define LORA_MOSI  D10  // MOSI (GPIO 9)

// 기본 SPI 버스 사용
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

bool loraOK = false;
int initErrorCode = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n==========================================");
    Serial.println("   [+] Standard SPI LoRa Init Test...     ");
    Serial.println("==========================================");

    // 1. SPI 기본 버스 시작 (D8, D9, D10)
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // 2. SX1262 초기화 시도
    initErrorCode = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);

    if (initErrorCode == RADIOLIB_ERR_NONE) {
        radio.setTCXO(1.6);
        radio.setDio2AsRfSwitch(true);
        loraOK = true;
        Serial.println(" [SUCCESS] LoRa Radio Init Success!");
    } else {
        Serial.printf(" [FAIL] Radio Init Failed! Code: %d\n", initErrorCode);
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
        Serial.printf("[RETRY] Waiting... Error Code: %d\n", initErrorCode);
    }

    delay(1000);
}