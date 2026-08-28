#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// XIAO ESP32-S3 + Wio-SX1262 핀 매핑 (D1~D4, D8~D10)
#define LORA_NSS   D1
#define LORA_BUSY  D2
#define LORA_NRST  D3
#define LORA_DIO1  D4

#define LORA_SCK   D8
#define LORA_MISO  D9
#define LORA_MOSI  D10

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);

int lastErrorCode = 0;

void setup() {
    Serial.begin(115200);
    
    // 시리얼 연결될 때까지 기다림
    while (!Serial) {
        delay(100);
    }
    delay(1000);

    Serial.println("\n\n==========================================");
    Serial.println("   [+] LoRa Diagnosis Starting...         ");
    Serial.println("==========================================");

    // SPI 통신 초기화
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // SX1262 라디오 시작
    lastErrorCode = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);
}

void loop() {
    if (lastErrorCode == RADIOLIB_ERR_NONE) {
        String payload = "TARGET:PING,ALT:100.0";
        int txState = radio.transmit(payload);
        if (txState == RADIOLIB_ERR_NONE) {
            Serial.printf("[TX Success] Sent: %s\n", payload.c_str());
        } else {
            Serial.printf("[TX Fail] Code: %d\n", txState);
        }
    } else {
        // 에러 코드를 1초마다 지속적으로 찍음
        Serial.printf("[CRITICAL ERROR] LoRa Init Failed! Error Code: %d\n", lastErrorCode);
    }

    delay(1000);
}