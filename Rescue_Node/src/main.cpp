#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// ========================================================
// Seeed Studio XIAO ESP32-S3 + Wio-SX1262 공식 GPIO 핀 맵
// ========================================================
#define LORA_NSS   1   // D1 (NSS / CS)
#define LORA_BUSY  2   // D2 (BUSY)
#define LORA_NRST  3   // D3 (RESET)
#define LORA_DIO1  4   // D4 (DIO1)

// XIAO ESP32-S3 SPI 물리 GPIO
#define LORA_SCK   7   // SCK (GPIO 7)
#define LORA_MISO  8   // MISO (GPIO 8)
#define LORA_MOSI  9   // MOSI (GPIO 9)

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n\n==========================================");
    Serial.println("   [+] Wio-SX1262 Hardware Init Test...   ");
    Serial.println("==========================================");

    // 1. SPI 버스 명시적 시작
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // 2. SX1262 초기화 (주파수 923.0MHz, BW 125, SF 9, CR 7, SyncWord 0x12, TxPower 10)
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(" [SUCCESS] LoRa Radio Init Success! (SX1262 Connected)");
    } else {
        Serial.printf(" [FAIL] Error Code: %d\n", state);
    }
    Serial.println("==========================================\n");
}

void loop() {
    String payload = "TARGET:PING,ALT:100.0";
    
    int txState = radio.transmit(payload);
    if (txState == RADIOLIB_ERR_NONE) {
        Serial.printf("[TX Success] Sent: %s\n", payload.c_str());
    } else {
        Serial.printf("[TX Fail] Code: %d\n", txState);
    }

    delay(1000);
}