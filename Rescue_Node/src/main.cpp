#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// XIAO ESP32-S3 + Wio-SX1262 전용 물리 GPIO 매핑
#define LORA_NSS   1   // D1 (CS / NSS)
#define LORA_BUSY  2   // D2 (BUSY)
#define LORA_NRST  3   // D3 (RESET)
#define LORA_DIO1  4   // D4 (DIO1)

// XIAO ESP32-S3 표준 SPI 핀
#define LORA_SCK   7   // D8
#define LORA_MISO  8   // D9
#define LORA_MOSI  9   // D10

// SPI 객체 명시적 생성
SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);

bool loraOK = false;

void setup() {
    Serial.begin(115200);
    
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000));
    delay(500);

    Serial.println("\n==========================================");
    Serial.println("   [+] LoRa Pin Mapping Test Booting...   ");
    Serial.println("==========================================");

    // SPI 핀 명시적 시작
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // SX1262 초기화 시도
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);
    
    if (state == RADIOLIB_ERR_NONE) {
        loraOK = true;
        Serial.println(" [SUCCESS] LoRa Radio Init Success!");
    } else {
        Serial.printf(" [FAIL] LoRa Init Failed! Error Code: %d\n", state);
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