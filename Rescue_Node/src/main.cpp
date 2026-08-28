#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// ========================================================
// XIAO ESP32-S3 + Wio-SX1262 소켓 전용 물리 GPIO 번호
// ========================================================
#define LORA_NSS   1   // D1 (CS / NSS)
#define LORA_BUSY  2   // D2 (BUSY)
#define LORA_NRST  3   // D3 (RESET)
#define LORA_DIO1  4   // D4 (DIO1)

// XIAO ESP32-S3 온보드 SPI 실제 GPIO
#define LORA_SCK   7   // D8 (SCK)
#define LORA_MISO  8   // D9 (MISO)
#define LORA_MOSI  9   // D10 (MOSI)

// SPI 객체 명시적 생성 (FSPI 버스)
SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n==========================================");
    Serial.println("   [+] Wio-SX1262 Socket Module Test...   ");
    Serial.println("==========================================");

    // 1. 소켓 연결 핀 강제 리셋 시퀀스 (칩 깨우기)
    pinMode(LORA_NRST, OUTPUT);
    digitalWrite(LORA_NRST, LOW);
    delay(20);
    digitalWrite(LORA_NRST, HIGH);
    delay(50);

    // 2. SPI 버스 명시적 통신 개시
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // 3. SX1262 초기화 시도
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);

    if (state == RADIOLIB_ERR_NONE) {
        // Wio-SX1262 온보드 TCXO 및 RF 스위치 제어
        radio.setTCXO(1.6);
        radio.setDio2AsRfSwitch(true);

        Serial.println(" [SUCCESS] LoRa Radio Init Success!");
    } else {
        Serial.printf(" [FAIL] Radio Init Failed! Code: %d\n", state);
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