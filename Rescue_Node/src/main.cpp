#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// XIAO ESP32-S3 + Wio-SX1262 표준 GPIO 핀
#define LORA_NSS   D1   // GPIO 1
#define LORA_BUSY  D2   // GPIO 2
#define LORA_NRST  D3   // GPIO 3
#define LORA_DIO1  D4   // GPIO 4

#define LORA_SCK   D8   // GPIO 7
#define LORA_MISO  D9   // GPIO 8
#define LORA_MOSI  D10  // GPIO 9

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

bool loraOK = false;
int initErrorCode = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n==========================================");
    Serial.println("   [+] Forced Hardware Reset Test...      ");
    Serial.println("==========================================");

    // 1. NRST / BUSY / CS 핀 강제 하드웨어 초기화 (SX1262 깨우기)
    pinMode(LORA_NRST, OUTPUT);
    digitalWrite(LORA_NRST, LOW);
    delay(10);
    digitalWrite(LORA_NRST, HIGH);
    delay(20); // 칩 깨어나는 시간 대기

    pinMode(LORA_BUSY, INPUT);
    pinMode(LORA_NSS, OUTPUT);
    digitalWrite(LORA_NSS, HIGH);

    // 2. SPI 버스 명시적 시작
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // 3. RadioLib SX1262 초기화 시도
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
        Serial.printf("[RETRY] Error Code: %d\n", initErrorCode);
    }

    delay(1000);
}