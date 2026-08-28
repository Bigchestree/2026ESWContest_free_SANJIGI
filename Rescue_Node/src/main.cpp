#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// XIAO ESP32-S3 + Wio-SX1262 GPIO 매핑
#define LORA_NSS   1   // D1 (CS / NSS)
#define LORA_BUSY  2   // D2 (BUSY)
#define LORA_NRST  3   // D3 (RESET)
#define LORA_DIO1  4   // D4 (DIO1)

#define LORA_SCK   7   // D8 (SCK)
#define LORA_MISO  8   // D9 (MISO)
#define LORA_MOSI  9   // D10 (MOSI)

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);

bool loraOK = false;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n==========================================");
    Serial.println("   [+] Wio-SX1262 Tx Power Fix Mode...    ");
    Serial.println("==========================================");

    // SPI 버스 시작
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // 1. 라디오 기본 초기화 (주파수 923.0MHz, BW 125.0kHz, SF 9, CR 7, SyncWord 0x12, TxPower 10dBm)
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);

    if (state == RADIOLIB_ERR_NONE) {
        // 2. Wio-SX1262 온보드 TCXO(전원) 제어 구문
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
        
        // 송신 시도
        int txState = radio.transmit(payload);

        if (txState == RADIOLIB_ERR_NONE) {
            Serial.printf("[TX Success] Sent: %s\n", payload.c_str());
        } else {
            Serial.printf("[TX Fail] Code: %d\n", txState);
        }
    } else {
        Serial.println("[Waiting] LoRa not ready...");
    }

    delay(1000);
}