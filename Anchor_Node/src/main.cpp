#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// ==========================================
// 앵커 번호만 변경
// 앵커2 -> 2
// 앵커3 -> 3
// ==========================================
#define ANCHOR_NUM 3

// ==========================================
// XIAO ESP32-S3 + Wio-SX1262 B2B 핀
// ==========================================
#define LORA_NSS    41
#define LORA_BUSY   40
#define LORA_NRST   42
#define LORA_DIO1   39

#define LORA_SCK     7
#define LORA_MISO    8
#define LORA_MOSI    9

SPIClass loraSPI(FSPI);

SX1262 radio = new Module(
    LORA_NSS,
    LORA_DIO1,
    LORA_NRST,
    LORA_BUSY,
    loraSPI
);

bool loraOK = false;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=====================================");
    Serial.printf(" ANCHOR NODE %d\n", ANCHOR_NUM);
    Serial.println(" XIAO ESP32-S3 + Wio-SX1262 B2B");
    Serial.println("=====================================");

    // SPI 시작
    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_NSS
    );

    // SX1262 초기화
    int state = radio.begin(
        923.0,   // 주파수
        125.0,   // BW
        9,       // SF
        7,       // CR
        0x12,    // Sync Word
        10,      // TX Power
        8,       // Current Limit
        1.8,     // TCXO
        false    // LDO
    );

    if (state == RADIOLIB_ERR_NONE) {
        radio.setDio2AsRfSwitch(true);

        int rxState = radio.startReceive();

        if (rxState == RADIOLIB_ERR_NONE) {
            loraOK = true;

            Serial.println("[SUCCESS] LoRa Init");
            Serial.println("[SUCCESS] RX Mode");
            Serial.println("Waiting for TARGET:PING...");
        } else {
            Serial.printf("[FAIL] RX Start: %d\n", rxState);
        }
    } else {
        Serial.printf("[FAIL] LoRa Init: %d\n", state);
    }
}

void loop() {

    if (!loraOK) {
        delay(1000);
        return;
    }

    String strData;

    int state = radio.readData(strData);

    // ==========================================
    // 조난자 핑 수신
    // ==========================================
    if (state == RADIOLIB_ERR_NONE) {

        float rssi = radio.getRSSI();
        float snr  = radio.getSNR();

        Serial.println();
        Serial.println("========== RX ==========");
        Serial.print("DATA : ");
        Serial.println(strData);

        Serial.print("RSSI : ");
        Serial.print(rssi);
        Serial.println(" dBm");

        Serial.print("SNR  : ");
        Serial.print(snr);
        Serial.println(" dB");

        // 조난자 패킷만 중계
        if (strData.startsWith("TARGET:PING")) {

            // 고도값 추출
            String altStr = "0.0";

            int altIdx = strData.indexOf("ALT:");

            if (altIdx != -1) {
                altStr = strData.substring(altIdx + 4);
            }

            // 앵커2/3 송신 충돌 방지
            // 앵커2 약 180ms
            // 앵커3 약 260ms
            delay(ANCHOR_NUM * 80 + 20);

            // 마스터로 보낼 패킷
            String forwardMsg =
                "ANCHOR" +
                String(ANCHOR_NUM) +
                ":RSSI:" +
                String((int)rssi) +
                ",ALT:" +
                altStr;

            Serial.print("[TX RELAY] ");
            Serial.println(forwardMsg);

            int txState = radio.transmit(forwardMsg);

            if (txState == RADIOLIB_ERR_NONE) {
                Serial.println("[TX SUCCESS]");
            } else {
                Serial.printf("[TX FAIL] %d\n", txState);
            }
        }

        // 다시 수신 모드
        radio.startReceive();
    }
}