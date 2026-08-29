#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// =====================================================
// 이 코드가 앵커2
// =====================================================
#define ANCHOR_NUM 2

// =====================================================
// Wio-SX1262 B2B PIN
// =====================================================
#define LORA_NSS   41
#define LORA_BUSY  40
#define LORA_NRST  42
#define LORA_DIO1  39

#define LORA_SCK    7
#define LORA_MISO   8
#define LORA_MOSI   9

SPIClass loraSPI(FSPI);

SX1262 radio = new Module(
    LORA_NSS,
    LORA_DIO1,
    LORA_NRST,
    LORA_BUSY,
    loraSPI
);

// =====================================================
// RX FLAG
// =====================================================
volatile bool receivedFlag = false;

void setFlag() {
    receivedFlag = true;
}

// =====================================================
// SETUP
// =====================================================
void setup() {

    Serial.begin(115200);
    delay(3000);

    Serial.println();
    Serial.println("=========================");
    Serial.println("ANCHOR 2 START");
    Serial.println("=========================");

    // SPI
    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_NSS
    );

    Serial.println("[SPI] OK");

    // LoRa
    int state = radio.begin(
        923.0,
        125.0,
        9,
        7,
        0x12,
        10,
        8,
        1.8,
        false
    );

    Serial.print("[LoRa begin] ");
    Serial.println(state);

    if (state != RADIOLIB_ERR_NONE) {

        Serial.println("[FAIL] LoRa Init");
        return;
    }

    radio.setDio2AsRfSwitch(true);

    // 패킷 수신 인터럽트
    radio.setPacketReceivedAction(setFlag);

    state = radio.startReceive();

    Serial.print("[RX START] ");
    Serial.println(state);

    Serial.println("WAITING FOR TARGET...");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

    if (!receivedFlag) {
        return;
    }

    receivedFlag = false;

    String msg;

    int state = radio.readData(msg);

    if (state != RADIOLIB_ERR_NONE) {

        Serial.print("[RX ERROR] ");
        Serial.println(state);

        radio.startReceive();
        return;
    }

    // =================================================
    // 받은 패킷 정보
    // =================================================
    int targetRssi =
        (int)radio.getRSSI();

    float targetSnr =
        radio.getSNR();

    Serial.println();
    Serial.println("-------------------------");

    Serial.print("RX DATA : ");
    Serial.println(msg);

    Serial.print("RSSI    : ");
    Serial.println(targetRssi);

    Serial.print("SNR     : ");
    Serial.println(targetSnr);

    // =================================================
    // TARGET 패킷만 처리
    // =================================================
    if (msg.startsWith("TARGET:PING")) {

        // ALT 추출
        String alt = "0.0";

        int altIndex =
            msg.indexOf("ALT:");

        if (altIndex != -1) {

            alt =
                msg.substring(
                    altIndex + 4
                );
        }

        // ---------------------------------------------
        // 마스터로 보낼 패킷 생성
        // ---------------------------------------------
        String relayMsg =
            "ANCHOR2:RSSI:"
            +
            String(targetRssi)
            +
            ",ALT:"
            +
            alt;

        Serial.print("RELAY   : ");
        Serial.println(relayMsg);

        // ---------------------------------------------
        // 약간 기다린 뒤 송신
        // ---------------------------------------------
        delay(180);

        int txState =
            radio.transmit(relayMsg);

        if (txState == RADIOLIB_ERR_NONE) {

            Serial.println(
                "[RELAY SUCCESS]"
            );

        } else {

            Serial.print(
                "[RELAY FAIL] "
            );

            Serial.println(
                txState
            );
        }
    }

    Serial.println("-------------------------");

    // =================================================
    // 다시 수신모드
    // =================================================
    int rxState =
        radio.startReceive();

    if (rxState != RADIOLIB_ERR_NONE) {

        Serial.print(
            "[RX RESTART FAIL] "
        );

        Serial.println(
            rxState
        );
    }
}