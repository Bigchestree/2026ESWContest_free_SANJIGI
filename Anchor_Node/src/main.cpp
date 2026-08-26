#include <Arduino.h>
#include <RadioLib.h>

#define ANCHOR_ID "ANCHOR2" // Anchor 3 적용 시 "ANCHOR3"로 변경

#define LORA_NSS   D1
#define LORA_BUSY  D2
#define LORA_NRST  D3
#define LORA_DIO1  D4

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

void setup() {
    Serial.begin(115200);
    radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);
}

void loop() {
    String strData;
    int state = radio.receive(strData);

    // 조난자 패킷 수신 시 ("TARGET:PING,ALT:xxx")
    if (state == RADIOLIB_ERR_NONE && strData.startsWith("TARGET:PING")) {
        int rssi = radio.getRSSI(); // 패킷 수신 RSSI 측정

        // 고도 값 추출
        String altStr = "0.0";
        int altIdx = strData.indexOf("ALT:");
        if (altIdx != -1) {
            altStr = strData.substring(altIdx + 4);
        }

        // 마스터 노드 전송용 패킷 포맷 ("ANCHOR2:RSSI:-68,ALT:120.5")
        String forwardMsg = String(ANCHOR_ID) + ":RSSI:" + String(rssi) + ",ALT:" + altStr;

        delay(100); // 패킷 충돌 방지 미세 대기
        radio.transmit(forwardMsg);
    }
}