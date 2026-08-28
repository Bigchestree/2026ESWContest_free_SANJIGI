#include <Arduino.h>
#include <RadioLib.h>

// ==========================================
// ⚙️ 앵커 노드 설정 (업로드 시 이 부분만 변경!)
// 앵커 2 업로드 시: #define ANCHOR_NUM 2
// 앵커 3 업로드 시: #define ANCHOR_NUM 3
// ==========================================
#define ANCHOR_NUM 3

#define LORA_NSS   D1
#define LORA_BUSY  D2
#define LORA_NRST  D3
#define LORA_DIO1  D4

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.printf("\n[+] Anchor Node %d Booting...\n", ANCHOR_NUM);

    // LoRa 라디오 초기화
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(" [+] LoRa Init Success!");
        radio.startReceive(); // 패킷 수신 대기 시작
    } else {
        Serial.printf(" [-] LoRa Init Failed: %d\n", state);
    }
}

void loop() {
    String strData;
    // non-blocking 방식으로 수신 확인
    int state = radio.readData(strData);

    // 조난자 패킷 수신 시 ("TARGET:PING,ALT:xxx")
    if (state == RADIOLIB_ERR_NONE && strData.startsWith("TARGET:PING")) {
        int rssi = radio.getRSSI(); // 조난자로부터 수신한 RSSI 측정

        // 고도 값 추출
        String altStr = "0.0";
        int altIdx = strData.indexOf("ALT:");
        if (altIdx != -1) {
            altStr = strData.substring(altIdx + 4);
        }

        // 앵커2와 앵커3 간 전파 충돌 방지 차등 딜레이 (앵커2: 100ms, 앵커3: 250ms)
        delay(ANCHOR_NUM * 80 + 20); 

        // 마스터 노드 전송용 패킷 생성 ("ANCHOR2:RSSI:-68,ALT:120.5")
        String forwardMsg = "ANCHOR" + String(ANCHOR_NUM) + ":RSSI:" + String(rssi) + ",ALT:" + altStr;

        Serial.printf("[TX Relay] %s\n", forwardMsg.c_str());

        radio.transmit(forwardMsg);
        radio.startReceive(); // 전송 후 다시 수신 모드로 복귀
    }
}