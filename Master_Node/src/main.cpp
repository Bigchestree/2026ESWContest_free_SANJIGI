#include <Arduino.h>
#include <RadioLib.h>
#include <TFT_eSPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <algorithm>
#include <math.h>

// Wio-SX1262 핀 매핑
#define LORA_NSS   D1
#define LORA_BUSY  D2
#define LORA_NRST  D3
#define LORA_DIO1  D4

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
TFT_eSPI tft = TFT_eSPI();

// ==========================================
// 1. 하드코딩된 앵커 상대 좌표 (Master = Anchor 1 = 원점 0,0,0)
// ==========================================
const float AX = 0.0,   AY = 0.0,   AZ = 0.0;     // Anchor 1 (Master)
const float BX = 80.0,  BY = 0.0,   BZ = 0.0;     // Anchor 2
const float CX = 40.0,  CY = 69.3,  CZ = 0.0;     // Anchor 3

// ==========================================
// 2. RSSI 버퍼 및 중앙값(Median) 구조체
// ==========================================
struct RssiBuffer {
    int buf[5];
    int head = 0;
    int count = 0;

    void add(int rssi) {
        buf[head] = rssi;
        head = (head + 1) % 5;
        if (count < 5) count++;
    }

    int getMedian() {
        if (count < 5) return -999;
        int temp[5];
        for (int i = 0; i < 5; i++) temp[i] = buf[i];
        std::sort(temp, temp + 5);
        return temp[2]; // 중앙값 (3번째)
    }
};

RssiBuffer bufA, bufB, bufC;
int medianRssiA = -999, medianRssiB = -999, medianRssiC = -999;
float latestTargetAlt = 0.0; // 수신된 조난자 BME280 고도

volatile bool triggerSnapshot = false;

// ==========================================
// 3. BLE 서비스 및 콜백
// ==========================================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class TriggerCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String rxVal = pChar->getValue().c_str();
        if (rxVal == "MEASURE" || rxVal == "1") {
            triggerSnapshot = true;
        }
    }
};

// Path Loss Model (RSSI -> 3D 전체 거리 변환)
float rssiToDistance3D(int rssi) {
    int A = -40;    // 1m 거리 기준 RSSI (실측 데이터 반영)
    float n = 2.8;  // 경로 손실 지수
    return pow(10.0, (float)(A - rssi) / (10.0 * n));
}

// ==========================================
// 4. 백그라운드 데이터 수신 및 오버라이트
// ==========================================
void parseIncomingPacket(String msg, int directRssi) {
    // 1) Target 직접 수신
    if (msg.startsWith("TARGET:PING")) {
        bufA.add(directRssi);
        if (bufA.count == 5) medianRssiA = bufA.getMedian();

        int altIdx = msg.indexOf("ALT:");
        if (altIdx != -1) {
            latestTargetAlt = msg.substring(altIdx + 4).toFloat();
        }
    }
    // 2) Anchor 2 중계 수신
    else if (msg.startsWith("ANCHOR2:")) {
        int rssiIdx = msg.indexOf("RSSI:");
        int altIdx = msg.indexOf(",ALT:");
        if (rssiIdx != -1) {
            int rssiVal = msg.substring(rssiIdx + 5, altIdx).toInt();
            bufB.add(rssiVal);
            if (bufB.count == 5) medianRssiB = bufB.getMedian();
        }
    }
    // 3) Anchor 3 중계 수신
    else if (msg.startsWith("ANCHOR3:")) {
        int rssiIdx = msg.indexOf("RSSI:");
        int altIdx = msg.indexOf(",ALT:");
        if (rssiIdx != -1) {
            int rssiVal = msg.substring(rssiIdx + 5, altIdx).toInt();
            bufC.add(rssiVal);
            if (bufC.count == 5) medianRssiC = bufC.getMedian();
        }
    }
}

// ==========================================
// 5. Z축(BME280 고도) 보정 2D 삼각측량 1회 연산
// ==========================================
void runTrilaterationSnapshot() {
    if (medianRssiA == -999 || medianRssiB == -999 || medianRssiC == -999) {
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Collecting 5 Samples...", 10, 10);
        return;
    }

    int rA = medianRssiA, rB = medianRssiB, rC = medianRssiC;

    // 1) 3D 직거리 계산
    float d1_3d = rssiToDistance3D(rA);
    float d2_3d = rssiToDistance3D(rB);
    float d3_3d = rssiToDistance3D(rC);

    // 2) Z축 고도차(Delta Z) 피타고라스 정리 보정 -> 2D 평면 거리 변환
    float dzA = fabs(latestTargetAlt - AZ);
    float dzB = fabs(latestTargetAlt - BZ);
    float dzC = fabs(latestTargetAlt - CZ);

    float d1 = (d1_3d > dzA) ? sqrt(d1_3d * d1_3d - dzA * dzA) : 0.1;
    float d2 = (d2_3d > dzB) ? sqrt(d2_3d * d2_3d - dzB * dzB) : 0.1;
    float d3 = (d3_3d > dzC) ? sqrt(d3_3d * d3_3d - dzC * dzC) : 0.1;

    // 3) 보정된 평면 거리로 2D 삼변측량 계산
    float A = 2 * BX;
    float B = 2 * BY;
    float C = d1*d1 - d2*d2 + BX*BX + BY*BY;
    float D = 2 * CX;
    float E = 2 * CY;
    float F = d1*d1 - d3*d3 + CX*CX + CY*CY;

    float targetX = (C * E - F * B) / (A * E - D * B);
    float targetY = (A * F - C * D) / (A * E - D * B);

    // 4) LCD 렌더링
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawString("[ RESCUE TARGET MAP ]", 10, 10);
    tft.printf("\nPos (Master=0,0):\nX: %.1f m, Y: %.1f m\nZ (Alt): %.1f m\n", targetX, targetY, latestTargetAlt);
    tft.printf("\nMed RSSI:\n[%d] [%d] [%d]\n", rA, rB, rC);

    // 맵 시각화
    int screenX = map(targetX, -20, 100, 20, 220);
    int screenY = map(targetY, -20, 100, 220, 40);
    tft.fillCircle(screenX, screenY, 6, TFT_RED);
    tft.drawCircle(screenX, screenY, 12, TFT_YELLOW);
}

// ==========================================
// 6. Setup & Main Loop
// ==========================================
void setup() {
    Serial.begin(115200);

    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Init Master Node...", 10, 10);

    // SX1262 초기화
    radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);

    // BLE 서비스 초기화
    BLEDevice::init("Master_Rescue_Node");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pChar = pService->createCharacteristic(
                                  CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_WRITE
                               );
    pChar->setCallbacks(new TriggerCallbacks());
    pService->start();
    
    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->start();
}

void loop() {
    // 1) 패킷 백그라운드 수신
    String strData;
    int state = radio.receive(strData);
    if (state == RADIOLIB_ERR_NONE) {
        int directRssi = radio.getRSSI();
        parseIncomingPacket(strData, directRssi);
    }

    // 2) BLE [위치 측정] 명령 수신 시 연산 실행
    if (triggerSnapshot) {
        triggerSnapshot = false;
        runTrilaterationSnapshot();
    }
}