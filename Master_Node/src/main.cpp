#include <Arduino.h>
#include <RadioLib.h>
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
        return temp[2];
    }
};

RssiBuffer bufA, bufB, bufC;
int medianRssiA = -999, medianRssiB = -999, medianRssiC = -999;
float latestTargetAlt = 0.0;

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

float rssiToDistance3D(int rssi) {
    int A = -40;    // 1m 거리 기준 RSSI
    float n = 2.8;  // 경로 손실 지수
    return pow(10.0, (float)(A - rssi) / (10.0 * n));
}

// ==========================================
// 4. 백그라운드 데이터 수신 및 파싱
// ==========================================
void parseIncomingPacket(String msg, int directRssi) {
    if (msg.startsWith("TARGET:PING")) {
        bufA.add(directRssi);
        if (bufA.count == 5) medianRssiA = bufA.getMedian();

        int altIdx = msg.indexOf("ALT:");
        if (altIdx != -1) {
            latestTargetAlt = msg.substring(altIdx + 4).toFloat();
        }
        Serial.printf("[RX] Target Direct PING | RSSI: %d dBm | Alt: %.1f m\n", directRssi, latestTargetAlt);
    }
    else if (msg.startsWith("ANCHOR2:")) {
        int rssiIdx = msg.indexOf("RSSI:");
        if (rssiIdx != -1) {
            int altIdx = msg.indexOf(",ALT:", rssiIdx);
            String rssiStr = (altIdx != -1) ? msg.substring(rssiIdx + 5, altIdx) : msg.substring(rssiIdx + 5);
            int val = rssiStr.toInt();
            bufB.add(val);
            if (bufB.count == 5) medianRssiB = bufB.getMedian();
            Serial.printf("[RX] Anchor 2 Relay | RSSI: %d dBm\n", val);
        }
    }
    else if (msg.startsWith("ANCHOR3:")) {
        int rssiIdx = msg.indexOf("RSSI:");
        if (rssiIdx != -1) {
            int altIdx = msg.indexOf(",ALT:", rssiIdx);
            String rssiStr = (altIdx != -1) ? msg.substring(rssiIdx + 5, altIdx) : msg.substring(rssiIdx + 5);
            int val = rssiStr.toInt();
            bufC.add(val);
            if (bufC.count == 5) medianRssiC = bufC.getMedian();
            Serial.printf("[RX] Anchor 3 Relay | RSSI: %d dBm\n", val);
        }
    }
}

// ==========================================
// 5. Z축 보정 2D 삼각측량 연산 & 시리얼 출력
// ==========================================
void runTrilaterationSnapshot() {
    Serial.println("\n==================================================");
    Serial.println("         [ RESCUE TARGET POSITION REPORT ]        ");
    Serial.println("==================================================");

    if (medianRssiA == -999 || medianRssiB == -999 || medianRssiC == -999) {
        Serial.println(" [!] WARNING: Collecting samples... (Need 5 samples/node)");
        Serial.printf("     Current Samples -> A: %d/5, B: %d/5, C: %d/5\n", bufA.count, bufB.count, bufC.count);
        Serial.println("==================================================\n");
        return;
    }

    int rA = medianRssiA, rB = medianRssiB, rC = medianRssiC;

    float d1_3d = rssiToDistance3D(rA);
    float d2_3d = rssiToDistance3D(rB);
    float d3_3d = rssiToDistance3D(rC);

    float dzA = fabs(latestTargetAlt - AZ);
    float dzB = fabs(latestTargetAlt - BZ);
    float dzC = fabs(latestTargetAlt - CZ);

    float d1 = (d1_3d > dzA) ? sqrt(d1_3d * d1_3d - dzA * dzA) : 0.1;
    float d2 = (d2_3d > dzB) ? sqrt(d2_3d * d2_3d - dzB * dzB) : 0.1;
    float d3 = (d3_3d > dzC) ? sqrt(d3_3d * d3_3d - dzC * dzC) : 0.1;

    float A = 2 * BX;
    float B = 2 * BY;
    float C = d1*d1 - d2*d2 + BX*BX + BY*BY;
    float D = 2 * CX;
    float E = 2 * CY;
    float F = d1*d1 - d3*d3 + CX*CX + CY*CY;

    float denom = (A * E - D * B);
    if (fabs(denom) < 0.001) {
        Serial.println(" [!] ERROR: Mathematical Singularity (Division by zero)");
        return;
    }

    float targetX = (C * E - F * B) / denom;
    float targetY = (A * F - C * D) / denom;

    Serial.printf(" [ Median RSSI ]  A(Master): %d | B: %d | C: %d (dBm)\n", rA, rB, rC);
    Serial.printf(" [ 2D Distance ]  d1: %.2fm | d2: %.2fm | d3: %.2fm\n", d1, d2, d3);
    Serial.println(" --------------------------------------------------");
    Serial.printf(" [ TARGET POS  ]  X = %.2f m\n", targetX);
    Serial.printf("                  Y = %.2f m\n", targetY);
    Serial.printf("                  Z = %.2f m (Altitude)\n", latestTargetAlt);
    Serial.println(" --------------------------------------------------");
    
    Serial.printf("DATA,%.2f,%.2f,%.2f,%d,%d,%d\n", targetX, targetY, latestTargetAlt, rA, rB, rC);
    Serial.println("==================================================\n");
}

// ==========================================
// 6. Setup & Main Loop (부팅 안정화 조치 적용)
// ==========================================
void setup() {
    Serial.begin(115200);
    
    // USB CDC 연결 및 전원 안정을 위한 2초 대기
    delay(2000); 

    Serial.println("\n-------------------------------------------");
    Serial.println(" Initializing Master Rescue System...");
    Serial.println("-------------------------------------------");

    // 1. LoRa 무선 모듈 초기화
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(" [+] LoRa Radio Init Success!");
        radio.startReceive();
    } else {
        Serial.printf(" [-] LoRa Radio Init Failed, code: %d\n", state);
    }

    // 2. BLE 초기화 (경량화 설정으로 메모리 crash 방지)
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
    pAdv->setMinPreferred(0x06);  // 전력 소모 및 간섭 감소 옵션
    pAdv->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println(" [+] BLE Trigger Service Ready!");
    Serial.println(" System Ready. Waiting for LoRa packets / BLE commands...\n");
}

void loop() {
    // 1) 비동기 백그라운드 패킷 수신
    String strData;
    int state = radio.readData(strData);
    if (state == RADIOLIB_ERR_NONE) {
        int directRssi = radio.getRSSI();
        parseIncomingPacket(strData, directRssi);
        radio.startReceive(); // 다음 패킷 수신 대기
    }

    // 2) BLE [위치 측정] 명령 수신 시 연산 실행
    if (triggerSnapshot) {
        triggerSnapshot = false;
        runTrilaterationSnapshot();
    }
}