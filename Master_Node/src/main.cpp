#include <Arduino.h>
#include <RadioLib.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <algorithm>
#include <math.h>

#define LORA_NSS   D1
#define LORA_BUSY  D2
#define LORA_NRST  D3
#define LORA_DIO1  D4

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

// 앵커 좌표 설정
const float AX = 0.0,   AY = 0.0,   AZ = 0.0;
const float BX = 80.0,  BY = 0.0,   BZ = 0.0;
const float CX = 40.0,  CY = 69.3,  CZ = 0.0;

struct RssiBuffer {
    int buf[5];
    int head = 0;
    int count = 0;
    unsigned long lastRxTime = 0;

    void add(int rssi) {
        buf[head] = rssi;
        head = (head + 1) % 5;
        if (count < 5) count++;
        lastRxTime = millis();
    }

    int getMedian() {
        if (count < 5) return -999;
        int temp[5];
        for (int i = 0; i < 5; i++) temp[i] = buf[i];
        std::sort(temp, temp + 5);
        return temp[2];
    }

    bool isAlive() {
        return (millis() - lastRxTime < 5000) && (count > 0);
    }
};

RssiBuffer bufA, bufB, bufC;
int medianRssiA = -999, medianRssiB = -999, medianRssiC = -999;
float latestTargetAlt = 0.0;

volatile bool triggerSnapshot = false;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_DATA_UUID      "a3c17822-1d5b-4176-a447-0624916a0487"

BLECharacteristic *pDataChar = NULL;
bool deviceConnected = false;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) { 
        deviceConnected = false; 
        BLEDevice::startAdvertising(); 
    }
};

class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String rxVal = pChar->getValue().c_str();
        if (rxVal == "MEASURE" || rxVal == "1") {
            triggerSnapshot = true;
        }
    }
};

float rssiToDistance3D(int rssi) {
    int A = -40;
    float n = 2.8;
    return pow(10.0, (float)(A - rssi) / (10.0 * n));
}

void sendBleNotify(String msg) {
    if (deviceConnected && pDataChar != NULL) {
        pDataChar->setValue(msg.c_str());
        pDataChar->notify();
    }
}

// ==========================================
// 조난자 패킷 포맷(TARGET:PING,ALT:xxx) 맞춤 파싱
// ==========================================
void parseIncomingPacket(String msg, int directRssi) {
    // 1) 조난자 직접 수신 패킷
    if (msg.startsWith("TARGET:PING")) {
        bufA.add(directRssi);
        if (bufA.count == 5) medianRssiA = bufA.getMedian();

        int altIdx = msg.indexOf("ALT:");
        if (altIdx != -1) {
            latestTargetAlt = msg.substring(altIdx + 4).toFloat();
        }
    }
    // 2) 앵커 2 중계 패킷
    else if (msg.startsWith("ANCHOR2:")) {
        int rssiIdx = msg.indexOf("RSSI:");
        if (rssiIdx != -1) {
            int altIdx = msg.indexOf(",ALT:", rssiIdx);
            String rssiStr = (altIdx != -1) ? msg.substring(rssiIdx + 5, altIdx) : msg.substring(rssiIdx + 5);
            bufB.add(rssiStr.toInt());
            if (bufB.count == 5) medianRssiB = bufB.getMedian();
        }
    }
    // 3) 앵커 3 중계 패킷
    else if (msg.startsWith("ANCHOR3:")) {
        int rssiIdx = msg.indexOf("RSSI:");
        if (rssiIdx != -1) {
            int altIdx = msg.indexOf(",ALT:", rssiIdx);
            String rssiStr = (altIdx != -1) ? msg.substring(rssiIdx + 5, altIdx) : msg.substring(rssiIdx + 5);
            bufC.add(rssiStr.toInt());
            if (bufC.count == 5) medianRssiC = bufC.getMedian();
        }
    }
}

void runTrilaterationSnapshot() {
    if (medianRssiA == -999 || medianRssiB == -999 || medianRssiC == -999) {
        sendBleNotify("ERR:샘플수집중 (A:" + String(bufA.count) + "/5, B:" + String(bufB.count) + "/5, C:" + String(bufC.count) + "/5)");
        return;
    }

    float d1_3d = rssiToDistance3D(medianRssiA);
    float d2_3d = rssiToDistance3D(medianRssiB);
    float d3_3d = rssiToDistance3D(medianRssiC);

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
        sendBleNotify("ERR:연산오류(Singularity)");
        return;
    }

    float targetX = (C * E - F * B) / denom;
    float targetY = (A * F - C * D) / denom;

    String resMsg = "RES:" + String(targetX, 2) + "," + String(targetY, 2) + "," + String(latestTargetAlt, 1) + "," + String(d1, 1) + "," + String(d2, 1) + "," + String(d3, 1);
    sendBleNotify(resMsg);
}

unsigned long lastStatusTime = 0;
void checkAndSendNodeStatus() {
    if (millis() - lastStatusTime > 1000) {
        lastStatusTime = millis();
        String statMsg = "STAT:" + String(bufA.isAlive()?1:0) + "," + String(bufB.isAlive()?1:0) + "," + String(bufC.isAlive()?1:0) + "," + String(medianRssiA) + "," + String(medianRssiB) + "," + String(medianRssiC);
        sendBleNotify(statMsg);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);
    radio.startReceive();

    BLEDevice::init("Master_Rescue_Node");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic *pCmdChar = pService->createCharacteristic(
                                    CHAR_CMD_UUID,
                                    BLECharacteristic::PROPERTY_WRITE
                                 );
    pCmdChar->setCallbacks(new CommandCallbacks());

    pDataChar = pService->createCharacteristic(
                    CHAR_DATA_UUID,
                    BLECharacteristic::PROPERTY_READ |
                    BLECharacteristic::PROPERTY_NOTIFY
                );
    pDataChar->addDescriptor(new BLE2902());

    pService->start();
    
    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->start();
}

void loop() {
    String strData;
    int state = radio.readData(strData);
    if (state == RADIOLIB_ERR_NONE) {
        parseIncomingPacket(strData, radio.getRSSI());
        radio.startReceive();
    }

    checkAndSendNodeStatus();

    if (triggerSnapshot) {
        triggerSnapshot = false;
        runTrilaterationSnapshot();
    }
}