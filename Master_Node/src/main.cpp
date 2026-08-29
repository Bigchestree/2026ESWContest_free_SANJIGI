#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <algorithm>

// =====================================================
// LoRa PIN
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
// LoRa RX FLAG
// =====================================================
volatile bool receivedFlag = false;

void setFlag() {
    receivedFlag = true;
}

// =====================================================
// RSSI BUFFER 구조체
// =====================================================
struct RssiBuffer {

    int data[5];

    int index = 0;
    int count = 0;

    int median = -999;

    unsigned long lastRxTime = 0;

    // -------------------------------------------------
    // RSSI 추가
    // -------------------------------------------------
    void add(int rssi) {

        data[index] = rssi;

        index++;

        if (index >= 5) {
            index = 0;
        }

        if (count < 5) {
            count++;
        }

        lastRxTime = millis();

        // 5개가 모이면 중앙값 계산
        if (count == 5) {

            int temp[5];

            for (int i = 0; i < 5; i++) {
                temp[i] = data[i];
            }

            std::sort(
                temp,
                temp + 5
            );

            median = temp[2];
        }
    }

    // -------------------------------------------------
    // 살아있는 노드인지
    // -------------------------------------------------
    bool isAlive() {

        return
            count > 0 &&
            (
                millis() -
                lastRxTime
                <
                5000
            );
    }
};

// =====================================================
// 각각 별도 RSSI BUFFER
// =====================================================

// 마스터가 조난자를 직접 받은 RSSI
RssiBuffer targetBuffer;

// 앵커2가 조난자를 받은 RSSI
RssiBuffer anchor2Buffer;

// 앵커3가 조난자를 받은 RSSI
RssiBuffer anchor3Buffer;


// =====================================================
// 조난자 고도
// =====================================================
float targetAltitude = 0.0;


// =====================================================
// BLE UUID
// =====================================================
#define SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_DATA_UUID "a3c17822-1d5b-4176-a447-0624916a0487"

BLECharacteristic* pDataChar = nullptr;

bool deviceConnected = false;

unsigned long lastStatTime = 0;

// =====================================================
// BLE SERVER CALLBACK
// =====================================================
class ServerCallbacks : public BLEServerCallbacks {

    void onConnect(
        BLEServer* pServer
    ) override {

        deviceConnected = true;

        Serial.println(
            "[BLE] CONNECTED"
        );
    }

    void onDisconnect(
        BLEServer* pServer
    ) override {

        deviceConnected = false;

        Serial.println(
            "[BLE] DISCONNECTED"
        );

        BLEDevice::startAdvertising();
    }
};


// =====================================================
// BLE COMMAND CALLBACK
// =====================================================
class CommandCallbacks :
    public BLECharacteristicCallbacks {

    void onWrite(
    BLECharacteristic* pChar
) override {

    String value =
        pChar->getValue().c_str();

    value.trim();

    Serial.print("[BLE CMD] ");
    Serial.println(value);

    // =================================================
    // 측정 명령
    // =================================================
    if (value == "MEASURE") {

        int rssi1 = targetBuffer.median;
        int rssi2 = anchor2Buffer.median;
        int rssi3 = anchor3Buffer.median;    

        float d1 = rssiToDistance(rssi1);
        float d2 = rssiToDistance(rssi2);
        float d3 = rssiToDistance(rssi3);
        
        Serial.println();
        Serial.println("==============================");
        Serial.println("MEASURE SNAPSHOT");

        // 세 노드 모두 살아있는지 확인
        bool targetOK  = targetBuffer.isAlive();
        bool anchor2OK = anchor2Buffer.isAlive();
        bool anchor3OK = anchor3Buffer.isAlive();

        // 중앙값 5개가 모두 준비됐는지 확인
        bool medianOK =
            targetBuffer.median  != -999 &&
            anchor2Buffer.median != -999 &&
            anchor3Buffer.median != -999;

        if (
            targetOK &&
            anchor2OK &&
            anchor3OK &&
            medianOK
        ) {

            Serial.print("MASTER  RSSI : ");
            Serial.println(rssi1);

            Serial.print("ANCHOR2 RSSI : ");
            Serial.println(rssi2);

            Serial.print("ANCHOR3 RSSI : ");
            Serial.println(rssi3);

            Serial.println("------------------------------");

            Serial.print("MASTER  DIST : ");
            Serial.print(d1, 2);
            Serial.println(" m");

            Serial.print("ANCHOR2 DIST : ");
            Serial.print(d2, 2);
            Serial.println(" m");

            Serial.print("ANCHOR3 DIST : ");
            Serial.print(d3, 2);
            Serial.println(" m");

            Serial.print("ALT          : ");
            Serial.println(targetAltitude);

            Serial.println("[MEASURE READY]");
        }

        else {

            Serial.println("[MEASURE FAIL]");

            if (!targetOK) {
                Serial.println("TARGET NOT ALIVE");
            }

            if (!anchor2OK) {
                Serial.println("ANCHOR2 NOT ALIVE");
            }

            if (!anchor3OK) {
                Serial.println("ANCHOR3 NOT ALIVE");
            }

            if (!medianOK) {
                Serial.println("RSSI MEDIAN NOT READY");
            }
        }

        Serial.println("==============================");
    }
    }
};


// =====================================================
// BLE Notify
// =====================================================
void sendBle(
    String msg
) {

    if (
        deviceConnected &&
        pDataChar != nullptr
    ) {

        pDataChar->setValue(
            msg.c_str()
        );

        pDataChar->notify();

        Serial.print(
            "[BLE TX] "
        );

        Serial.println(
            msg
        );
    }
}

void sendStat() {

    int targetAlive =
        targetBuffer.isAlive() ? 1 : 0;

    int anchor2Alive =
        anchor2Buffer.isAlive() ? 1 : 0;

    int anchor3Alive =
        anchor3Buffer.isAlive() ? 1 : 0;

    String stat =
        "STAT:"
        + String(targetAlive)
        + ","
        + String(anchor2Alive)
        + ","
        + String(anchor3Alive)
        + ","
        + String(targetBuffer.median)
        + ","
        + String(anchor2Buffer.median)
        + ","
        + String(anchor3Buffer.median);

    sendBle(stat);
}

// =====================================================
// RSSI -> 거리(m)
// 현재는 테스트용 기본값
// A = 1m 거리에서 RSSI 기준값
// n = 환경 계수
// =====================================================
float rssiToDistance(int rssi) {

    const int A = -40;
    const float n = 2.8;

    float distance =
        pow(
            10.0,
            (float)(A - rssi) /
            (10.0 * n)
        );

    return distance;
}

// =====================================================
// TARGET 패킷 처리
// =====================================================
void processTarget(
    String msg,
    int directRssi
) {

    // 직접 RSSI 저장
    targetBuffer.add(
        directRssi
    );

    // ALT 추출
    int altIndex =
        msg.indexOf(
            "ALT:"
        );

    if (altIndex != -1) {

        targetAltitude =
            msg.substring(
                altIndex + 4
            ).toFloat();
    }

    Serial.println();
    Serial.println(
        "[TARGET]"
    );

    Serial.print(
        "RSSI   : "
    );

    Serial.println(
        directRssi
    );

    Serial.print(
        "SAMPLE : "
    );

    Serial.print(
        targetBuffer.count
    );

    Serial.println(
        "/5"
    );

    if (
        targetBuffer.median !=
        -999
    ) {

        Serial.print(
            "MEDIAN : "
        );

        Serial.println(
            targetBuffer.median
        );
    }

    Serial.print(
        "ALT    : "
    );

    Serial.println(
        targetAltitude
    );
}


// =====================================================
// ANCHOR2 패킷 처리
// =====================================================
void processAnchor2(
    String msg
) {

    int rssiIndex =
        msg.indexOf(
            "RSSI:"
        );

    if (rssiIndex == -1) {
        return;
    }

    int altIndex =
        msg.indexOf(
            ",ALT:"
        );

    String rssiText;

    if (altIndex != -1) {

        rssiText =
            msg.substring(
                rssiIndex + 5,
                altIndex
            );

    } else {

        rssiText =
            msg.substring(
                rssiIndex + 5
            );
    }

    int anchorRssi =
        rssiText.toInt();

    anchor2Buffer.add(
        anchorRssi
    );

    Serial.println();
    Serial.println(
        "[ANCHOR2]"
    );

    Serial.print(
        "TARGET RSSI : "
    );

    Serial.println(
        anchorRssi
    );

    Serial.print(
        "SAMPLE      : "
    );

    Serial.print(
        anchor2Buffer.count
    );

    Serial.println(
        "/5"
    );

    if (
        anchor2Buffer.median !=
        -999
    ) {

        Serial.print(
            "MEDIAN      : "
        );

        Serial.println(
            anchor2Buffer.median
        );
    }
}


// =====================================================
// ANCHOR3 패킷 처리
// =====================================================
void processAnchor3(
    String msg
) {

    int rssiIndex =
        msg.indexOf(
            "RSSI:"
        );

    if (rssiIndex == -1) {
        return;
    }

    int altIndex =
        msg.indexOf(
            ",ALT:"
        );

    String rssiText;

    if (altIndex != -1) {

        rssiText =
            msg.substring(
                rssiIndex + 5,
                altIndex
            );

    } else {

        rssiText =
            msg.substring(
                rssiIndex + 5
            );
    }

    int anchorRssi =
        rssiText.toInt();

    anchor3Buffer.add(
        anchorRssi
    );

    Serial.println();
    Serial.println(
        "[ANCHOR3]"
    );

    Serial.print(
        "TARGET RSSI : "
    );

    Serial.println(
        anchorRssi
    );

    Serial.print(
        "SAMPLE      : "
    );

    Serial.print(
        anchor3Buffer.count
    );

    Serial.println(
        "/5"
    );

    if (
        anchor3Buffer.median !=
        -999
    ) {

        Serial.print(
            "MEDIAN      : "
        );

        Serial.println(
            anchor3Buffer.median
        );
    }
}


// =====================================================
// 현재 중앙값 상태 표시
// =====================================================
void printAllMedian() {

    Serial.println();
    Serial.println(
        "=============================="
    );

    Serial.println(
        "CURRENT MEDIAN RSSI"
    );

    Serial.print(
        "TARGET  : "
    );

    Serial.println(
        targetBuffer.median
    );

    Serial.print(
        "ANCHOR2 : "
    );

    Serial.println(
        anchor2Buffer.median
    );

    Serial.print(
        "ANCHOR3 : "
    );

    Serial.println(
        anchor3Buffer.median
    );

    Serial.println(
        "=============================="
    );
}


// =====================================================
// SETUP
// =====================================================
void setup() {

    Serial.begin(
        115200
    );

    delay(
        3000
    );

    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "MASTER 3-RSSI TEST"
    );

    Serial.println(
        "================================"
    );


    // =================================================
    // SPI
    // =================================================
    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_NSS
    );

    Serial.println(
        "[SPI] OK"
    );


    // =================================================
    // LoRa
    // =================================================
    int state =
        radio.begin(
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

    Serial.print(
        "[LoRa begin] "
    );

    Serial.println(
        state
    );

    if (
        state !=
        RADIOLIB_ERR_NONE
    ) {

        Serial.println(
            "[FAIL] LoRa Init"
        );

        return;
    }

    radio.setDio2AsRfSwitch(
        true
    );

    radio.setPacketReceivedAction(
        setFlag
    );

    state =
        radio.startReceive();

    Serial.print(
        "[RX START] "
    );

    Serial.println(
        state
    );


    // =================================================
    // BLE
    // =================================================
    BLEDevice::init(
        "Master_Rescue_Node"
    );

    BLEServer* pServer =
        BLEDevice::createServer();

    pServer->setCallbacks(
        new ServerCallbacks()
    );

    BLEService* pService =
        pServer->createService(
            SERVICE_UUID
        );

    BLECharacteristic* pCmdChar =
        pService->createCharacteristic(
            CHAR_CMD_UUID,
            BLECharacteristic::PROPERTY_WRITE
        );

    pCmdChar->setCallbacks(
        new CommandCallbacks()
    );

    pDataChar =
        pService->createCharacteristic(
            CHAR_DATA_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY
        );

    pDataChar->addDescriptor(
        new BLE2902()
    );

    pService->start();

    BLEAdvertising* pAdvertising =
        BLEDevice::getAdvertising();

    pAdvertising->addServiceUUID(
        SERVICE_UUID
    );

    pAdvertising->start();

    Serial.println(
        "[BLE] READY"
    );

    Serial.println(
        "WAITING FOR TARGET / ANCHOR2 / ANCHOR3..."
    );
}


// =====================================================
// LOOP
// =====================================================

void loop() {

    // =================================================
    // 1초마다 HTML로 상태 전송
    // =================================================
    if (millis() - lastStatTime >= 1000) {

        lastStatTime = millis();

        sendStat();
    }

    // LoRa 패킷이 없으면 여기서 종료
    if (!receivedFlag) {
        return;
    }
    receivedFlag = false;

    String msg;

    int state =
        radio.readData(
            msg
        );

    if (
        state !=
        RADIOLIB_ERR_NONE
    ) {

        Serial.print(
            "[RX ERROR] "
        );

        Serial.println(
            state
        );

        radio.startReceive();

        return;
    }

    // =================================================
    // 마스터가 실제로 받은 패킷 RSSI
    // =================================================
    int directRssi =
        (int)radio.getRSSI();

    float snr =
        radio.getSNR();

    Serial.println();
    Serial.println(
        "------------------------------"
    );

    Serial.print(
        "DATA : "
    );

    Serial.println(
        msg
    );

    Serial.print(
        "DIRECT RSSI : "
    );

    Serial.println(
        directRssi
    );

    Serial.print(
        "SNR : "
    );

    Serial.println(
        snr
    );


    // =================================================
    // 패킷 종류 구분
    // =================================================
    if (
        msg.startsWith(
            "TARGET:PING"
        )
    ) {

        processTarget(
            msg,
            directRssi
        );
    }

    else if (
        msg.startsWith(
            "ANCHOR2:"
        )
    ) {

        processAnchor2(
            msg
        );
    }

    else if (
        msg.startsWith(
            "ANCHOR3:"
        )
    ) {

        processAnchor3(
            msg
        );
    }

    else {

        Serial.println(
            "[UNKNOWN PACKET]"
        );
    }

    Serial.println(
        "------------------------------"
    );


    // =================================================
    // 현재 중앙값 확인
    // =================================================
    printAllMedian();


    // =================================================
    // BLE로 원본 패킷 전달
    // =================================================
    sendBle(
        msg
    );


    // =================================================
    // 다시 수신모드
    // =================================================
    int rxState =
        radio.startReceive();

    if (
        rxState !=
        RADIOLIB_ERR_NONE
    ) {

        Serial.print(
            "[RX RESTART FAIL] "
        );

        Serial.println(
            rxState
        );
    }
}