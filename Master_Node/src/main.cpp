#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <algorithm>
#include <Preferences.h>

// =====================================================
// 앵커 거리 기본값(m)
// 실제 값은 BLE SETDIST 명령으로 변경 + NVS 저장
// SETDIST:D12,D13,D23
// 예) SETDIST:20.0,23.0,18.0
// =====================================================
float D12 = 20.0f;   // Master  <-> Anchor2
float D13 = 23.0f;   // Master  <-> Anchor3
float D23 = 18.0f;   // Anchor2 <-> Anchor3

Preferences preferences;

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
    // RSSI 안정도 기반 가중치
    // 최근 5개가 안정적일수록 1.0에 가까움
    // 많이 흔들릴수록 자동으로 가중치 감소
    // -------------------------------------------------
    float getWeight() {

        if (count < 5) {
            return 0.2f;
        }

        float mean = 0.0f;

        for (int i = 0; i < 5; i++) {
            mean += data[i];
        }

        mean /= 5.0f;

        float variance = 0.0f;

        for (int i = 0; i < 5; i++) {
            float diff = data[i] - mean;
            variance += diff * diff;
        }

        variance /= 5.0f;

        // 분산이 0이면 1.0
        // 흔들림이 커질수록 0에 가까워짐
        float weight =
            1.0f /
            (1.0f + variance / 9.0f);

        // 한 앵커를 완전히 무시하지는 않도록 하한 설정
        if (weight < 0.10f) {
            weight = 0.10f;
        }

        return weight;
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
// 함수 사전 선언
// =====================================================
void sendBle(String msg);
float rssiToDistance(int rssi);
bool calculateAnchorCoordinates();
bool trilaterate2D(float d1, float d2, float d3, float &x, float &y);
bool weightedLeastSquares2D(
    float d1, float d2, float d3,
    float w1, float w2, float w3,
    float &x, float &y
);


// =====================================================
// 앵커 간 실제 거리 입력(m)
//
// 현장에서 걸음/줄로 재서 이 3개만 수정
// D12 = Master <-> Anchor2
// D13 = Master <-> Anchor3
// D23 = Anchor2 <-> Anchor3
// =====================================================



// =====================================================
// 자동 계산되는 앵커 좌표
//
// Master  = (0, 0)
// Anchor2 = (D12, 0)
// Anchor3 = (anchor3X, anchor3Y)
// =====================================================
float anchor3X = 0.0f;
float anchor3Y = 0.0f;


// =====================================================
// 앵커 좌표 자동 계산
// 성공 true / 삼각형 성립 안 하면 false
// =====================================================
bool calculateAnchorCoordinates() {

    // 삼각형 성립 조건
    if (
        D12 <= 0.0f ||
        D13 <= 0.0f ||
        D23 <= 0.0f
    ) {
        return false;
    }

    if (
        D12 + D13 <= D23 ||
        D12 + D23 <= D13 ||
        D13 + D23 <= D12
    ) {
        return false;
    }

    anchor3X =
        (
            D12 * D12 +
            D13 * D13 -
            D23 * D23
        )
        /
        (
            2.0f * D12
        );

    float ySquared =
        D13 * D13 -
        anchor3X * anchor3X;

    if (ySquared <= 0.0f) {
        return false;
    }

    anchor3Y =
        sqrt(ySquared);

    return true;
}


// =====================================================
// RSSI -> 거리(m)
//
// 현재 테스트값
// A = 1m 기준 RSSI
// n = 환경 감쇠계수
// =====================================================
float rssiToDistance(int rssi) {

    const int A = -40;
    const float n = 2.8f;

    return pow(
        10.0f,
        (float)(A - rssi) /
        (10.0f * n)
    );
}


// =====================================================
// 2D 삼변측량
// =====================================================
bool trilaterate2D(
    float d1,
    float d2,
    float d3,
    float &x,
    float &y
) {

    // Anchor1 = Master = (0, 0)
    const float ax = 0.0f;
    const float ay = 0.0f;

    // Anchor2 = (D12, 0)
    const float bx = D12;
    const float by = 0.0f;

    // Anchor3 = 자동 계산 좌표
    const float cx = anchor3X;
    const float cy = anchor3Y;

    float a11 = 2.0f * (bx - ax);
    float a12 = 2.0f * (by - ay);

    float a21 = 2.0f * (cx - ax);
    float a22 = 2.0f * (cy - ay);

    float b1 =
        d1 * d1 -
        d2 * d2 +
        bx * bx +
        by * by -
        ax * ax -
        ay * ay;

    float b2 =
        d1 * d1 -
        d3 * d3 +
        cx * cx +
        cy * cy -
        ax * ax -
        ay * ay;

    float det =
        a11 * a22 -
        a12 * a21;

    if (fabs(det) < 0.0001f) {
        return false;
    }

    x =
        (
            b1 * a22 -
            a12 * b2
        ) / det;

    y =
        (
            a11 * b2 -
            b1 * a21
        ) / det;

    return true;
}


// =====================================================
// 가중 최소제곱 2D 위치 추정
//
// 기존 삼변측량 결과를 시작점으로 사용하고,
// RSSI 거리 3개와 가장 잘 맞는 X/Y를 반복 계산.
//
// 안정적인 앵커 = 큰 가중치
// 흔들리는 앵커 = 작은 가중치
// =====================================================
bool weightedLeastSquares2D(
    float d1,
    float d2,
    float d3,
    float w1,
    float w2,
    float w3,
    float &x,
    float &y
) {

    const float ax[3] = {
        0.0f,
        D12,
        anchor3X
    };

    const float ay[3] = {
        0.0f,
        0.0f,
        anchor3Y
    };

    const float measured[3] = {
        d1,
        d2,
        d3
    };

    const float weight[3] = {
        w1,
        w2,
        w3
    };

    // -------------------------------------------------
    // 시작점:
    // 기존 삼변측량 결과를 먼저 사용
    // 실패하면 앵커 삼각형 중심에서 시작
    // -------------------------------------------------
    if (!trilaterate2D(d1, d2, d3, x, y)) {

        x =
            (
                ax[0] +
                ax[1] +
                ax[2]
            ) / 3.0f;

        y =
            (
                ay[0] +
                ay[1] +
                ay[2]
            ) / 3.0f;
    }

    // -------------------------------------------------
    // Gauss-Newton 반복
    // -------------------------------------------------
    for (int iter = 0; iter < 12; iter++) {

        float h11 = 0.0f;
        float h12 = 0.0f;
        float h22 = 0.0f;

        float g1 = 0.0f;
        float g2 = 0.0f;

        for (int i = 0; i < 3; i++) {

            float dx = x - ax[i];
            float dy = y - ay[i];

            float predicted =
                sqrtf(
                    dx * dx +
                    dy * dy
                );

            if (predicted < 0.001f) {
                predicted = 0.001f;
            }

            // 잔차 = 예측거리 - RSSI 측정거리
            float residual =
                predicted -
                measured[i];

            float jx =
                dx /
                predicted;

            float jy =
                dy /
                predicted;

            float w =
                weight[i];

            h11 +=
                w *
                jx *
                jx;

            h12 +=
                w *
                jx *
                jy;

            h22 +=
                w *
                jy *
                jy;

            g1 +=
                w *
                jx *
                residual;

            g2 +=
                w *
                jy *
                residual;
        }

        float det =
            h11 * h22 -
            h12 * h12;

        if (fabsf(det) < 0.000001f) {
            return false;
        }

        float stepX =
            -(
                h22 * g1 -
                h12 * g2
            ) / det;

        float stepY =
            -(
                -h12 * g1 +
                h11 * g2
            ) / det;

        // 너무 큰 한 번의 점프 방지
        const float MAX_STEP = 5.0f;

        if (stepX > MAX_STEP) stepX = MAX_STEP;
        if (stepX < -MAX_STEP) stepX = -MAX_STEP;

        if (stepY > MAX_STEP) stepY = MAX_STEP;
        if (stepY < -MAX_STEP) stepY = -MAX_STEP;

        x += stepX;
        y += stepY;

        // 충분히 수렴하면 종료
        if (
            fabsf(stepX) < 0.001f &&
            fabsf(stepY) < 0.001f
        ) {
            break;
        }
    }

    if (
        isnan(x) ||
        isnan(y) ||
        isinf(x) ||
        isinf(y)
    ) {
        return false;
    }

    return true;
}


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
    // 앵커 거리 설정 명령
    // 형식: SETDIST:D12,D13,D23
    // 예) SETDIST:20.0,23.0,18.0
    // =================================================
    if (value.startsWith("SETDIST:")) {

        String data = value.substring(8);

        int comma1 = data.indexOf(',');
        int comma2 = data.indexOf(',', comma1 + 1);

        if (comma1 <= 0 || comma2 <= comma1 + 1) {
            Serial.println("[SETDIST FAIL] FORMAT");
            sendBle("ERR:SETDIST_FORMAT");
            return;
        }

        float newD12 = data.substring(0, comma1).toFloat();
        float newD13 = data.substring(comma1 + 1, comma2).toFloat();
        float newD23 = data.substring(comma2 + 1).toFloat();

        // 먼저 임시값으로 삼각형 성립 여부 확인
        float oldD12 = D12;
        float oldD13 = D13;
        float oldD23 = D23;

        D12 = newD12;
        D13 = newD13;
        D23 = newD23;

        if (!calculateAnchorCoordinates()) {
            D12 = oldD12;
            D13 = oldD13;
            D23 = oldD23;
            calculateAnchorCoordinates();

            Serial.println("[SETDIST FAIL] INVALID TRIANGLE");
            sendBle("ERR:SETDIST_INVALID");
            return;
        }

        // 유효한 값만 ESP32 NVS에 저장
        preferences.putFloat("d12", D12);
        preferences.putFloat("d13", D13);
        preferences.putFloat("d23", D23);

        Serial.println("[SETDIST OK]");
        Serial.print("D12 = ");
        Serial.println(D12, 2);
        Serial.print("D13 = ");
        Serial.println(D13, 2);
        Serial.print("D23 = ");
        Serial.println(D23, 2);

        String reply =
            "DIST:"
            + String(D12, 2) + ","
            + String(D13, 2) + ","
            + String(D23, 2);

        sendBle(reply);
        return;
    }

    // =================================================
    // 측정 명령
    // =================================================
    if (value == "MEASURE") {

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

            int rssi1 = targetBuffer.median;
            int rssi2 = anchor2Buffer.median;
            int rssi3 = anchor3Buffer.median;

            float d1 = rssiToDistance(rssi1);
            float d2 = rssiToDistance(rssi2);
            float d3 = rssiToDistance(rssi3);

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

            float x = 0.0f;
            float y = 0.0f;

            // =========================================
            // 앵커별 RSSI 안정도 가중치
            // =========================================
            float w1 = targetBuffer.getWeight();
            float w2 = anchor2Buffer.getWeight();
            float w3 = anchor3Buffer.getWeight();

            Serial.println("------------------------------");
            Serial.print("MASTER  WEIGHT : ");
            Serial.println(w1, 3);

            Serial.print("ANCHOR2 WEIGHT : ");
            Serial.println(w2, 3);

            Serial.print("ANCHOR3 WEIGHT : ");
            Serial.println(w3, 3);

            // =========================================
            // 가중 최소제곱 위치 계산
            // =========================================
            bool triOK =
                weightedLeastSquares2D(
                    d1,
                    d2,
                    d3,
                    w1,
                    w2,
                    w3,
                    x,
                    y
                );

            // WLS가 실패하면 기존 삼변측량으로 자동 복귀
            if (!triOK) {

                Serial.println(
                    "[WLS FAIL] 기존 삼변측량으로 fallback"
                );

                triOK =
                    trilaterate2D(
                        d1,
                        d2,
                        d3,
                        x,
                        y
                    );
            }
            else {
                Serial.println(
                    "[WLS OK] 가중 최소제곱 적용"
                );
            }

            if (triOK) {

                float z = targetAltitude;

                Serial.println("------------------------------");

                Serial.print("X            : ");
                Serial.print(x, 2);
                Serial.println(" m");

                Serial.print("Y            : ");
                Serial.print(y, 2);
                Serial.println(" m");

                Serial.print("Z(ALT)       : ");
                Serial.print(z, 2);
                Serial.println(" m");

                String result =
                    "RES:"
                    + String(x, 2) + ","
                    + String(y, 2) + ","
                    + String(z, 2) + ","
                    + String(d1, 2) + ","
                    + String(d2, 2) + ","
                    + String(d3, 2);

                sendBle(result);

                Serial.println("[MEASURE READY]");
            }

            else {

                Serial.println("[TRILATERATION FAIL]");

                sendBle(
                    "ERR:TRILATERATION_FAIL"
                );
            }
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
    // 저장된 앵커 거리 불러오기
    // 최초 실행이면 위 기본값 사용
    // =================================================
    preferences.begin("sanjigi", false);

    D12 = preferences.getFloat("d12", D12);
    D13 = preferences.getFloat("d13", D13);
    D23 = preferences.getFloat("d23", D23);

    Serial.println("[ANCHOR DISTANCES]");
    Serial.print("D12 = ");
    Serial.println(D12, 2);
    Serial.print("D13 = ");
    Serial.println(D13, 2);
    Serial.print("D23 = ");
    Serial.println(D23, 2);

    // =================================================
    // 앵커 좌표 자동 계산
    // =================================================
    bool anchorOK =
        calculateAnchorCoordinates();

    if (!anchorOK) {

        Serial.println(
            "[FAIL] ANCHOR DISTANCE INVALID"
        );

        Serial.println(
            "CHECK D12 / D13 / D23"
        );

        return;
    }

    Serial.println(
        "[ANCHOR COORDINATES]"
    );

    Serial.println(
        "MASTER  : (0.00, 0.00)"
    );

    Serial.print(
        "ANCHOR2 : ("
    );

    Serial.print(
        D12,
        2
    );

    Serial.println(
        ", 0.00)"
    );

    Serial.print(
        "ANCHOR3 : ("
    );

    Serial.print(
        anchor3X,
        2
    );

    Serial.print(
        ", "
    );

    Serial.print(
        anchor3Y,
        2
    );

    Serial.println(
        ")"
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