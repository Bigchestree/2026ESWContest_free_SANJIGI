#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// =====================================================
// SANJIGI TARGET NODE
// XIAO ESP32-S3 + Wio-SX1262 + BME280 1개
//
// BME280:
//   VCC -> 3V3
//   GND -> GND
//   SDA -> D4 / GPIO5
//   SCL -> D5 / GPIO6
//
// ALT 값은 "전원 켠 위치 = 0m" 기준 상대고도입니다.
// =====================================================

// ---------------- LoRa ----------------
#define LORA_NSS     41
#define LORA_BUSY    40
#define LORA_NRST    42
#define LORA_DIO1    39
#define LORA_SCK      7
#define LORA_MISO     8
#define LORA_MOSI     9
#define LORA_RF_SW   38

SPIClass loraSPI(FSPI);

SX1262 radio = new Module(
    LORA_NSS,
    LORA_DIO1,
    LORA_NRST,
    LORA_BUSY,
    loraSPI
);

// ---------------- BME280 ----------------
#define BME_SDA 5
#define BME_SCL 6

Adafruit_BME280 bme;

bool bmeOK = false;
float basePressurePa = 0.0f;

// =====================================================
// 시작 위치의 기압을 평균내서 Z=0 기준점 생성
// =====================================================
bool calibrateBasePressure() {

    const int SAMPLE_COUNT = 30;

    float sum = 0.0f;
    int validCount = 0;

    Serial.println();
    Serial.println("[BME] 상대고도 기준점 측정 중...");

    for (int i = 0; i < SAMPLE_COUNT; i++) {

        float p = bme.readPressure();

        if (!isnan(p) && p > 10000.0f) {
            sum += p;
            validCount++;
        }

        delay(100);
    }

    if (validCount < 10) {
        Serial.println("[BME ERROR] 기준 기압 측정 실패");
        return false;
    }

    basePressurePa = sum / validCount;

    Serial.print("[BME] 기준 기압 = ");
    Serial.print(basePressurePa / 100.0f, 2);
    Serial.println(" hPa");

    Serial.println("[BME] 현재 위치를 Z = 0.00m 로 설정");
    Serial.println();

    return true;
}

// =====================================================
// 현재 기압을 시작점 기준 상대고도(m)로 변환
//
// h = 44330 * (1 - (P/P0)^0.1903)
// =====================================================
float readRelativeAltitude() {

    if (!bmeOK || basePressurePa <= 0.0f) {
        return 0.0f;
    }

    float pressurePa = bme.readPressure();

    if (isnan(pressurePa) || pressurePa <= 10000.0f) {
        return 0.0f;
    }

    float ratio = pressurePa / basePressurePa;

    float altitude =
        44330.0f *
        (1.0f - powf(ratio, 0.19029495f));

    return altitude;
}

void setup() {

    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("======================================");
    Serial.println(" SANJIGI TARGET + BME280");
    Serial.println("======================================");

    // =================================================
    // BME280 초기화
    // =================================================
    Wire.begin(BME_SDA, BME_SCL);

    if (bme.begin(0x76, &Wire)) {
        bmeOK = true;
        Serial.println("[BME] 0x76 연결 성공");
    }
    else if (bme.begin(0x77, &Wire)) {
        bmeOK = true;
        Serial.println("[BME] 0x77 연결 성공");
    }
    else {
        Serial.println("[BME ERROR] 센서를 찾지 못했습니다.");
        Serial.println("ALT는 임시로 0.00m 전송합니다.");
    }

    if (bmeOK) {

        bme.setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X16,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::FILTER_X16,
            Adafruit_BME280::STANDBY_MS_500
        );

        if (!calibrateBasePressure()) {
            bmeOK = false;
        }
    }

    // =================================================
    // LoRa 초기화
    // =================================================
    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_NSS
    );

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

    if (state != RADIOLIB_ERR_NONE) {

        Serial.print("[LoRa ERROR] Init Failed: ");
        Serial.println(state);

        while (true) {
            delay(1000);
        }
    }

    radio.setDio2AsRfSwitch(true);

    Serial.println("[LoRa] Init Success");
    Serial.println("======================================");
    Serial.println();
}

void loop() {

    float altitude = readRelativeAltitude();

    String payload =
        "TARGET:PING,ALT:"
        + String(altitude, 2);

    int txState = radio.transmit(payload);

    if (txState == RADIOLIB_ERR_NONE) {

        Serial.print("[TX Success] ");
        Serial.println(payload);

        if (bmeOK) {
            Serial.print("  Temp : ");
            Serial.print(bme.readTemperature(), 1);
            Serial.println(" C");

            Serial.print("  Z    : ");
            Serial.print(altitude, 2);
            Serial.println(" m");
        }
    }
    else {

        Serial.print("[TX Fail] ");
        Serial.println(txState);
    }

    delay(1000);
}
