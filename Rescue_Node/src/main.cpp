#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Wio-SX1262 핀 매핑
#define LORA_NSS   D1
#define LORA_BUSY  D2
#define LORA_NRST  D3
#define LORA_DIO1  D4

// BME280 I2C 핀 매핑 (XIAO ESP32-S3 기준)
#define BME_SDA    D4
#define BME_SCL    D5

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
Adafruit_BME280 bme;

bool bmeOK = false;
bool loraOK = false;

void setup() {
    Serial.begin(115200);
    
    // USB CDC 시리얼 인식 안정화 대기 (최대 3초)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000));
    delay(500);

    Serial.println("\n==========================================");
    Serial.println("   [+] Target Node Booting Sequence...   ");
    Serial.println("==========================================");

    // 1. I2C 및 BME280 초기화 테스트
    Wire.begin(BME_SDA, BME_SCL);
    if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
        bmeOK = true;
        Serial.println(" [OK] BME280 Sensor Ready!");
    } else {
        Serial.println(" [WARN] BME280 Not Found! Using Dummy Alt (100.0m)");
    }

    // 2. Wio-SX1262 LoRa 초기화 테스트
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);
    if (state == RADIOLIB_ERR_NONE) {
        loraOK = true;
        Serial.println(" [OK] LoRa Radio Init Success!");
    } else {
        Serial.printf(" [FAIL] LoRa Init Failed! Error Code: %d\n", state);
    }

    Serial.println("==========================================\n");
}

void loop() {
    float altitude = 100.0; // 기본 더미 고도값

    // 센서 정상 인식 시 실제 고도 읽기
    if (bmeOK) {
        altitude = bme.readAltitude(1013.25);
    }

    String payload = "TARGET:PING,ALT:" + String(altitude, 1);

    // LoRa 라디오가 정상이면 송신
    if (loraOK) {
        int txState = radio.transmit(payload);
        if (txState == RADIOLIB_ERR_NONE) {
            Serial.printf("[TX Success] Sent: %s\n", payload.c_str());
        } else {
            Serial.printf("[TX Fail] Code: %d\n", txState);
        }
    } else {
        Serial.println("[Waiting] LoRa module not initialized...");
    }

    delay(1000);
}