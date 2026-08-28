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

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
Adafruit_BME280 bme; // BME280 기압/온습도 센서

void setup() {
    Serial.begin(115200);
    delay(2000);
    Wire.begin(D4, D5);
    Serial.println("\n--- I2C Scanner Running ---");

    // 2. Wio-SX1262 LoRa 초기화 (923MHz 규격)
    int state = radio.begin(923.0, 125.0, 9, 7, 0x12, 10, 8);
    if (state != RADIOLIB_ERR_NONE) {
        while (true); // 초기화 실패 시 정지
    }
}

void loop() {
    byte error, address;
    int nDevices = 0;

    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("I2C 장치 발견! 주소: 0x%02X\n", address);
            nDevices++;
        }
    }

    if (nDevices == 0) {
        Serial.println("연결된 I2C 장치를 찾을 수 없습니다.");
    }
    
    delay(3000);
    
    // 해수면 표준 기압(1013.25 hPa) 기준 현재 고도(m) 측정
    float altitude = bme.readAltitude(1013.25);

    // 핑 패킷 생성 (예: "TARGET:PING,ALT:120.5")
    String payload = "TARGET:PING,ALT:" + String(altitude, 1);

    // 1초 간격 브로드캐스트 전송
    radio.transmit(payload);
    delay(1000);
}
