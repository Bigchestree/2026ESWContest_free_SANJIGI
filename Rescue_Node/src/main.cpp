#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// =====================================================
// LoRa : XIAO ESP32-S3 + Wio-SX1262 B2B
// =====================================================
#define LORA_NSS   41
#define LORA_BUSY  40
#define LORA_NRST  42
#define LORA_DIO1  39

#define LORA_SCK   7
#define LORA_MISO  8
#define LORA_MOSI  9

SPIClass loraSPI(FSPI);

SX1262 radio = new Module(
    LORA_NSS,
    LORA_DIO1,
    LORA_NRST,
    LORA_BUSY,
    loraSPI
);

// =====================================================
// BMP280
// =====================================================
#define BMP_SDA      5
#define BMP_SCL      6
#define BMP_ADDRESS  0x76

Adafruit_BMP280 bmp;

bool bmpOK = false;
bool loraOK = false;

// 기준 해면기압
// 나중에 실제 지역 기압으로 보정 가능
float SEA_LEVEL_HPA = 1013.25;

// =====================================================
// BMP280 초기화
// =====================================================
void initBMP280() {

    Serial.println();
    Serial.println("========== BMP280 INIT ==========");

    Wire.begin(BMP_SDA, BMP_SCL);
    delay(300);

    Serial.printf("SDA : GPIO%d\n", BMP_SDA);
    Serial.printf("SCL : GPIO%d\n", BMP_SCL);
    Serial.printf("ADDR: 0x%02X\n", BMP_ADDRESS);

    if (!bmp.begin(BMP_ADDRESS)) {

        Serial.println("[FAIL] BMP280 초기화 실패");
        bmpOK = false;
        return;
    }

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,     // 온도
        Adafruit_BMP280::SAMPLING_X16,    // 기압
        Adafruit_BMP280::FILTER_X16,      // 흔들림 완화
        Adafruit_BMP280::STANDBY_MS_500
    );

    bmpOK = true;

    Serial.println("[SUCCESS] BMP280 연결 성공!");
    Serial.println("=================================");
}

// =====================================================
// LoRa 초기화
// =====================================================
void initLoRa() {

    Serial.println();
    Serial.println("========== LORA INIT ==========");

    // SX1262 하드웨어 리셋
    pinMode(LORA_NRST, OUTPUT);

    digitalWrite(LORA_NRST, LOW);
    delay(20);

    digitalWrite(LORA_NRST, HIGH);
    delay(100);

    // 전용 SPI 시작
    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_NSS
    );

    // Wio-SX1262 B2B
    int state = radio.begin(
        923.0,      // MHz
        125.0,      // BW
        9,          // SF
        7,          // CR
        0x12,       // Sync word
        10,         // TX power
        8,          // Current limit
        1.6,        // TCXO voltage
        true        // LDO
    );

    if (state == RADIOLIB_ERR_NONE) {

        radio.setDio2AsRfSwitch(true);

        loraOK = true;

        Serial.println("[SUCCESS] SX1262 LoRa 연결 성공!");

    } else {

        loraOK = false;

        Serial.printf(
            "[FAIL] LoRa 초기화 실패 : %d\n",
            state
        );
    }

    Serial.println("===============================");
}

// =====================================================
// BMP280 측정
// =====================================================
float getAltitude() {

    if (!bmpOK)
        return 0.0;

    return bmp.readAltitude(SEA_LEVEL_HPA);
}

// =====================================================
// SETUP
// =====================================================
void setup() {

    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("     SANJIGI TARGET NODE - BMP280");
    Serial.println("==========================================");

    initBMP280();

    initLoRa();

    Serial.println();
    Serial.println("System Start!");
    Serial.println();
}

// =====================================================
// LOOP
// =====================================================
void loop() {

    // -----------------------------------------
    // BMP280가 죽어 있으면 전송하지 않음
    // -----------------------------------------
    if (!bmpOK) {

        Serial.println(
            "[ERROR] BMP280 연결 안 됨"
        );

        delay(2000);
        return;
    }

    // -----------------------------------------
    // LoRa가 죽어 있으면 전송하지 않음
    // -----------------------------------------
    if (!loraOK) {

        Serial.println(
            "[ERROR] LoRa 연결 안 됨"
        );

        delay(2000);
        return;
    }

    // -----------------------------------------
    // BMP280 측정
    // -----------------------------------------
    float temperature =
        bmp.readTemperature();

    float pressure =
        bmp.readPressure() / 100.0F;

    float altitude =
        getAltitude();

    Serial.println("--------------------------------");

    Serial.printf(
        "[BMP] TEMP : %.2f C\n",
        temperature
    );

    Serial.printf(
        "[BMP] PRESS: %.2f hPa\n",
        pressure
    );

    Serial.printf(
        "[BMP] ALT  : %.2f m\n",
        altitude
    );

    // -----------------------------------------
    // LoRa 패킷 생성
    //
    // TARGET:PING,ALT:123.4
    // -----------------------------------------
    String payload =
        "TARGET:PING,ALT:" +
        String(altitude, 1);

    Serial.printf(
        "[LoRa TX] %s\n",
        payload.c_str()
    );

    // -----------------------------------------
    // LoRa 송신
    // -----------------------------------------
    int state =
        radio.transmit(payload);

    if (state == RADIOLIB_ERR_NONE) {

        Serial.println(
            "[TX SUCCESS]"
        );

    } else {

        Serial.printf(
            "[TX FAIL] %d\n",
            state
        );
    }

    Serial.println("--------------------------------");

    delay(1000);
}