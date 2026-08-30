#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// =====================================================
// SANJIGI TARGET NODE
// XIAO ESP32-S3 + Wio-SX1262 B2B + BMP280
// =====================================================

// ---------- Wio-SX1262 B2B pins ----------
#define LORA_NSS   41
#define LORA_BUSY  40
#define LORA_NRST  42
#define LORA_DIO1  39
#define LORA_SCK   7
#define LORA_MISO  8
#define LORA_MOSI  9

// ---------- BMP280 I2C ----------
#define BMP_SDA      5
#define BMP_SCL      6
#define BMP_ADDRESS  0x76

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);
Adafruit_BMP280 bmp;

bool loraOK = false;
bool bmpOK = false;

bool validPressure(float p) {
  return isfinite(p) && p >= 300.0f && p <= 1100.0f;
}

void initBMP280() {
  Wire.begin(BMP_SDA, BMP_SCL);
  delay(300);

  bmpOK = bmp.begin(BMP_ADDRESS);
  if (!bmpOK) {
    Serial.println("[BMP] FAIL - BMP280 not found at 0x76");
    return;
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );

  Serial.println("[BMP] OK");
}

void initLoRa() {
  pinMode(LORA_NRST, OUTPUT);
  digitalWrite(LORA_NRST, LOW);
  delay(20);
  digitalWrite(LORA_NRST, HIGH);
  delay(100);

  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int state = radio.begin(
    923.0, 125.0, 9, 7, 0x12,
    10, 8, 1.6, true
  );

  if (state == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    loraOK = true;
    Serial.println("[LORA] OK");
  } else {
    Serial.printf("[LORA] FAIL code=%d\n", state);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== SANJIGI TARGET / PRESS MODE ===");
  initBMP280();
  initLoRa();
}

void loop() {
  if (!bmpOK) {
    Serial.println("[ERROR] BMP280 unavailable");
    delay(2000);
    return;
  }

  if (!loraOK) {
    Serial.println("[ERROR] LoRa unavailable");
    delay(2000);
    return;
  }

  float pressure = bmp.readPressure() / 100.0f; // hPa
  float temp = bmp.readTemperature();

  if (!validPressure(pressure)) {
    Serial.printf("[PRESS INVALID] %.2f hPa - TX skipped\n", pressure);
    delay(1000);
    return;
  }

  // New common packet format
  // TARGET:PING,PRESS:1002.35
  String payload = "TARGET:PING,PRESS:" + String(pressure, 2);

  int state = radio.transmit(payload);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[TX] %s, TEMP:%.2fC\n", payload.c_str(), temp);
  } else {
    Serial.printf("[TX FAIL] code=%d\n", state);
  }

  delay(1000);
}
