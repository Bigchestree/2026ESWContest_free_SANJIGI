#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <algorithm>
#include <math.h>

// =====================================================
// SANJIGI MASTER NODE
// PRESS relative-height correction + RSSI median + WLS
// =====================================================

// ---------- Wio-SX1262 B2B pins ----------
#define LORA_NSS   41
#define LORA_BUSY  40
#define LORA_NRST  42
#define LORA_DIO1  39
#define LORA_SCK   7
#define LORA_MISO  8
#define LORA_MOSI  9

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);

// ---------- Anchor XY coordinates (meters) ----------
const float AX = 0.0f,  AY = 0.0f;
const float BX = 80.0f, BY = 0.0f;
const float CX = 40.0f, CY = 69.0f;

// ---------- BLE ----------
#define SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_DATA_UUID "a3c17822-1d5b-4176-a447-0624916a0487"

BLECharacteristic* pDataChar = nullptr;
bool deviceConnected = false;

// ---------- NVS ----------
Preferences prefs;

// =====================================================
// RSSI buffer
// =====================================================
struct RssiBuffer {
  int buf[5] = {0};
  int head = 0;
  int count = 0;
  unsigned long lastRxTime = 0;

  void add(int rssi) {
    if (rssi > -10 || rssi < -140) return;

    buf[head] = rssi;
    head = (head + 1) % 5;
    if (count < 5) count++;
    lastRxTime = millis();
  }

  bool ready() const {
    return count >= 5;
  }

  bool alive() const {
    return count > 0 && (millis() - lastRxTime) < 5000;
  }

  int median() const {
    if (!ready()) return -999;
    int temp[5];
    for (int i = 0; i < 5; i++) temp[i] = buf[i];
    std::sort(temp, temp + 5);
    return temp[2];
  }

  float sigma() const {
    if (count < 2) return 20.0f;

    float mean = 0;
    for (int i = 0; i < count; i++) mean += buf[i];
    mean /= count;

    float sum = 0;
    for (int i = 0; i < count; i++) {
      float d = buf[i] - mean;
      sum += d * d;
    }

    return sqrtf(sum / (count - 1));
  }
};

RssiBuffer rssiA, rssiB, rssiC;

// =====================================================
// Pressure rolling buffer
// =====================================================
struct PressureBuffer {
  float buf[10] = {0};
  int head = 0;
  int count = 0;
  unsigned long lastRxTime = 0;

  void add(float p) {
    if (!isfinite(p) || p < 300.0f || p > 1100.0f) return;

    buf[head] = p;
    head = (head + 1) % 10;
    if (count < 10) count++;
    lastRxTime = millis();
  }

  bool fresh() const {
    return count > 0 && (millis() - lastRxTime) < 5000;
  }

  float median() const {
    if (count == 0) return NAN;

    float temp[10];
    for (int i = 0; i < count; i++) temp[i] = buf[i];
    std::sort(temp, temp + count);

    if (count % 2 == 1) return temp[count / 2];
    return (temp[count / 2 - 1] + temp[count / 2]) * 0.5f;
  }
};

PressureBuffer pressureBuf;

// Saved reference pressure measured with THE SAME Target BMP280
// while Target was physically placed beside A / B / C.
float basePressA = NAN;
float basePressB = NAN;
float basePressC = NAN;

// =====================================================
// Helpers
// =====================================================
bool validPressure(float p) {
  return isfinite(p) && p >= 300.0f && p <= 1100.0f;
}

bool allBasesReady() {
  return validPressure(basePressA) &&
         validPressure(basePressB) &&
         validPressure(basePressC);
}

float parsePressure(const String& msg) {
  int idx = msg.indexOf("PRESS:");
  if (idx < 0) return NAN;

  String s = msg.substring(idx + 6);
  int comma = s.indexOf(',');
  if (comma >= 0) s = s.substring(0, comma);

  float p = s.toFloat();
  return validPressure(p) ? p : NAN;
}

// Relative altitude of Target with respect to anchor reference pressure.
// Positive = Target is higher than that anchor.
float relativeHeightMeters(float targetPressure, float anchorPressure) {
  if (!validPressure(targetPressure) || !validPressure(anchorPressure)) return NAN;

  return 44330.0f *
         (1.0f - powf(targetPressure / anchorPressure, 0.19029495f));
}

// Current RSSI-distance model.
// This MUST still be field-calibrated later.
float rssiToDistance3D(int rssi) {
  const float A = -40.0f; // RSSI at 1 m
  const float n = 2.8f;   // path-loss exponent
  return powf(10.0f, (A - (float)rssi) / (10.0f * n));
}

float rssiWeight(const RssiBuffer& b) {
  float s = b.sigma();

  // Lower RSSI variance -> larger weight.
  // Clamp to avoid one anchor dominating completely.
  float w = 1.0f / (s * s + 4.0f);
  if (w < 0.01f) w = 0.01f;
  if (w > 0.25f) w = 0.25f;
  return w;
}

void sendBle(const String& msg) {
  Serial.printf("[BLE TX] %s\n", msg.c_str());

  if (deviceConnected && pDataChar) {
    pDataChar->setValue(msg.c_str());
    pDataChar->notify();
  }
}

// =====================================================
// Save / load base pressures
// =====================================================
void loadBases() {
  prefs.begin("sanjigi", false);

  basePressA = prefs.getFloat("pressA", NAN);
  basePressB = prefs.getFloat("pressB", NAN);
  basePressC = prefs.getFloat("pressC", NAN);

  Serial.printf("[BASE] A=%.2f B=%.2f C=%.2f hPa\n",
                basePressA, basePressB, basePressC);
}

void saveBase(char which, float p) {
  if (!validPressure(p)) return;

  if (which == 'A') {
    basePressA = p;
    prefs.putFloat("pressA", p);
  } else if (which == 'B') {
    basePressB = p;
    prefs.putFloat("pressB", p);
  } else if (which == 'C') {
    basePressC = p;
    prefs.putFloat("pressC", p);
  }

  sendBle("CALOK:" + String(which) + "," + String(p, 2));
}

void clearBases() {
  basePressA = basePressB = basePressC = NAN;
  prefs.remove("pressA");
  prefs.remove("pressB");
  prefs.remove("pressC");
  sendBle("CALCLR");
}

void sendBaseStatus() {
  String msg = "BASE:" +
               String(validPressure(basePressA) ? basePressA : -1.0f, 2) + "," +
               String(validPressure(basePressB) ? basePressB : -1.0f, 2) + "," +
               String(validPressure(basePressC) ? basePressC : -1.0f, 2);
  sendBle(msg);
}

// =====================================================
// Packet parser
// =====================================================
void parseIncomingPacket(const String& msg, int directRssi) {
  if (msg.startsWith("TARGET:PING")) {
    rssiA.add(directRssi);

    float p = parsePressure(msg);
    if (validPressure(p)) pressureBuf.add(p);
    return;
  }

  if (msg.startsWith("ANCHOR2:")) {
    int idx = msg.indexOf("RSSI:");
    if (idx >= 0) {
      int end = msg.indexOf(',', idx);
      String rs = (end >= 0) ? msg.substring(idx + 5, end)
                             : msg.substring(idx + 5);
      rssiA.ready(); // no-op; keeps intent explicit
      rssiB.add(rs.toInt());
    }

    // Relay pressure may recover PRESS if direct Target packet was missed.
    float p = parsePressure(msg);
    if (validPressure(p)) pressureBuf.add(p);
    return;
  }

  if (msg.startsWith("ANCHOR3:")) {
    int idx = msg.indexOf("RSSI:");
    if (idx >= 0) {
      int end = msg.indexOf(',', idx);
      String rs = (end >= 0) ? msg.substring(idx + 5, end)
                             : msg.substring(idx + 5);
      rssiC.add(rs.toInt());
    }

    float p = parsePressure(msg);
    if (validPressure(p)) pressureBuf.add(p);
    return;
  }
}

// =====================================================
// Weighted nonlinear least squares in XY
// =====================================================
bool solveWLS(const float d[3], const float w[3], float& x, float& y) {
  const float px[3] = {AX, BX, CX};
  const float py[3] = {AY, BY, CY};

  // Start from triangle centroid.
  x = (AX + BX + CX) / 3.0f;
  y = (AY + BY + CY) / 3.0f;

  for (int iter = 0; iter < 12; iter++) {
    float h00 = 0, h01 = 0, h11 = 0;
    float g0 = 0, g1 = 0;

    for (int i = 0; i < 3; i++) {
      float dx = x - px[i];
      float dy = y - py[i];
      float ri = sqrtf(dx * dx + dy * dy);

      if (ri < 0.05f) ri = 0.05f;

      float residual = ri - d[i];
      float jx = dx / ri;
      float jy = dy / ri;

      h00 += w[i] * jx * jx;
      h01 += w[i] * jx * jy;
      h11 += w[i] * jy * jy;

      g0 += w[i] * jx * residual;
      g1 += w[i] * jy * residual;
    }

    float det = h00 * h11 - h01 * h01;
    if (fabsf(det) < 1e-6f) return false;

    float stepX = -( h11 * g0 - h01 * g1) / det;
    float stepY = -(-h01 * g0 + h00 * g1) / det;

    x += stepX;
    y += stepY;

    if (sqrtf(stepX * stepX + stepY * stepY) < 0.01f) break;
  }

  return isfinite(x) && isfinite(y);
}

// =====================================================
// Snapshot
// =====================================================
void runSnapshot() {
  if (!rssiA.ready() || !rssiB.ready() || !rssiC.ready()) {
    sendBle("ERR:RSSI_SAMPLE,A:" + String(rssiA.count) +
            ",B:" + String(rssiB.count) +
            ",C:" + String(rssiC.count));
    return;
  }

  int medA = rssiA.median();
  int medB = rssiB.median();
  int medC = rssiC.median();

  float d3d[3] = {
    rssiToDistance3D(medA),
    rssiToDistance3D(medB),
    rssiToDistance3D(medC)
  };

  float dxy[3] = {d3d[0], d3d[1], d3d[2]};
  float dz[3] = {NAN, NAN, NAN};

  bool zCorrection = false;
  float currentPress = pressureBuf.median();

  if (pressureBuf.fresh() && validPressure(currentPress) && allBasesReady()) {
    dz[0] = relativeHeightMeters(currentPress, basePressA);
    dz[1] = relativeHeightMeters(currentPress, basePressB);
    dz[2] = relativeHeightMeters(currentPress, basePressC);

    bool geometryOK = true;

    for (int i = 0; i < 3; i++) {
      if (!isfinite(dz[i]) || fabsf(dz[i]) >= d3d[i]) {
        geometryOK = false;
        break;
      }
    }

    if (geometryOK) {
      for (int i = 0; i < 3; i++) {
        float sq = d3d[i] * d3d[i] - dz[i] * dz[i];
        dxy[i] = sqrtf(max(sq, 0.25f));
      }
      zCorrection = true;
    }
  }

  float weights[3] = {
    rssiWeight(rssiA),
    rssiWeight(rssiB),
    rssiWeight(rssiC)
  };

  float x = 0, y = 0;

  if (!solveWLS(dxy, weights, x, y)) {
    sendBle("ERR:WLS");
    return;
  }

  // Z is reported relative to Anchor A reference.
  // Absolute sea-level altitude is intentionally not claimed here.
  float zRelA = (zCorrection && isfinite(dz[0])) ? dz[0] : NAN;

  String res = "RES:" +
               String(x, 2) + "," +
               String(y, 2) + "," +
               String(isfinite(zRelA) ? zRelA : 0.0f, 2) + "," +
               String(dxy[0], 2) + "," +
               String(dxy[1], 2) + "," +
               String(dxy[2], 2) + "," +
               String(zCorrection ? 1 : 0);

  sendBle(res);

  if (zCorrection) {
    sendBle("DZ:" + String(dz[0], 2) + "," +
                    String(dz[1], 2) + "," +
                    String(dz[2], 2));
  } else {
    sendBle("WARN:ZCORR_OFF");
  }
}

// =====================================================
// BLE callbacks
// =====================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = pChar->getValue().c_str();
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "MEASURE" || cmd == "1") {
      runSnapshot();
      return;
    }

    if (cmd == "GETBASE") {
      sendBaseStatus();
      return;
    }

    if (cmd == "CLRBASE") {
      clearBases();
      return;
    }

    if (cmd == "CAL:A" || cmd == "CAL:B" || cmd == "CAL:C") {
      // Require several recent pressure samples so a single noisy sample
      // cannot become the anchor reference.
      if (!pressureBuf.fresh() || pressureBuf.count < 5) {
        sendBle("ERR:PRESS_SAMPLE," + String(pressureBuf.count) + "/5");
        return;
      }

      float p = pressureBuf.median();
      if (!validPressure(p)) {
        sendBle("ERR:PRESS_INVALID");
        return;
      }

      saveBase(cmd.charAt(4), p);
      return;
    }

    sendBle("ERR:CMD");
  }
};

// =====================================================
// Status
// =====================================================
unsigned long lastStatus = 0;

void sendStatus() {
  if (millis() - lastStatus < 1000) return;
  lastStatus = millis();

  float p = pressureBuf.median();

  String msg =
      "STAT:" +
      String(rssiA.alive() ? 1 : 0) + "," +
      String(rssiB.alive() ? 1 : 0) + "," +
      String(rssiC.alive() ? 1 : 0) + "," +
      String(rssiA.median()) + "," +
      String(rssiB.median()) + "," +
      String(rssiC.median()) + "," +
      String(pressureBuf.fresh() && validPressure(p) ? p : -1.0f, 2) + "," +
      String(allBasesReady() ? 1 : 0);

  sendBle(msg);
}

// =====================================================
// Setup / loop
// =====================================================
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
    radio.startReceive();
    Serial.println("[LORA] Master OK");
  } else {
    Serial.printf("[LORA] Master FAIL code=%d\n", state);
  }
}

void initBLE() {
  BLEDevice::init("Master_Rescue_Node");

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic* pCmdChar = pService->createCharacteristic(
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

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("[BLE] Ready");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== SANJIGI MASTER / PRESS + WLS ===");

  loadBases();
  initLoRa();
  initBLE();
}

void loop() {
  String msg;
  int state = radio.readData(msg);

  if (state == RADIOLIB_ERR_NONE) {
    int directRssi = (int)radio.getRSSI();
    parseIncomingPacket(msg, directRssi);
    radio.startReceive();
  }

  sendStatus();
}
