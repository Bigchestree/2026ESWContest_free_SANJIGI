#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <algorithm>
#include <math.h>

// =====================================================
// SANJIGI MASTER NODE
// HTML BLE protocol exact match
// =====================================================

// ---------- Wio-SX1262 B2B ----------
#define LORA_NSS   41
#define LORA_BUSY  40
#define LORA_NRST  42
#define LORA_DIO1  39
#define LORA_SCK    7
#define LORA_MISO   8
#define LORA_MOSI   9

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSPI);
bool loraOK = false;

// ---------- Anchor XY ----------
const float AX = 0.0f,  AY = 0.0f;
const float BX = 80.0f, BY = 0.0f;
const float CX = 40.0f, CY = 69.0f;

// ---------- BLE UUID: exact match with dashboard ----------
#define SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_DATA_UUID "a3c17822-1d5b-4176-a447-0624916a0487"

BLECharacteristic* pDataChar = nullptr;
bool deviceConnected = false;

Preferences prefs;

float modelA = -40.0f;
float modelN = 2.8f;

// =====================================================
// RSSI buffer
// =====================================================
struct RssiBuffer {
  int buf[5] = {0};
  int head = 0;
  int count = 0;
  unsigned long lastRx = 0;

  void add(int rssi) {
    if (rssi > -10 || rssi < -140) return;
    buf[head] = rssi;
    head = (head + 1) % 5;
    if (count < 5) count++;
    lastRx = millis();
  }

  bool ready() const { return count >= 5; }
  bool alive() const { return count > 0 && (millis() - lastRx < 5000); }

  int median() const {
    if (!ready()) return -999;
    int t[5];
    for (int i = 0; i < 5; i++) t[i] = buf[i];
    std::sort(t, t + 5);
    return t[2];
  }

  float sigma() const {
    if (count < 2) return 20.0f;
    float mean = 0.0f;
    for (int i = 0; i < count; i++) mean += buf[i];
    mean /= count;

    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
      float d = buf[i] - mean;
      sum += d * d;
    }
    return sqrtf(sum / (count - 1));
  }
};

RssiBuffer rssiA, rssiB, rssiC;

// =====================================================
// Pressure buffer
// =====================================================
struct PressureBuffer {
  float buf[10] = {0};
  int head = 0;
  int count = 0;
  unsigned long lastRx = 0;

  void add(float p) {
    if (!isfinite(p) || p < 300.0f || p > 1100.0f) return;
    buf[head] = p;
    head = (head + 1) % 10;
    if (count < 10) count++;
    lastRx = millis();
  }

  bool fresh() const {
    return count > 0 && (millis() - lastRx < 5000);
  }

  float median() const {
    if (count == 0) return NAN;
    float t[10];
    for (int i = 0; i < count; i++) t[i] = buf[i];
    std::sort(t, t + count);
    if (count % 2 == 1) return t[count / 2];
    return (t[count / 2 - 1] + t[count / 2]) * 0.5f;
  }
};

PressureBuffer pressureBuf;
float basePressA = NAN;
float basePressB = NAN;
float basePressC = NAN;

// =====================================================
// BLE send
// =====================================================
void sendBle(const String& msg) {
  Serial.printf("[BLE TX] %s\n", msg.c_str());

  if (deviceConnected && pDataChar) {
    pDataChar->setValue(msg.c_str());
    pDataChar->notify();
    delay(15);
  }
}

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

  int start = idx + 6;
  int end = msg.indexOf(',', start);
  String s = (end >= 0) ? msg.substring(start, end) : msg.substring(start);

  float p = s.toFloat();
  return validPressure(p) ? p : NAN;
}

int parseRelayRssi(const String& msg) {
  int idx = msg.indexOf("RSSI:");
  if (idx < 0) return -999;

  int start = idx + 5;
  int end = msg.indexOf(',', start);
  String s = (end >= 0) ? msg.substring(start, end) : msg.substring(start);

  int rssi = s.toInt();
  if (rssi > -10 || rssi < -140) return -999;
  return rssi;
}

float relativeHeight(float targetP, float baseP) {
  if (!validPressure(targetP) || !validPressure(baseP)) return NAN;
  return 44330.0f *
         (1.0f - powf(targetP / baseP, 0.19029495f));
}

float rssiToDistance(int rssi) {
  return powf(10.0f,
              (modelA - (float)rssi) /
              (10.0f * modelN));
}

float rssiWeight(const RssiBuffer& b) {
  float s = b.sigma();
  float w = 1.0f / (s * s + 4.0f);
  if (w < 0.01f) w = 0.01f;
  if (w > 0.25f) w = 0.25f;
  return w;
}

// =====================================================
// NVS load / save
// =====================================================
void loadSettings() {
  prefs.begin("sanjigi", false);

  basePressA = prefs.getFloat("pressA", NAN);
  basePressB = prefs.getFloat("pressB", NAN);
  basePressC = prefs.getFloat("pressC", NAN);

  modelA = prefs.getFloat("modelA", -40.0f);
  modelN = prefs.getFloat("modelN", 2.8f);

  if (!isfinite(modelA) || modelA < -100 || modelA > -10) modelA = -40.0f;
  if (!isfinite(modelN) || modelN < 1 || modelN > 6) modelN = 2.8f;
}

void sendBase() {
  sendBle(
    "BASE:" +
    String(validPressure(basePressA) ? basePressA : -1.0f, 2) + "," +
    String(validPressure(basePressB) ? basePressB : -1.0f, 2) + "," +
    String(validPressure(basePressC) ? basePressC : -1.0f, 2)
  );
}

void saveBase(char which, float p) {
  if (!validPressure(p)) {
    sendBle("ERR:PRESS_INVALID");
    return;
  }

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

void sendModel() {
  sendBle("MODEL:" + String(modelA, 2) + "," + String(modelN, 3));
}

void setModel(float a, float n) {
  if (!isfinite(a) || a < -100 || a > -10 ||
      !isfinite(n) || n < 1 || n > 6) {
    sendBle("ERR:MODEL_RANGE");
    return;
  }

  modelA = a;
  modelN = n;

  prefs.putFloat("modelA", modelA);
  prefs.putFloat("modelN", modelN);

  // Dashboard handles MODELOK:
  sendBle("MODELOK:" + String(modelA, 2) + "," + String(modelN, 3));
}

// =====================================================
// LoRa packet parser
// =====================================================
void parsePacket(const String& msg, int directRssi) {
  Serial.printf("[LORA RX] %s | directRSSI=%d\n",
                msg.c_str(), directRssi);

  if (msg.startsWith("TARGET:PING")) {
    rssiA.add(directRssi);

    float p = parsePressure(msg);
    if (validPressure(p)) pressureBuf.add(p);
    return;
  }

  if (msg.startsWith("ANCHOR2:")) {
    int rssi = parseRelayRssi(msg);
    if (rssi != -999) rssiB.add(rssi);

    float p = parsePressure(msg);
    if (validPressure(p)) pressureBuf.add(p);
    return;
  }

  if (msg.startsWith("ANCHOR3:")) {
    int rssi = parseRelayRssi(msg);
    if (rssi != -999) rssiC.add(rssi);

    float p = parsePressure(msg);
    if (validPressure(p)) pressureBuf.add(p);
    return;
  }
}

// =====================================================
// Weighted least squares
// =====================================================
bool solveWLS(const float d[3],
              const float w[3],
              float& x,
              float& y) {

  const float px[3] = {AX, BX, CX};
  const float py[3] = {AY, BY, CY};

  x = (AX + BX + CX) / 3.0f;
  y = (AY + BY + CY) / 3.0f;

  for (int iter = 0; iter < 20; iter++) {

    float h00 = 0, h01 = 0, h11 = 0;
    float g0 = 0, g1 = 0;

    for (int i = 0; i < 3; i++) {

      float dx = x - px[i];
      float dy = y - py[i];

      float pred = sqrtf(dx * dx + dy * dy);
      if (pred < 0.01f) pred = 0.01f;

      float residual = pred - d[i];

      float jx = dx / pred;
      float jy = dy / pred;

      h00 += w[i] * jx * jx;
      h01 += w[i] * jx * jy;
      h11 += w[i] * jy * jy;

      g0 += w[i] * jx * residual;
      g1 += w[i] * jy * residual;
    }

    float det = h00 * h11 - h01 * h01;

    if (fabsf(det) < 1e-8f) {
      return false;
    }

    float stepX = -( h11 * g0 - h01 * g1) / det;
    float stepY = -(-h01 * g0 + h00 * g1) / det;

    float step = sqrtf(stepX * stepX + stepY * stepY);

    if (step > 30.0f) {
      float k = 30.0f / step;
      stepX *= k;
      stepY *= k;
    }

    x += stepX;
    y += stepY;

    if (!isfinite(x) || !isfinite(y)) {
      return false;
    }

    if (sqrtf(stepX * stepX + stepY * stepY) < 0.01f) {
      break;
    }
  }

  return true;
}

// =====================================================
// MEASURE
//
// Exact Dashboard output:
// RES:X,Y,Z,d1,d2,d3,zMode
// =====================================================
void runSnapshot() {

  Serial.println("\n========== MEASURE ==========");

  if (!rssiA.ready() || !rssiB.ready() || !rssiC.ready()) {
    sendBle(
      "ERR:SAMPLE," +
      String(rssiA.count) + "/5," +
      String(rssiB.count) + "/5," +
      String(rssiC.count) + "/5"
    );
    return;
  }

  if (!rssiA.alive() || !rssiB.alive() || !rssiC.alive()) {
    sendBle("ERR:STALE");
    return;
  }

  int medianRssi[3] = {
    rssiA.median(),
    rssiB.median(),
    rssiC.median()
  };

  float d3d[3] = {
    rssiToDistance(medianRssi[0]),
    rssiToDistance(medianRssi[1]),
    rssiToDistance(medianRssi[2])
  };

  // Default: XY uses RSSI ranges directly.
  float dxy[3] = {
    d3d[0],
    d3d[1],
    d3d[2]
  };

  float p = pressureBuf.median();
  float dz[3] = {NAN, NAN, NAN};

  bool zCorrection = false;

  // Apply PRESS correction only when valid.
  // If vertical difference is larger than RSSI distance,
  // do NOT collapse dxy to 0.1m. Just disable Z correction.
  if (pressureBuf.fresh() &&
      validPressure(p) &&
      allBasesReady()) {

    dz[0] = relativeHeight(p, basePressA);
    dz[1] = relativeHeight(p, basePressB);
    dz[2] = relativeHeight(p, basePressC);

    bool geometryOK = true;

    for (int i = 0; i < 3; i++) {
      if (!isfinite(dz[i]) ||
          d3d[i] * d3d[i] <= dz[i] * dz[i]) {
        geometryOK = false;
      }
    }

    if (geometryOK) {
      for (int i = 0; i < 3; i++) {
        dxy[i] = sqrtf(
          d3d[i] * d3d[i] -
          dz[i] * dz[i]
        );
      }
      zCorrection = true;
    }
  }

  float weights[3] = {
    rssiWeight(rssiA),
    rssiWeight(rssiB),
    rssiWeight(rssiC)
  };

  float x = 0.0f;
  float y = 0.0f;

  if (!solveWLS(dxy, weights, x, y)) {
    sendBle("ERR:WLS");
    return;
  }

  float z =
    (zCorrection && isfinite(dz[0]))
    ? dz[0]
    : 0.0f;

  Serial.printf(
    "RSSI A=%d B=%d C=%d\n",
    medianRssi[0],
    medianRssi[1],
    medianRssi[2]
  );

  Serial.printf(
    "DIST d1=%.2f d2=%.2f d3=%.2f\n",
    dxy[0],
    dxy[1],
    dxy[2]
  );

  Serial.printf(
    "RESULT X=%.2f Y=%.2f Z=%.2f ZMODE=%d\n",
    x, y, z, zCorrection ? 1 : 0
  );

  String result =
    "RES:" +
    String(x, 2) + "," +
    String(y, 2) + "," +
    String(z, 2) + "," +
    String(dxy[0], 2) + "," +
    String(dxy[1], 2) + "," +
    String(dxy[2], 2) + "," +
    String(zCorrection ? 1 : 0);

  sendBle(result);

  if (!zCorrection) {
    sendBle("WARN:ZCORR_OFF");
  }

  Serial.println("=============================\n");
}

// =====================================================
// BLE callbacks
// =====================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    Serial.println("[BLE] CONNECTED");
  }

  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    Serial.println("[BLE] DISCONNECTED");
    BLEDevice::startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {

    String cmd = pChar->getValue().c_str();
    cmd.trim();

    Serial.printf("[BLE RX CMD] %s\n", cmd.c_str());

    if (cmd == "MEASURE" || cmd == "1") {
      runSnapshot();
      return;
    }

    if (cmd == "GETBASE") {
      sendBase();
      return;
    }

    if (cmd == "CAL:A" ||
        cmd == "CAL:B" ||
        cmd == "CAL:C") {

      if (!pressureBuf.fresh() ||
          pressureBuf.count < 5) {

        sendBle(
          "ERR:PRESS_SAMPLE," +
          String(pressureBuf.count) +
          "/5"
        );
        return;
      }

      saveBase(
        cmd.charAt(4),
        pressureBuf.median()
      );
      return;
    }

    if (cmd == "GETMODEL") {
      sendModel();
      return;
    }

    if (cmd.startsWith("SETMODEL:")) {

      String values = cmd.substring(9);
      int comma = values.indexOf(',');

      if (comma < 0) {
        sendBle("ERR:MODEL_FORMAT");
        return;
      }

      float a =
        values.substring(0, comma).toFloat();

      float n =
        values.substring(comma + 1).toFloat();

      setModel(a, n);
      return;
    }

    sendBle("ERR:CMD");
  }
};

// =====================================================
// Status
//
// Dashboard reads:
// STAT:aliveA,aliveB,aliveC,rssiA,rssiB,rssiC,currentPress
// =====================================================
unsigned long lastStatus = 0;

void sendStatus() {

  if (millis() - lastStatus < 1000) {
    return;
  }

  lastStatus = millis();

  float p = pressureBuf.median();

  String stat =
    "STAT:" +
    String(rssiA.alive() ? 1 : 0) + "," +
    String(rssiB.alive() ? 1 : 0) + "," +
    String(rssiC.alive() ? 1 : 0) + "," +
    String(rssiA.median()) + "," +
    String(rssiB.median()) + "," +
    String(rssiC.median()) + "," +
    String(
      pressureBuf.fresh() && validPressure(p)
      ? p
      : -1.0f,
      2
    );

  sendBle(stat);
}

// =====================================================
// Setup
// =====================================================
void setup() {

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" SANJIGI MASTER BLE PROTOCOL FIXED");
  Serial.println("====================================");

  loadSettings();

  pinMode(LORA_NRST, OUTPUT);

  digitalWrite(LORA_NRST, LOW);
  delay(20);

  digitalWrite(LORA_NRST, HIGH);
  delay(100);

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
    1.6,
    true
  );

  if (state == RADIOLIB_ERR_NONE) {

    radio.setDio2AsRfSwitch(true);
    radio.startReceive();

    loraOK = true;

    Serial.println("[LORA] OK");

  } else {

    Serial.printf(
      "[LORA] FAIL code=%d\n",
      state
    );
  }

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

  Serial.println("[BLE] READY");
}

// =====================================================
// Loop
// =====================================================
void loop() {

  if (loraOK) {

    String packet;

    int state =
      radio.readData(packet);

    if (state == RADIOLIB_ERR_NONE) {

      int directRssi =
        (int)lroundf(
          radio.getRSSI()
        );

      parsePacket(
        packet,
        directRssi
      );

      radio.startReceive();
    }
  }

  sendStatus();

  delay(2);
}
