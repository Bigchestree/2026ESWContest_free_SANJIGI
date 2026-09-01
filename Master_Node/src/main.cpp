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
// + BLE anchor distance input (AB / AC / BC)
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

volatile bool loraPacketReceived = false;

void IRAM_ATTR onLoraPacketReceived() {
  loraPacketReceived = true;
}

// ---------- Anchor XY coordinates (meters) ----------
// A(Master) is fixed at (0,0).
// B is placed on +X axis at (AB, 0).
// C is calculated from AB / AC / BC.
const float AX = 0.0f;
const float AY = 0.0f;

float BX = NAN;
float BY = 0.0f;
float CX = NAN;
float CY = NAN;

float distAB = NAN;  // Master(A) <-> Anchor2(B)
float distAC = NAN;  // Master(A) <-> Anchor3(C)
float distBC = NAN;  // Anchor2(B) <-> Anchor3(C)

// ---------- BLE ----------
#define SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_DATA_UUID "a3c17822-1d5b-4176-a447-0624916a0487"

BLECharacteristic* pDataChar = nullptr;
bool deviceConnected = false;
volatile bool restartAdvertising = false;
unsigned long bleDisconnectedAt = 0;

// ---------- NVS ----------
Preferences prefs;

// ---------- RSSI distance model (editable via BLE / saved in NVS) ----------
float modelA = -40.0f;  // RSSI at 1 m
float modelN = 2.8f;    // path-loss exponent

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
float rssiToDistance3D(int rssi) {
  return powf(10.0f, (modelA - (float)rssi) / (10.0f * modelN));
}

float rssiWeight(const RssiBuffer& b) {
  float s = b.sigma();

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
// RSSI model A / n save-load
// =====================================================
void loadRssiModel() {
  modelA = prefs.getFloat("modelA", -40.0f);
  modelN = prefs.getFloat("modelN", 2.8f);

  if (!isfinite(modelA) || modelA > -10.0f || modelA < -120.0f) {
    modelA = -40.0f;
  }
  if (!isfinite(modelN) || modelN < 1.0f || modelN > 8.0f) {
    modelN = 2.8f;
  }

  Serial.printf("[RSSI MODEL] A=%.2f n=%.3f\n", modelA, modelN);
}

void sendRssiModel() {
  sendBle("MODEL:" + String(modelA, 2) + "," + String(modelN, 3));
}

bool setRssiModel(float newA, float newN) {
  if (!isfinite(newA) || newA > -10.0f || newA < -120.0f) return false;
  if (!isfinite(newN) || newN < 1.0f || newN > 8.0f) return false;

  modelA = newA;
  modelN = newN;

  prefs.putFloat("modelA", modelA);
  prefs.putFloat("modelN", modelN);

  sendRssiModel();
  return true;
}

// =====================================================
// Anchor distances AB / AC / BC
// =====================================================
bool calculateAnchorCoordinates(float ab, float ac, float bc) {
  if (!isfinite(ab) || !isfinite(ac) || !isfinite(bc)) return false;
  if (ab <= 0.0f || ac <= 0.0f || bc <= 0.0f) return false;

  // Triangle inequality
  if (ab + ac <= bc || ab + bc <= ac || ac + bc <= ab) return false;

  float cx = (ac * ac + ab * ab - bc * bc) / (2.0f * ab);
  float cy2 = ac * ac - cx * cx;

  if (!isfinite(cx) || !isfinite(cy2) || cy2 <= 0.0001f) return false;

  float cy = sqrtf(cy2);
  if (!isfinite(cy)) return false;

  distAB = ab;
  distAC = ac;
  distBC = bc;

  BX = ab;
  BY = 0.0f;
  CX = cx;
  CY = cy;

  return true;
}

bool anchorDistancesReady() {
  return isfinite(distAB) && isfinite(distAC) && isfinite(distBC) &&
         isfinite(BX) && isfinite(CX) && isfinite(CY);
}

void loadAnchorDistances() {
  float ab = prefs.getFloat("distAB", NAN);
  float ac = prefs.getFloat("distAC", NAN);
  float bc = prefs.getFloat("distBC", NAN);

  if (calculateAnchorCoordinates(ab, ac, bc)) {
    Serial.printf("[ANCHOR DIST] AB=%.2f AC=%.2f BC=%.2f\n", distAB, distAC, distBC);
    Serial.printf("[ANCHOR XY] A=(0.00,0.00) B=(%.2f,0.00) C=(%.2f,%.2f)\n",
                  BX, CX, CY);
  } else {
    distAB = distAC = distBC = NAN;
    BX = CX = CY = NAN;
    Serial.println("[ANCHOR DIST] NOT SET");
  }
}

void sendAnchorDistances() {
  if (!anchorDistancesReady()) {
    sendBle("DIST:-1,-1,-1");
    return;
  }

  sendBle("DIST:" +
          String(distAB, 2) + "," +
          String(distAC, 2) + "," +
          String(distBC, 2));
}

bool saveAnchorDistances(float ab, float ac, float bc) {
  if (!calculateAnchorCoordinates(ab, ac, bc)) return false;

  prefs.putFloat("distAB", distAB);
  prefs.putFloat("distAC", distAC);
  prefs.putFloat("distBC", distBC);

  Serial.printf("[ANCHOR DIST SAVED] AB=%.2f AC=%.2f BC=%.2f\n", distAB, distAC, distBC);
  Serial.printf("[ANCHOR XY] A=(0.00,0.00) B=(%.2f,0.00) C=(%.2f,%.2f)\n",
                BX, CX, CY);

  sendAnchorDistances();
  return true;
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
      rssiA.ready();
      rssiB.add(rs.toInt());
    }

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
  if (!anchorDistancesReady()) {
    sendBle("ERR:DIST_NOT_SET");
    return;
  }

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
  String zReason = "UNKNOWN";
  float currentPress = pressureBuf.median();

  // ===== 기존 기압 상대고도 보정 로직 유지 + OFF 원인 기록 =====
  if (!pressureBuf.fresh()) {
    zReason = "PRESS_STALE";
  } else if (!validPressure(currentPress)) {
    zReason = "PRESS_INVALID";
  } else if (!allBasesReady()) {
    zReason = "BASE_NOT_SET";
  } else {
    dz[0] = relativeHeightMeters(currentPress, basePressA);
    dz[1] = relativeHeightMeters(currentPress, basePressB);
    dz[2] = relativeHeightMeters(currentPress, basePressC);

    bool geometryOK = true;

    for (int i = 0; i < 3; i++) {
      if (!isfinite(dz[i])) {
        zReason = String("DZ_INVALID_") + char('A' + i);
        geometryOK = false;
        break;
      }
      if (fabsf(dz[i]) >= d3d[i]) {
        zReason = String("DZ_GE_DIST_") + char('A' + i);
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
      zReason = "OK";
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
    sendBle("ZSTAT:ON,OK");
    sendBle("DZ:" + String(dz[0], 2) + "," +
                    String(dz[1], 2) + "," +
                    String(dz[2], 2));
  } else {
    sendBle("ZSTAT:OFF," + zReason);
    sendBle("WARN:ZCORR_OFF");
  }
}

// =====================================================
// BLE callbacks
// =====================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    restartAdvertising = false;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    bleDisconnectedAt = millis();
    restartAdvertising = true;
    Serial.println("[BLE] Client disconnected");
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = pChar->getValue().c_str();
    cmd.trim();
    cmd.toUpperCase();

    Serial.printf("[BLE RX] %s\n", cmd.c_str());

    if (cmd == "MEASURE" || cmd == "1") {
      Serial.println("[MEASURE] runSnapshot() start");
      runSnapshot();
      return;
    }

    // ----- Anchor distances -----
    if (cmd == "GETDIST") {
      sendAnchorDistances();
      return;
    }

    if (cmd.startsWith("SETDIST:")) {
      String values = cmd.substring(8);

      int c1 = values.indexOf(',');
      int c2 = (c1 >= 0) ? values.indexOf(',', c1 + 1) : -1;

      if (c1 < 0 || c2 < 0) {
        sendBle("ERR:DIST_FORMAT");
        return;
      }

      float ab = values.substring(0, c1).toFloat();
      float ac = values.substring(c1 + 1, c2).toFloat();
      float bc = values.substring(c2 + 1).toFloat();

      if (!saveAnchorDistances(ab, ac, bc)) {
        sendBle("ERR:DIST_TRIANGLE");
      }
      return;
    }

    // ----- Pressure calibration: unchanged -----
    if (cmd == "GETBASE") {
      sendBaseStatus();
      return;
    }

    if (cmd == "CLRBASE") {
      clearBases();
      return;
    }

    if (cmd == "CAL:A" || cmd == "CAL:B" || cmd == "CAL:C") {
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

    // ----- RSSI A/n model: unchanged -----
    if (cmd == "GETMODEL") {
      sendRssiModel();
      return;
    }

    if (cmd.startsWith("SETMODEL:")) {
      String values = cmd.substring(9);
      int comma = values.indexOf(',');

      if (comma < 0) {
        sendBle("ERR:MODEL_FORMAT");
        return;
      }

      float newA = values.substring(0, comma).toFloat();
      float newN = values.substring(comma + 1).toFloat();

      if (!setRssiModel(newA, newN)) {
        sendBle("ERR:MODEL_RANGE");
      }
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
  if (millis() - lastStatus < 2000) return;
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

    // DIO1에서 실제 RX_DONE이 발생했을 때만 FIFO를 읽는다.
    radio.setPacketReceivedAction(onLoraPacketReceived);

    int rxState = radio.startReceive();
    if (rxState == RADIOLIB_ERR_NONE) {
      Serial.println("[LORA] Master OK / RX interrupt armed");
    } else {
      Serial.printf("[LORA] startReceive FAIL code=%d\n", rxState);
    }
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
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
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

  Serial.println("\n=== SANJIGI MASTER / PRESS + WLS + DIST ===");

  loadBases();
  loadRssiModel();
  loadAnchorDistances();

  initLoRa();
  initBLE();
}

void loop() {
  // 패킷 수신 완료 인터럽트가 발생했을 때만 FIFO를 읽는다.
  if (loraPacketReceived) {
    loraPacketReceived = false;

    String msg;
    int state = radio.readData(msg);

    if (state == RADIOLIB_ERR_NONE) {
      int directRssi = (int)radio.getRSSI();

      Serial.printf("[LORA RX] %s | RSSI:%d dBm\n",
                    msg.c_str(), directRssi);

      parseIncomingPacket(msg, directRssi);
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("[LORA RX] CRC mismatch - packet discarded");
    } else {
      Serial.printf("[LORA RX] readData error=%d\n", state);
    }

    // 다음 패킷 수신 대기
    int rxState = radio.startReceive();
    if (rxState != RADIOLIB_ERR_NONE) {
      Serial.printf("[LORA] restart RX FAIL code=%d\n", rxState);
    }
  }

  // Disconnect callback 안에서 즉시 advertising을 재시작하지 않고
  // BLE 스택이 정리될 시간을 준 뒤 재시작한다.
  if (restartAdvertising && !deviceConnected &&
      millis() - bleDisconnectedAt >= 500) {
    restartAdvertising = false;
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising restarted");
  }

  sendStatus();
}
