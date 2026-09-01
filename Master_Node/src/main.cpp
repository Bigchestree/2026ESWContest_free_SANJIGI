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

// ================= LoRa =================
#define LORA_NSS  41
#define LORA_BUSY 40
#define LORA_NRST 42
#define LORA_DIO1 39
#define LORA_SCK  7
#define LORA_MISO 8
#define LORA_MOSI 9

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(
  LORA_NSS,
  LORA_DIO1,
  LORA_NRST,
  LORA_BUSY,
  loraSPI
);

bool loraOK = false;
volatile bool receivedFlag = false;

void setFlag() {
  receivedFlag = true;
}

// =====================================================
// 고정 Anchor 좌표
// AB = 15m
// AC = 20m
// BC = 15m
// =====================================================

const float AX = 0.0f;
const float AY = 0.0f;

const float BX = 15.0f;
const float BY = 0.0f;

const float CX = 13.3333f;
const float CY = 14.9071f;

// ================= BLE =================

#define SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_DATA_UUID "a3c17822-1d5b-4176-a447-0624916a0487"

BLECharacteristic* dataChar = nullptr;
bool bleConnected = false;

Preferences prefs;

// ================= RSSI 모델 =================

float modelA = -40.0f;
float modelN = 2.8f;

// ================= 기준 기압 =================

float baseA = NAN;
float baseB = NAN;
float baseC = NAN;

// =====================================================
// RSSI BUFFER
// =====================================================

struct RssiBuf {

  int v[5] = {0};

  int idx = 0;
  int count = 0;

  unsigned long last = 0;

  void add(int r) {

    if (r > -10 || r < -140) return;

    v[idx++] = r;

    if (idx >= 5) {
      idx = 0;
    }

    if (count < 5) {
      count++;
    }

    last = millis();
  }

  bool ready() {
    return count >= 5;
  }

  bool alive() {
    return count > 0 &&
           millis() - last < 7000;
  }

  int median() {

    if (count < 5) {
      return -999;
    }

    int t[5];

    memcpy(
      t,
      v,
      sizeof(t)
    );

    std::sort(
      t,
      t + 5
    );

    return t[2];
  }

  float spread() {

    if (count < 2) {
      return 10.0f;
    }

    float mean = 0.0f;

    for (int i = 0; i < count; i++) {
      mean += v[i];
    }

    mean /= count;

    float sum = 0.0f;

    for (int i = 0; i < count; i++) {

      float d =
        v[i] - mean;

      sum += d * d;
    }

    return sqrtf(
      sum / (count - 1)
    );
  }
};

RssiBuf rA;
RssiBuf rB;
RssiBuf rC;

// =====================================================
// PRESS BUFFER
// 30개 기압값 중앙값 사용
// =====================================================

struct PressBuf {

  float v[30] = {0};

  int idx = 0;
  int count = 0;

  unsigned long last = 0;

  void add(float p) {

    if (!isfinite(p)) return;

    if (p < 300.0f ||
        p > 1100.0f) {
      return;
    }

    v[idx++] = p;

    if (idx >= 30) {
      idx = 0;
    }

    if (count < 30) {
      count++;
    }

    last = millis();
  }

  bool fresh() {

    return count > 0 &&
           millis() - last < 7000;
  }

  float median() {

    if (count == 0) {
      return NAN;
    }

    float t[30];

    for (int i = 0; i < count; i++) {
      t[i] = v[i];
    }

    std::sort(
      t,
      t + count
    );

    if (count % 2) {

      return t[count / 2];

    } else {

      return (
        t[count / 2 - 1] +
        t[count / 2]
      ) / 2.0f;
    }
  }
};

PressBuf press;

// =====================================================
// 공통
// =====================================================

bool validPress(float p) {

  return isfinite(p) &&
         p >= 300.0f &&
         p <= 1100.0f;
}

void bleSend(const String& s) {

  Serial.println(
    "[BLE TX] " + s
  );

  if (
    !bleConnected ||
    dataChar == nullptr
  ) {
    return;
  }

  dataChar->setValue(
    s.c_str()
  );

  dataChar->notify();

  delay(20);
}

bool getPressure(
  const String& msg,
  float& p
) {

  int i =
    msg.indexOf("PRESS:");

  if (i < 0) {
    return false;
  }

  String s =
    msg.substring(i + 6);

  int comma =
    s.indexOf(',');

  if (comma >= 0) {

    s =
      s.substring(
        0,
        comma
      );
  }

  p = s.toFloat();

  return validPress(p);
}

bool getRelayRSSI(
  const String& msg,
  int& r
) {

  int i =
    msg.indexOf("RSSI:");

  if (i < 0) {
    return false;
  }

  String s =
    msg.substring(i + 5);

  int comma =
    s.indexOf(',');

  if (comma >= 0) {

    s =
      s.substring(
        0,
        comma
      );
  }

  r = s.toInt();

  return r <= -10 &&
         r >= -140;
}

// =====================================================
// RSSI → 거리
// =====================================================

float rssiDistance(int rssi) {

  return powf(
    10.0f,
    (
      modelA -
      (float)rssi
    ) /
    (
      10.0f *
      modelN
    )
  );
}

float getWeight(RssiBuf& b) {

  float sigma =
    b.spread();

  float w =
    1.0f /
    (
      sigma * sigma +
      4.0f
    );

  if (w < 0.01f) {
    w = 0.01f;
  }

  if (w > 0.25f) {
    w = 0.25f;
  }

  return w;
}

// =====================================================
// 기압 → 상대 높이
// =====================================================

float pressureHeight(
  float current,
  float reference
) {

  return
    44330.0f *
    (
      1.0f -
      powf(
        current / reference,
        0.19029495f
      )
    );
}

// =====================================================
// LoRa 패킷 처리
// =====================================================

void processPacket(
  const String& msg,
  int directRSSI
) {

  // TARGET 직접 수신 = A RSSI
  if (
    msg.startsWith(
      "TARGET:PING"
    )
  ) {

    rA.add(
      directRSSI
    );

    float p;

    if (
      getPressure(
        msg,
        p
      )
    ) {

      press.add(p);
    }

    Serial.printf(
      "[TARGET] RSSI=%d PRESS=%.2f SAMPLE=%d/30\n",
      directRSSI,
      press.median(),
      press.count
    );

    return;
  }

  // Anchor2가 측정한 Target RSSI
  if (
    msg.startsWith(
      "ANCHOR2:"
    )
  ) {

    int rssi;

    if (
      getRelayRSSI(
        msg,
        rssi
      )
    ) {

      rB.add(rssi);

      Serial.printf(
        "[A2] RSSI=%d\n",
        rssi
      );
    }

    return;
  }

  // Anchor3가 측정한 Target RSSI
  if (
    msg.startsWith(
      "ANCHOR3:"
    )
  ) {

    int rssi;

    if (
      getRelayRSSI(
        msg,
        rssi
      )
    ) {

      rC.add(rssi);

      Serial.printf(
        "[A3] RSSI=%d\n",
        rssi
      );
    }

    return;
  }

  Serial.print(
    "[UNKNOWN] "
  );

  Serial.println(msg);
}

// =====================================================
// WLS
// =====================================================

bool solveWLS(
  float d[3],
  float w[3],
  float& x,
  float& y
) {

  const float px[3] = {
    AX,
    BX,
    CX
  };

  const float py[3] = {
    AY,
    BY,
    CY
  };

  // 고정 앵커 삼각형 중심 부근에서 시작
  x = 9.4444f;
  y = 4.9690f;

  for (
    int iter = 0;
    iter < 20;
    iter++
  ) {

    float h00 = 0.0f;
    float h01 = 0.0f;
    float h11 = 0.0f;

    float g0 = 0.0f;
    float g1 = 0.0f;

    for (
      int i = 0;
      i < 3;
      i++
    ) {

      float dx =
        x - px[i];

      float dy =
        y - py[i];

      float predicted =
        sqrtf(
          dx * dx +
          dy * dy
        );

      if (
        predicted <
        0.01f
      ) {

        predicted =
          0.01f;
      }

      float residual =
        predicted -
        d[i];

      float jx =
        dx /
        predicted;

      float jy =
        dy /
        predicted;

      h00 +=
        w[i] *
        jx *
        jx;

      h01 +=
        w[i] *
        jx *
        jy;

      h11 +=
        w[i] *
        jy *
        jy;

      g0 +=
        w[i] *
        jx *
        residual;

      g1 +=
        w[i] *
        jy *
        residual;
    }

    float det =
      h00 * h11 -
      h01 * h01;

    if (
      fabsf(det) <
      0.000001f
    ) {

      return false;
    }

    float sx =
      -(
        h11 * g0 -
        h01 * g1
      ) /
      det;

    float sy =
      -(
        -h01 * g0 +
        h00 * g1
      ) /
      det;

    float step =
      sqrtf(
        sx * sx +
        sy * sy
      );

    if (
      step >
      30.0f
    ) {

      sx *=
        30.0f /
        step;

      sy *=
        30.0f /
        step;
    }

    x += sx;
    y += sy;

    if (
      !isfinite(x) ||
      !isfinite(y)
    ) {

      return false;
    }

    if (
      sqrtf(
        sx * sx +
        sy * sy
      ) <
      0.01f
    ) {

      break;
    }
  }

  return true;
}

// =====================================================
// 위치 계산
// =====================================================

void measure() {

  Serial.println();
  Serial.println(
    "===== MEASURE ====="
  );

  // RSSI 5개씩 필요
  if (
    !rA.ready() ||
    !rB.ready() ||
    !rC.ready()
  ) {

    String err =
      "ERR:SAMPLE," +
      String(rA.count) +
      "/5," +
      String(rB.count) +
      "/5," +
      String(rC.count) +
      "/5";

    bleSend(err);

    Serial.println(err);

    return;
  }

  // 오래된 RSSI 거부
  if (
    !rA.alive() ||
    !rB.alive() ||
    !rC.alive()
  ) {

    bleSend(
      "ERR:STALE"
    );

    return;
  }

  // RSSI 중앙값
  int ma =
    rA.median();

  int mb =
    rB.median();

  int mc =
    rC.median();

  // RSSI 기반 3D 거리
  float d3d[3] = {

    rssiDistance(ma),

    rssiDistance(mb),

    rssiDistance(mc)
  };

  // 기본적으로 RSSI 거리 그대로 사용
  float dxy[3] = {

    d3d[0],

    d3d[1],

    d3d[2]
  };

  // WLS 가중치
  float w[3] = {

    getWeight(rA),

    getWeight(rB),

    getWeight(rC)
  };

  // ===================================================
  // 기압 Z 보정
  // ===================================================

  float z = 0.0f;

  float dzA = 0.0f;
  float dzB = 0.0f;
  float dzC = 0.0f;

  int zMode = 0;

  float currentP =
    press.median();

  bool basesReady =
    validPress(baseA) &&
    validPress(baseB) &&
    validPress(baseC);

  if (
    press.fresh() &&
    validPress(currentP) &&
    basesReady
  ) {

    // Target과 각 Anchor의 높이 차이
    dzA =
      pressureHeight(
        currentP,
        baseA
      );

    dzB =
      pressureHeight(
        currentP,
        baseB
      );

    dzC =
      pressureHeight(
        currentP,
        baseC
      );

    // -----------------------------------------------
    // 작은 Z 흔들림 제거
    // ±1.5m 이하는 0m
    // -----------------------------------------------

    if (
      fabsf(dzA) <
      1.5f
    ) {
      dzA = 0.0f;
    }

    if (
      fabsf(dzB) <
      1.5f
    ) {
      dzB = 0.0f;
    }

    if (
      fabsf(dzC) <
      1.5f
    ) {
      dzC = 0.0f;
    }

    // -----------------------------------------------
    // 3D RSSI 거리보다 높이차가 작을 때만
    // 수평거리 보정 적용
    // -----------------------------------------------

    bool zValid =
      fabsf(dzA) < d3d[0] &&
      fabsf(dzB) < d3d[1] &&
      fabsf(dzC) < d3d[2];

    if (zValid) {

      dxy[0] =
        sqrtf(
          d3d[0] * d3d[0] -
          dzA * dzA
        );

      dxy[1] =
        sqrtf(
          d3d[1] * d3d[1] -
          dzB * dzB
        );

      dxy[2] =
        sqrtf(
          d3d[2] * d3d[2] -
          dzC * dzC
        );

      // 화면에 표시할 Z는
      // Master(A)를 0m 기준으로 한 Target 상대높이
      z = dzA;

      zMode = 1;

    } else {

      // 기압값과 RSSI 거리가 물리적으로 맞지 않으면
      // 위치 계산은 RSSI 거리만 사용
      z = 0.0f;

      zMode = 0;

      Serial.println(
        "[Z] CORRECTION OFF - DZ >= RSSI DIST"
      );
    }
  }

  // ===================================================
  // WLS 삼변측량
  // ===================================================

  float x;
  float y;

  if (
    !solveWLS(
      dxy,
      w,
      x,
      y
    )
  ) {

    String err =
      "ERR:WLS,D:" +
      String(dxy[0], 2) +
      "," +
      String(dxy[1], 2) +
      "," +
      String(dxy[2], 2);

    Serial.println(err);

    bleSend(err);

    return;
  }

  // ===================================================
  // SERIAL DEBUG
  // ===================================================

  Serial.printf(
    "RSSI A=%d B=%d C=%d\n",
    ma,
    mb,
    mc
  );

  Serial.printf(
    "3D DIST A=%.2f B=%.2f C=%.2f\n",
    d3d[0],
    d3d[1],
    d3d[2]
  );

  Serial.printf(
    "DZ A=%.2f B=%.2f C=%.2f\n",
    dzA,
    dzB,
    dzC
  );

  Serial.printf(
    "XY DIST A=%.2f B=%.2f C=%.2f\n",
    dxy[0],
    dxy[1],
    dxy[2]
  );

  Serial.printf(
    "PRESS CURRENT=%.2f\n",
    currentP
  );

  Serial.printf(
    "BASE A=%.2f B=%.2f C=%.2f\n",
    baseA,
    baseB,
    baseC
  );

  Serial.printf(
    "RESULT X=%.2f Y=%.2f Z=%.2f ZMODE=%d\n",
    x,
    y,
    z,
    zMode
  );

  // ===================================================
  // BLE 결과
  // ===================================================

  String result =
    "RES:" +
    String(x, 2) +
    "," +
    String(y, 2) +
    "," +
    String(z, 2) +
    "," +
    String(dxy[0], 2) +
    "," +
    String(dxy[1], 2) +
    "," +
    String(dxy[2], 2) +
    "," +
    String(zMode);

  bleSend(result);

  // Z 상세정보도 HTML로 전송
  if (zMode) {

    String dzMsg =
      "DZ:" +
      String(dzA, 2) +
      "," +
      String(dzB, 2) +
      "," +
      String(dzC, 2);

    bleSend(dzMsg);
  }

  Serial.println(
    "==================="
  );
}

// =====================================================
// 설정 불러오기
// =====================================================

void loadSettings() {

  prefs.begin(
    "sanjigi",
    false
  );

  modelA =
    prefs.getFloat(
      "modelA",
      -40.0f
    );

  modelN =
    prefs.getFloat(
      "modelN",
      2.8f
    );

  baseA =
    prefs.getFloat(
      "baseA",
      NAN
    );

  baseB =
    prefs.getFloat(
      "baseB",
      NAN
    );

  baseC =
    prefs.getFloat(
      "baseC",
      NAN
    );

  Serial.printf(
    "[LOAD] MODEL A=%.2f N=%.3f\n",
    modelA,
    modelN
  );

  Serial.printf(
    "[LOAD] BASE A=%.2f B=%.2f C=%.2f\n",
    baseA,
    baseB,
    baseC
  );
}

// =====================================================
// 기준기압
// =====================================================

void sendBase() {

  String s =
    "BASE:" +
    String(
      validPress(baseA)
        ? baseA
        : -1.0f,
      2
    ) +
    "," +
    String(
      validPress(baseB)
        ? baseB
        : -1.0f,
      2
    ) +
    "," +
    String(
      validPress(baseC)
        ? baseC
        : -1.0f,
      2
    );

  bleSend(s);
}

void saveBase(char node) {

  // 최소 20개 기압 샘플 필요
  if (
    !press.fresh() ||
    press.count < 20
  ) {

    String err =
      "ERR:PRESS_SAMPLE," +
      String(press.count) +
      "/20";

    bleSend(err);

    Serial.println(err);

    return;
  }

  float p =
    press.median();

  if (
    !validPress(p)
  ) {

    bleSend(
      "ERR:PRESS_INVALID"
    );

    return;
  }

  if (
    node == 'A'
  ) {

    baseA = p;

    prefs.putFloat(
      "baseA",
      p
    );
  }

  else if (
    node == 'B'
  ) {

    baseB = p;

    prefs.putFloat(
      "baseB",
      p
    );
  }

  else if (
    node == 'C'
  ) {

    baseC = p;

    prefs.putFloat(
      "baseC",
      p
    );
  }

  bleSend(
    "CALOK:" +
    String(node) +
    "," +
    String(p, 2)
  );

  Serial.printf(
    "[CAL] %c = %.2f hPa\n",
    node,
    p
  );
}

// =====================================================
// RSSI 모델
// =====================================================

void sendModel() {

  bleSend(
    "MODEL:" +
    String(
      modelA,
      2
    ) +
    "," +
    String(
      modelN,
      3
    )
  );
}

void setModel(String s) {

  int comma =
    s.indexOf(',');

  if (
    comma < 0
  ) {

    bleSend(
      "ERR:MODEL_FORMAT"
    );

    return;
  }

  float a =
    s.substring(
      0,
      comma
    ).toFloat();

  float n =
    s.substring(
      comma + 1
    ).toFloat();

  if (
    a < -100 ||
    a > -10 ||
    n < 1 ||
    n > 6
  ) {

    bleSend(
      "ERR:MODEL_RANGE"
    );

    return;
  }

  modelA = a;
  modelN = n;

  prefs.putFloat(
    "modelA",
    a
  );

  prefs.putFloat(
    "modelN",
    n
  );

  bleSend(
    "MODELOK:" +
    String(a, 2) +
    "," +
    String(n, 3)
  );
}

// =====================================================
// BLE CALLBACK
// =====================================================

class ServerCB :
  public BLEServerCallbacks {

  void onConnect(
    BLEServer*
  ) override {

    bleConnected = true;

    Serial.println(
      "[BLE] CONNECTED"
    );
  }

  void onDisconnect(
    BLEServer*
  ) override {

    bleConnected = false;

    Serial.println(
      "[BLE] DISCONNECTED"
    );

    BLEDevice::startAdvertising();
  }
};

class CommandCB :
  public BLECharacteristicCallbacks {

  void onWrite(
    BLECharacteristic* c
  ) override {

    String cmd =
      c->getValue().c_str();

    cmd.trim();

    Serial.println(
      "[BLE RX] " +
      cmd
    );

    if (
      cmd == "MEASURE"
    ) {

      measure();

    }

    else if (
      cmd == "GETBASE"
    ) {

      sendBase();

    }

    else if (
      cmd == "CAL:A"
    ) {

      saveBase('A');

    }

    else if (
      cmd == "CAL:B"
    ) {

      saveBase('B');

    }

    else if (
      cmd == "CAL:C"
    ) {

      saveBase('C');

    }

    else if (
      cmd == "GETMODEL"
    ) {

      sendModel();

    }

    else if (
      cmd.startsWith(
        "SETMODEL:"
      )
    ) {

      setModel(
        cmd.substring(9)
      );

    }

    else {

      bleSend(
        "ERR:CMD"
      );
    }
  }
};

// =====================================================
// BLE STATUS
// =====================================================

unsigned long lastStat = 0;

void sendStatus() {

  if (
    !bleConnected
  ) {
    return;
  }

  if (
    millis() -
    lastStat <
    1000
  ) {

    return;
  }

  lastStat =
    millis();

  float p =
    press.median();

  String s =
    "STAT:" +
    String(
      rA.alive() ? 1 : 0
    ) +
    "," +
    String(
      rB.alive() ? 1 : 0
    ) +
    "," +
    String(
      rC.alive() ? 1 : 0
    ) +
    "," +
    String(
      rA.median()
    ) +
    "," +
    String(
      rB.median()
    ) +
    "," +
    String(
      rC.median()
    ) +
    "," +
    String(
      validPress(p)
        ? p
        : -1.0f,
      2
    );

  bleSend(s);
}

// =====================================================
// LoRa 초기화
// =====================================================

void initLoRa() {

  pinMode(
    LORA_NRST,
    OUTPUT
  );

  digitalWrite(
    LORA_NRST,
    LOW
  );

  delay(20);

  digitalWrite(
    LORA_NRST,
    HIGH
  );

  delay(100);

  loraSPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_NSS
  );

  int state =
    radio.begin(
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

  if (
    state ==
    RADIOLIB_ERR_NONE
  ) {

    radio.setDio2AsRfSwitch(
      true
    );

    radio.setPacketReceivedAction(
      setFlag
    );

    radio.startReceive();

    loraOK = true;

    Serial.println(
      "[LORA] MASTER OK"
    );

  } else {

    Serial.printf(
      "[LORA] FAIL %d\n",
      state
    );
  }
}

// =====================================================
// BLE 초기화
// =====================================================

void initBLE() {

  BLEDevice::init(
    "Master_Rescue_Node"
  );

  BLEDevice::setMTU(
    128
  );

  BLEServer* server =
    BLEDevice::createServer();

  server->setCallbacks(
    new ServerCB()
  );

  BLEService* service =
    server->createService(
      SERVICE_UUID
    );

  BLECharacteristic* cmd =
  service->createCharacteristic(
    CHAR_CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );

  cmd->setCallbacks(
    new CommandCB()
  );

  dataChar =
    service->createCharacteristic(
      CHAR_DATA_UUID,

      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
    );

  dataChar->addDescriptor(
    new BLE2902()
  );

  service->start();

  BLEAdvertising* adv =
    BLEDevice::getAdvertising();

  adv->addServiceUUID(
    SERVICE_UUID
  );

  adv->start();

  Serial.println(
    "[BLE] READY"
  );
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(
    115200
  );

  delay(1500);

  Serial.println();

  Serial.println(
    "=== SANJIGI MASTER / FIXED 15-20-15 / PRESS30 ==="
  );

  loadSettings();

  initLoRa();

  initBLE();

  Serial.println(
    "[SYSTEM] READY"
  );
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  if (
    loraOK &&
    receivedFlag
  ) {

    receivedFlag =
      false;

    String msg;

    int state =
      radio.readData(
        msg
      );

    if (
      state ==
      RADIOLIB_ERR_NONE
    ) {

      int rssi =
        (int)
        radio.getRSSI();

      processPacket(
        msg,
        rssi
      );

    }

    else if (
      state ==
      RADIOLIB_ERR_CRC_MISMATCH
    ) {

      Serial.println(
        "[RX] CRC ERROR"
      );

    }

    else {

      Serial.printf(
        "[RX] ERROR %d\n",
        state
      );
    }

    radio.startReceive();
  }

  sendStatus();
}