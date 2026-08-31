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
SX1262 radio = new Module(LORA_NSS,LORA_DIO1,LORA_NRST,LORA_BUSY,loraSPI);
bool loraOK=false;

// ================= 좌표 =================
const float AX=0.0f,  AY=0.0f;
const float BX=80.0f, BY=0.0f;
const float CX=40.0f, CY=69.0f;

// ================= BLE =================
#define SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_CMD_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_DATA_UUID "a3c17822-1d5b-4176-a447-0624916a0487"

BLECharacteristic* dataChar=nullptr;
bool bleConnected=false;
Preferences prefs;

// RSSI 거리식
float modelA=-40.0f;
float modelN=2.8f;

// 기준기압
float baseA=NAN,baseB=NAN,baseC=NAN;

// =====================================================
// RSSI 5개 중앙값
// =====================================================
struct RssiBuf {
  int v[5]={0},idx=0,count=0;
  unsigned long last=0;

  void add(int r){
    if(r>-10 || r<-140) return;
    v[idx++]=r;
    if(idx>=5) idx=0;
    if(count<5) count++;
    last=millis();
  }

  bool ready(){ return count>=5; }
  bool alive(){ return count>0 && millis()-last<5000; }

  int median(){
    if(count<5) return -999;
    int t[5];
    memcpy(t,v,sizeof(t));
    std::sort(t,t+5);
    return t[2];
  }

  float spread(){
    if(count<2) return 10.0f;
    float m=0,s=0;
    for(int i=0;i<count;i++) m+=v[i];
    m/=count;
    for(int i=0;i<count;i++){ float d=v[i]-m; s+=d*d; }
    return sqrtf(s/(count-1));
  }
};

RssiBuf rA,rB,rC;

// =====================================================
// Target PRESS
// =====================================================
struct PressBuf {
  float v[10]={0};
  int idx=0,count=0;
  unsigned long last=0;

  void add(float p){
    if(!isfinite(p)||p<300||p>1100) return;
    v[idx++]=p;
    if(idx>=10) idx=0;
    if(count<10) count++;
    last=millis();
  }

  bool fresh(){ return count>0 && millis()-last<5000; }

  float median(){
    if(count==0) return NAN;
    float t[10];
    for(int i=0;i<count;i++) t[i]=v[i];
    std::sort(t,t+count);
    if(count%2) return t[count/2];
    return (t[count/2-1]+t[count/2])*0.5f;
  }
};

PressBuf press;

// =====================================================
// 공통
// =====================================================
bool validPress(float p){
  return isfinite(p)&&p>=300&&p<=1100;
}

void bleSend(const String& s){
  Serial.println("[BLE TX] "+s);
  if(!bleConnected || !dataChar) return;
  dataChar->setValue(s.c_str());
  dataChar->notify();
  delay(20);
}

bool getPressure(const String& msg,float& p){
  int i=msg.indexOf("PRESS:");
  if(i<0) return false;

  String s=msg.substring(i+6);
  int c=s.indexOf(',');
  if(c>=0) s=s.substring(0,c);

  p=s.toFloat();
  return validPress(p);
}

bool getRelayRSSI(const String& msg,int& r){
  int i=msg.indexOf("RSSI:");
  if(i<0) return false;

  String s=msg.substring(i+5);
  int c=s.indexOf(',');
  if(c>=0) s=s.substring(0,c);

  r=s.toInt();
  return r<=-10 && r>=-140;
}

float rssiDistance(int r){
  return powf(10.0f,(modelA-r)/(10.0f*modelN));
}

float weight(RssiBuf& b){
  float s=b.spread();
  float w=1.0f/(s*s+4.0f);
  return constrain(w,0.01f,0.25f);
}

float pressureHeight(float current,float reference){
  return 44330.0f*(1.0f-powf(current/reference,0.19029495f));
}

// =====================================================
// LoRa 패킷 처리
// =====================================================
void processPacket(const String& msg,int directRSSI){
  Serial.printf("[RX] %s | RSSI=%d\n",msg.c_str(),directRSSI);

  if(msg.startsWith("TARGET:PING")){
    rA.add(directRSSI);

    float p;
    if(getPressure(msg,p)) press.add(p);
    return;
  }

  if(msg.startsWith("ANCHOR2:")){
    int r;
    if(getRelayRSSI(msg,r)) rB.add(r);
    return;
  }

  if(msg.startsWith("ANCHOR3:")){
    int r;
    if(getRelayRSSI(msg,r)) rC.add(r);
    return;
  }
}

// =====================================================
// WLS
// =====================================================
bool solveWLS(float d[3],float w[3],float& x,float& y){
  const float px[3]={AX,BX,CX};
  const float py[3]={AY,BY,CY};

  x=40.0f;
  y=23.0f;

  for(int n=0;n<20;n++){
    float h00=0,h01=0,h11=0,g0=0,g1=0;

    for(int i=0;i<3;i++){
      float dx=x-px[i];
      float dy=y-py[i];
      float r=sqrtf(dx*dx+dy*dy);

      if(r<0.01f) r=0.01f;

      float e=r-d[i];
      float jx=dx/r;
      float jy=dy/r;

      h00+=w[i]*jx*jx;
      h01+=w[i]*jx*jy;
      h11+=w[i]*jy*jy;
      g0 +=w[i]*jx*e;
      g1 +=w[i]*jy*e;
    }

    float det=h00*h11-h01*h01;
    if(fabsf(det)<0.000001f) return false;

    float sx=-(h11*g0-h01*g1)/det;
    float sy=-(-h01*g0+h00*g1)/det;

    float len=sqrtf(sx*sx+sy*sy);
    if(len>30){
      sx*=30.0f/len;
      sy*=30.0f/len;
    }

    x+=sx;
    y+=sy;

    if(!isfinite(x)||!isfinite(y)) return false;
    if(sqrtf(sx*sx+sy*sy)<0.01f) break;
  }

  return true;
}

// =====================================================
// 위치 계산
// =====================================================
void measure(){
  Serial.println("\n===== MEASURE =====");

  if(!rA.ready()||!rB.ready()||!rC.ready()){
    bleSend("ERR:SAMPLE,"+String(rA.count)+"/5,"+
           String(rB.count)+"/5,"+String(rC.count)+"/5");
    return;
  }

  if(!rA.alive()||!rB.alive()||!rC.alive()){
    bleSend("ERR:STALE");
    return;
  }

  int ma=rA.median();
  int mb=rB.median();
  int mc=rC.median();

  float d[3]={
    rssiDistance(ma),
    rssiDistance(mb),
    rssiDistance(mc)
  };

  float w[3]={
    weight(rA),
    weight(rB),
    weight(rC)
  };

  float x,y;

  if(!solveWLS(d,w,x,y)){
    bleSend("ERR:WLS");
    return;
  }

  // Z는 A 기준 상대기압만 사용.
  // RSSI 거리에서 Z를 빼지 않음.
  float z=0.0f;
  int zMode=0;
  float p=press.median();

  if(press.fresh() && validPress(p) && validPress(baseA)){
    z=pressureHeight(p,baseA);
    zMode=1;
  }

  Serial.printf("RSSI  A=%d B=%d C=%d\n",ma,mb,mc);
  Serial.printf("DIST  A=%.2f B=%.2f C=%.2f\n",d[0],d[1],d[2]);
  Serial.printf("RESULT X=%.2f Y=%.2f Z=%.2f\n",x,y,z);

  // HTML 형식과 정확히 동일
  String result="RES:"+
    String(x,2)+","+
    String(y,2)+","+
    String(z,2)+","+
    String(d[0],2)+","+
    String(d[1],2)+","+
    String(d[2],2)+","+
    String(zMode);

  bleSend(result);

  Serial.println("===================\n");
}

// =====================================================
// 설정
// =====================================================
void loadSettings(){
  prefs.begin("sanjigi",false);

  modelA=prefs.getFloat("modelA",-40.0f);
  modelN=prefs.getFloat("modelN",2.8f);

  baseA=prefs.getFloat("baseA",NAN);
  baseB=prefs.getFloat("baseB",NAN);
  baseC=prefs.getFloat("baseC",NAN);
}

void sendBase(){
  bleSend("BASE:"+
    String(validPress(baseA)?baseA:-1,2)+","+
    String(validPress(baseB)?baseB:-1,2)+","+
    String(validPress(baseC)?baseC:-1,2));
}

void saveBase(char node){
  if(!press.fresh() || press.count<5){
    bleSend("ERR:PRESS_SAMPLE");
    return;
  }

  float p=press.median();

  if(node=='A'){
    baseA=p;
    prefs.putFloat("baseA",p);
  }
  if(node=='B'){
    baseB=p;
    prefs.putFloat("baseB",p);
  }
  if(node=='C'){
    baseC=p;
    prefs.putFloat("baseC",p);
  }

  bleSend("CALOK:"+String(node)+","+String(p,2));
}

void sendModel(){
  bleSend("MODEL:"+String(modelA,2)+","+String(modelN,3));
}

void setModel(String s){
  int comma=s.indexOf(',');
  if(comma<0){
    bleSend("ERR:MODEL_FORMAT");
    return;
  }

  float a=s.substring(0,comma).toFloat();
  float n=s.substring(comma+1).toFloat();

  if(a<-100||a>-10||n<1||n>6){
    bleSend("ERR:MODEL_RANGE");
    return;
  }

  modelA=a;
  modelN=n;

  prefs.putFloat("modelA",a);
  prefs.putFloat("modelN",n);

  bleSend("MODELOK:"+String(a,2)+","+String(n,3));
}

// =====================================================
// BLE callbacks
// =====================================================
class ServerCB: public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected=true;
    Serial.println("[BLE] CONNECTED");
  }

  void onDisconnect(BLEServer*) override {
    bleConnected=false;
    Serial.println("[BLE] DISCONNECTED");
    BLEDevice::startAdvertising();
  }
};

class CommandCB: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String cmd=c->getValue().c_str();
    cmd.trim();

    Serial.println("[BLE RX] "+cmd);

    if(cmd=="MEASURE") measure();
    else if(cmd=="GETBASE") sendBase();
    else if(cmd=="CAL:A") saveBase('A');
    else if(cmd=="CAL:B") saveBase('B');
    else if(cmd=="CAL:C") saveBase('C');
    else if(cmd=="GETMODEL") sendModel();
    else if(cmd.startsWith("SETMODEL:")) setModel(cmd.substring(9));
    else bleSend("ERR:CMD");
  }
};

// =====================================================
// BLE 상태 전송
// =====================================================
unsigned long lastStat=0;

void sendStatus(){
  if(!bleConnected || millis()-lastStat<1000) return;
  lastStat=millis();

  float p=press.median();

  String s="STAT:"+
    String(rA.alive()?1:0)+","+
    String(rB.alive()?1:0)+","+
    String(rC.alive()?1:0)+","+
    String(rA.median())+","+
    String(rB.median())+","+
    String(rC.median())+","+
    String(validPress(p)?p:-1,2);

  bleSend(s);
}

// =====================================================
// LoRa 초기화
// 네 앵커와 동일한 방식
// =====================================================
void initLoRa(){
  pinMode(LORA_NRST,OUTPUT);

  digitalWrite(LORA_NRST,LOW);
  delay(20);
  digitalWrite(LORA_NRST,HIGH);
  delay(100);

  loraSPI.begin(LORA_SCK,LORA_MISO,LORA_MOSI,LORA_NSS);

  int state=radio.begin(
    923.0,125.0,9,7,0x12,
    10,8,1.6,true
  );

  if(state==RADIOLIB_ERR_NONE){
    radio.setDio2AsRfSwitch(true);
    radio.startReceive();
    loraOK=true;
    Serial.println("[LORA] MASTER OK");
  }else{
    Serial.printf("[LORA] FAIL code=%d\n",state);
  }
}

// =====================================================
// BLE 초기화
// =====================================================
void initBLE(){
  BLEDevice::init("Master_Rescue_Node");

  // RES 문자열 잘림 방지
  BLEDevice::setMTU(128);

  BLEServer* server=BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService* service=server->createService(SERVICE_UUID);

  BLECharacteristic* cmd=service->createCharacteristic(
    CHAR_CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  cmd->setCallbacks(new CommandCB());

  dataChar=service->createCharacteristic(
    CHAR_DATA_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  dataChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* adv=BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("[BLE] READY");
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup(){
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n=== SANJIGI MASTER ===");

  loadSettings();
  initLoRa();
  initBLE();
}

void loop(){
  if(loraOK){
    String msg;
    int state=radio.readData(msg);

    if(state==RADIOLIB_ERR_NONE){
      int directRSSI=(int)radio.getRSSI();
      processPacket(msg,directRSSI);
      radio.startReceive();
    }
  }

  sendStatus();
}