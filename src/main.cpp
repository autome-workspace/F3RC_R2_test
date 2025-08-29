#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <math.h>
//#include <esp32-hal-timer.h>
//#include <BLEUtils.h>

#define Aa 15
#define Ba 7
#define Ab 18
#define Bb 17
#define Ac 10
#define Bc 9

#define SERVICE_UUID        "32e29e2d-e309-42d6-833f-597d88fae281"
#define CHARACTERISTIC_UUID "c63c080c-5ccc-4905-a015-c59634eef0bc"

int stickRx = 0, stickRy = 0, stickLx = 0, stickLy = 0;
int t0Value = 0, t1Value = 0;

class BLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string raw = pCharacteristic->getValue();

    if (raw[0] == 0x01 && raw.back() == 0x03) { //␁label␂data␃
      size_t stxPos = raw.find(0x02);

      if (stxPos != std::string::npos) {
        std::string label = raw.substr(1, stxPos - 1);
        std::string data = raw.substr(stxPos + 1, raw.size() - stxPos - 2);

        size_t commaPos = data.find(',');
        if (commaPos != std::string::npos) {
          String xStr = data.substr(0, commaPos).c_str();
          String yStr = data.substr(commaPos + 1).c_str();

          if(label == "d0") {
            stickLx = xStr.toInt() - 512;   stickLy = yStr.toInt() - 512;
          }
          if(label == "d1") {
            stickRx = xStr.toInt() - 512;   stickRy = yStr.toInt() - 512;
          }
          

        } else if(label == "t0") {

          String textStr = data.c_str();
            t0Value = textStr.toInt();
            Serial.println(t0Value);

        } else if(label == "t1") {

          String textStr = data.c_str();
            t1Value = textStr.toInt();
            Serial.println(t1Value);

        } else if(0) {
          //other label here

        }

      }

    } else {
      Serial.println("Invalid format.");

    }
  }
};

class BLEConnectCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Connected");
  }
  void onDisconnect(BLEServer* pServer) {
    Serial.println("Disconnected");
    stickLx = 0;   stickLy = 0;
    stickRx = 0;   stickRy = 0;
    pServer->getAdvertising()->start();
  }
};

void setup() {
  Serial.begin(115200);
  BLEDevice::init("ESP32_BLE");  // デバイス名を設定
  BLEServer *pServer = BLEDevice::createServer();  // BLEサーバーを作る
  BLEService *pService = pServer->createService(SERVICE_UUID);  // サービスを作成

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setCallbacks(new BLECallbacks());
  pServer->setCallbacks(new BLEConnectCallbacks());
  pService->start();  // サービス開始

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();  // 広告開始（スマホに見えるように）
  
  Serial.println("BLE device is ready and waiting...");
  
  pinMode(Aa, OUTPUT);
  pinMode(Ba, OUTPUT);
  pinMode(Ab, OUTPUT);
  pinMode(Bb, OUTPUT);
  pinMode(Ac, OUTPUT);
  pinMode(Bc, OUTPUT);
}

float lx, ly, rx, ry;   //current input left/right
float cx = 0, cy = 0, cw = 0;   //current output

const int pulseRate = ( 100 ); // ~500,000Hz
const float head_offset = ( 60 ) * M_PI / 180;
const float wheelR = ( 1 );    //propotional to diameter of wheel
float motorMaxAngAcc = ( 5 );   //angular acceleration threshold(角加速度限界)    mm/(s*s)

const float robotR = ( 1 );
const float motorMaxSpeed = 235;

void loop() {

  while (micros() % (1000000 / pulseRate) > 1) {}
  //Serial.println(micros());

  //目標値（入力値）
  lx = stickLx / 2;   ly = stickLy / 2;
  rx = stickRx / 2;   //ry = stickRy / 2;

  float inputr = sqrt(lx*lx + ly*ly);
  if(inputr > motorMaxSpeed) {
    lx = lx * motorMaxSpeed / inputr;
    ly = ly * motorMaxSpeed / inputr;
  }
  if(abs(rx) > motorMaxSpeed) rx = rx * motorMaxSpeed / abs(rx);
  
  //目標と現在の値の差分をとる
  float dx = lx - cx;
  float dy = ly - cy;

  float dr = sqrt(dx*dx + dy*dy);
  float dtheta = atan2f(dy, dx);

  float dta, dtb, dtc;
  dta = M_PI*(0)/180 - head_offset - dtheta;
  dtb = M_PI*(120)/180 - head_offset - dtheta;
  dtc = M_PI*(240)/180 - head_offset - dtheta;

  float dw = rx - cw;
  
  float ratio = max(abs(dr*cosf(dta) - dw*robotR), 
                max(abs(dr*cosf(dtb) - dw*robotR), 
                    abs(dr*cosf(dtc) - dw*robotR))) / motorMaxAngAcc;

  if(ratio > 1) {
    cx += dx / ratio;
    cy += dy / ratio;
    cw += dw / ratio;
  } else {
    cx = lx;    cy = ly;    cw = rx;
  }
  float r = sqrt(cx*cx + cy*cy);
  float theta = atan2f(cy, cx);

  float ta, tb, tc;
  ta = M_PI*(0)/180 - head_offset - theta;
  tb = M_PI*(120)/180 - head_offset - theta;
  tc = M_PI*(240)/180 - head_offset - theta;

  int wa, wb, wc;
  wa = (r*cosf(ta) + cw) / wheelR;
  wb = (r*cosf(tb) + cw) / wheelR;
  wc = (r*cosf(tc) + cw) / wheelR;

  ratio = max(abs(wa), 
          max(abs(wb), 
              abs(wc))) / motorMaxSpeed;

  if(ratio > 1) {
  wa /= ratio;
  wb /= ratio;
  wc /= ratio;
  }

  if(wa > 0){
    analogWrite(Aa, wa);
    analogWrite(Ba, 0);
  } else {
    analogWrite(Aa, 0);
    analogWrite(Ba, -wa);
  }
  if(wb > 0){
    analogWrite(Ab, wb);
    analogWrite(Bb, 0);
  } else {
    analogWrite(Ab, 0);
    analogWrite(Bb, -wb);
  }
  if(wc > 0){
    analogWrite(Ac, wc);
    analogWrite(Bc, 0);
  } else {
    analogWrite(Ac, 0);
    analogWrite(Bc, -wc);
  }

}