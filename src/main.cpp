#include <Arduino.h>
#include <ESP32RotaryEncoder.h>

// 結線したピン番号を定義
const int A1_PIN = 18;
const int B1_PIN = 19;

// エンコーダのオブジェクトを作成
RotaryEncoder encoder(A1_PIN, B1_PIN, -1, 4);

void setup() {
  Serial.begin(115200);

  //　カウンタの起動
  encoder.begin();

  // カウント値を初期化
  encoder.setEncoderValue(0);
}

void loop() {
  // エンコーダの現在値を取得
  long encoderValue = encoder.getEncoderValue();

}