#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>



// BNO055センサーオブジェクトを作成
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(115200);
  Wire.begin(11, 12);
  delay(1000);
  if (!bno.begin()) {
    Serial.println("BNO055が見つかりませんでした。接続を確認してください。");
    while (1)
      delay(10);
  }

  delay(1000); // センサーのキャリブレーションを待つ
  bno.setExtCrystalUse(true);
}

void loop() {
  // オイラー角（向き）データを取得
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.printf("X: %.2f, Y: %.2f, Z: %.2f\n", euler.x(), euler.y(), euler.z());
  
  delay(100);
}