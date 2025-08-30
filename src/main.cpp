#include <Arduino.h>
#include "BNO055.h"

BNO055 imu;

void setup() {
    Serial.begin(115200);
    
    if (!imu.begin()) {
        Serial.println("BNO055の初期化に失敗しました！");
        while (1);
    }
    Serial.println("BNO055の初期化が完了しました。");
}

void loop() {
    // 100Hz (10ms間隔)でデータを読み取る
    static unsigned long lastRead = 0;
    if (millis() - lastRead >= 10) {
        lastRead = millis();
        float yaw = imu.getYaw();
        Serial.print("ヨー角: ");
        Serial.println(yaw);
    }
}  