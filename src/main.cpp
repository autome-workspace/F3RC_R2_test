#include <Arduino.h>
#include "BNO055.h"

BNO055 imu;

void setup() {
    Serial.begin(115200);
    if(!imu.begin()) {
        Serial.println("BNO055 initialization failed!");
        while(1);
    } else {
        Serial.println("BNO055 initialized successfully.");

    }
}

void loop() {
    float yaw = imu.getYaw();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    delay(1000);
}