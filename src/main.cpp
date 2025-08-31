#include <Arduino.h>
#include "Encoder.h"
#include "BNO055.h"
#include "Odometry.h"
#include "Drive.h"

#define I2C_SDA 11
#define I2C_SCL 12
#define ENCODERX_PINA 1
#define ENCODERX_PINB 2
#define ENCODERY_PINA 3
#define ENCODERY_PINB 4
#define ENCODER_PULSE_PER_REV 8192 // pulse/rev
#define ENCODER_WHEEL_RADIUS 20.0f // mm
#define WHEELX_DISTANCE 55.0f // mm
#define WHEELY_DISTANCE 55.0f // mm


#define MOTOR1_PINA 5
#define MOTOR1_PINB 6
#define MOTOR2_PINA 7
#define MOTOR2_PINB 8
#define MOTOR3_PINA 9
#define MOTOR3_PINB 10
#define MOTOR4_PINA 13
#define MOTOR4_PINB 14
#define MOTOR_PWM_MAX 255
const float MOTOR_SPD_MAX = 100.0f; // mm/s
const float MOTOR_ACC_MAX = 200.0f; // mm/s^2
const float MOTOR_SPD_LIMIT = 50.0f; // mm/s
const float MOTOR_ACC_LIMIT = 150.0f; // mm/s^2
const float MOTOR_WHEEL_RADIUS = 30.0f; // mm

// ライブラリのインスタンスを生成
Encoder encoder(ENCODERX_PINA, ENCODERX_PINB, 
                ENCODERY_PINA, ENCODERY_PINB, 
                ENCODER_PULSE_PER_REV, ENCODER_WHEEL_RADIUS);
BNO055 bno;
Odometry odometry(&encoder, &bno, WHEELX_DISTANCE, WHEELY_DISTANCE);

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // エンコーダの初期化
    encoder.begin();
    
    // BNO055の初期化
    if (!bno.begin()) {
        Serial.println("BNO055 not found or failed to initialize.");
        while (1) delay(100);
    }
    Serial.println("BNO055 initialized.");

    // オドメトリの初期化
    odometry.begin();
}

void loop() {
    // 1000Hzでオドメトリを更新
    odometry.update();

    // デバッグ情報（例：100msごとに表示）
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 100) {
        lastPrintTime = millis();
        Serial.print("X: ");
        Serial.print(odometry.getX());
        Serial.print(" mm, Y: ");
        Serial.print(odometry.getY());
        Serial.print(" mm, Theta: ");
        Serial.print(odometry.getYaw());
        Serial.println(" degrees");
    }
}