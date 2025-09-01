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
#define WHEELX_DISTANCE 55.0f // mm
#define WHEELY_DISTANCE 55.0f // mm
float ENCODER_WHEEL_RADIUS = 20.0f; // mm

#define MOTOR0_PINA 7
#define MOTOR0_PINB 15
#define MOTOR1_PINA 7
#define MOTOR1_PINB 8
#define MOTOR2_PINA 9
#define MOTOR2_PINB 10
#define MOTOR3_PINA 13
#define MOTOR3_PINB 14
#define MOTOR_PWM_MAX 500
float MOTOR_SPD_MAX = 100.0f; // mm/s
float MOTOR_ACC_MAX = 200.0f; // mm/s^2
float MOTOR_SPD_LIMIT = 50.0f; // mm/s
float MOTOR_ACC_LIMIT = 150.0f; // mm/s^2
float MOTOR_WHEEL_RADIUS = 30.0f; // mm

#define MAIN_PULSE_RATE 1000 // Hz
#define ODOM_PULSE_RATE 100 // Hz

// ライブラリのインスタンスを生成
Encoder encoder(ENCODERX_PINA, ENCODERX_PINB, 
                ENCODERY_PINA, ENCODERY_PINB, 
                ENCODER_PULSE_PER_REV, ENCODER_WHEEL_RADIUS);
BNO055 bno;
Odometry odometry(&encoder, &bno, WHEELX_DISTANCE, WHEELY_DISTANCE);

void setup() {
    Serial.begin(921600);

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
    
    bno.isFliped();

    if (bno.loadCalibration()) {
        Serial.println("EEPROMからキャリブレーションデータをロードしました。");
        Serial.println("センサーはすぐに使用可能です。");
        delay(3000);
    } else {
        Serial.println("EEPROMに有効なキャリブレーションデータが見つかりませんでした。");
        Serial.println("センサーを動かしてキャリブレーションを完了させてください。");
        while(1) delay(1000);
    }

}

void loop() {
    static long loopCount = -1;
    loopCount++;
    
    while(micros() % (1000000 / MAIN_PULSE_RATE) > 10);
    if(loopCount % (MAIN_PULSE_RATE / ODOM_PULSE_RATE) == 0)
        odometry.update();

    Serial.println(bno.getAccZ());

}