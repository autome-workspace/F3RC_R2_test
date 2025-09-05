#include <Arduino.h>
#include <string>
#include "Encoder.h"
#include "BNO055.h"
#include "Odometry.h"
#include "Drive.h"

typedef struct {
    /*std::string status;
    /* ステータスリスト
    Parking
    Moving
    Taking (石炭回収)
    Catching (石油回収)
    Putting （投下・設置） ...など？
    いいの思いつかない...
    */
    float targetX, targetY, targetYAW, SPD_LIMIT, ACC_LIMIT;
} ORDERLIST;

ORDERLIST ORDER_LIST[3] = 
{
    {0, 0, 0, 0, 0}, // 初期状態
    {0, 0, 0, 30, 10},
    {0, 0, 0, -30, 10}
};

#define I2C_SDA 11
#define I2C_SCL 12
#define ENCODERX_PINA 41
#define ENCODERX_PINB 38
#define ENCODERY_PINA 3
#define ENCODERY_PINB 4
#define MOTOR0_PINA 7
#define MOTOR0_PINB 15
#define MOTOR1_PINA 17
#define MOTOR1_PINB 18
#define MOTOR2_PINA 13
#define MOTOR2_PINB 14
#define MOTOR3_PINA 33
#define MOTOR3_PINB 34
#define ENCODER_PULSE_PER_REV 8192 // pulse/rev
#define ENCODER_WHEELX_DISTANCE 55.0f // mm
#define ENCODER_WHEELY_DISTANCE 0.0f // mm

// メカナム設定、旋回のため
#define DRIVE_WHEEL_X 300.0f // mm
#define DRIVE_WHEEL_Y 200.0f // mm

// オムニ設定
#define DRIVE_WHEEL_NUMBER 3 // ホイール数
#define DRIVE_WHEEL_DISTANCE 200.0f // mm

// オドメトリ精度に直結
#define ENCODER_WHEEL_RADIUS 24.1f // mm

// お好みの最高速で(下の限界値をなるべく合わせる)
#define MOTOR_PWM_MAX 100

// 走行中に補正されていく(予定)
float MOTOR_SPD_MAX = 500.0f; // mm/s
float MOTOR_ACC_MAX = 500.0f; // mm/s^2

//指令リストで指定
float ROBOT_SPD_LIMIT = 0.0f; // mm/s
float ROBOT_ACC_LIMIT = 0.0f; // mm/s^2
float ROBOT_TURN_LIMIT = 0.0f; // degree/s

//制御レート
#define MAIN_PULSE_RATE 100 // Hz
#define ODOM_PULSE_RATE 100 // Hz

uint8_t motorPIN[4][2] = {{MOTOR0_PINA, MOTOR0_PINB}, 
                          {MOTOR1_PINA, MOTOR1_PINB},
                          {MOTOR2_PINA, MOTOR2_PINB},
                          {MOTOR3_PINA, MOTOR3_PINB}};

Encoder encoder(ENCODERX_PINA, ENCODERX_PINB, 
                ENCODERY_PINA, ENCODERY_PINB, 
                ENCODER_PULSE_PER_REV, ENCODER_WHEEL_RADIUS);
BNO055 bno(I2C_SDA, I2C_SCL);
Odometry odometry(&encoder, &bno, ENCODER_WHEELX_DISTANCE, ENCODER_WHEELY_DISTANCE);
Drive drive(&odometry, MAIN_PULSE_RATE, MOTOR_PWM_MAX);

void setup() {
    Serial.begin(921600);
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    
    // 初期化
    encoder.begin();
    while(!bno.begin()) {
        Serial.println("BNO055 failed to initialize.");
        delay(1000);
    }
    odometry.begin();

    // モーターセットアップ
    for(int i = 0; i < 4; i++) {
        // PWMは16kHz,　1024段階
        ledcSetup(i, 16000, 10);
        ledcSetup(i+4, 16000, 10);
        ledcAttachPin(motorPIN[i][0], i);
        ledcAttachPin(motorPIN[i][1], i + 4);
    }

    //drive.driveSetMecanumX(DRIVE_WHEEL_X, DRIVE_WHEEL_Y);
    drive.driveSetOmni(DRIVE_WHEEL_DISTANCE, DRIVE_WHEEL_NUMBER);


}

void loop() {
    static long loopCount = 0;
    static long phase = 0;
    
    drive.driveSetLinearLimit(0, 0);
    drive.driveSetTurnLimit(20, 20);

    if(drive.drive(ORDER_LIST[phase].targetX,ORDER_LIST[phase].targetY, ORDER_LIST[phase].targetYAW)) phase++;
    
    while(micros() % (1000000 / MAIN_PULSE_RATE) > 1);
    if(loopCount % (MAIN_PULSE_RATE / ODOM_PULSE_RATE) == 0) {
        odometry.update();
        Serial.print(odometry.getYAW());
        Serial.print("  ");
        Serial.println(bno.getOperatingMode());
    }


    loopCount++;

}