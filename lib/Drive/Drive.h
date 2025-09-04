#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "Odometry.h"

class Drive {
public:

    // コンストラクタ
    Drive(Odometry* odometry, int16_t drive_pulse, int16_t PWM_MAX);

    // 旋回が無理ゲーだからメカナムO配置はお勧めしない
    void driveSetMecanumO(float DRIVE_WHEEL_X, float DRIVE_WHEEL_Y); // 上から見てO字
    void driveSetMecanumX(float DRIVE_WHEEL_X, float DRIVE_WHEEL_Y); // 上から見てX字
    void driveSetOmni(float DRIVE_WHEEL_DISTANCE, float DRIVE_WHEEL_NUMBER);
    
    // 限界設定
    void driveSetMax(float MOTOR_SPD_MAX, float MOTOR_ACC_MAX);

    // 上限設定 (限界 > 上限 > 0)
    void driveSetLinearLimit(float ROBOT_LSPD_LIMIT, float ROBOT_LACC_LIMIT);
    void driveSetTurnLimit(float ROBOT_TSPD_LIMIT, float ROBOT_TACC_LIMIT);

    

    // 移動関数
    bool drive(float targetX, float targetY, float targetYAW);
    //bool drive(float targetX, float targetY);
    //bool drive(float targetRadians);
    //bool stop();
    //bool quickStop();

private:
    void motorWrite(float v, float RAD, float w);

    // 機体から見た各方向への速度比、加速度比(最高速時を１としてそれ以下)
    float K_rad(float rad); // 機体基準の角度差をいれる

    Odometry* _odometry;
    int16_t PULSE;
    int16_t _PWM_MAX;

    const float distanceTolerance = 5; // mm
    float _ROBOT_LSPD_LIMIT;
    float _ROBOT_LACC_LIMIT;
    float _ROBOT_TSPD_LIMIT;
    float _ROBOT_TACC_LIMIT;
    float _MOTOR_SPD_MAX;
    float _MOTOR_ACC_MAX;

    float _DRIVE_WHEEL_X = 0.0f;
    float _DRIVE_WHEEL_Y = 0.0f;
    float _DRIVE_WHEEL_DISTANCE = 0.0f;
    int8_t _DRIVE_WHEEL_NUMBER = 0;
    int8_t MODE = -1;

    float cX = 0.0f;
    float cY = 0.0f;
    float cYAW = 0.0f;
    float cv_Y = 0.0f; //current v_X
    float cv_X = 0.0f; //current v_y
    float cw = 0.0f;

    int drive_status = 0;
    int pre_drive_status = 0;
    /*
        0: 停止
        1: 加速、回転
        2: 最大、回転
        3: 減速、回転
        4: 回転
    */

    

};


#endif