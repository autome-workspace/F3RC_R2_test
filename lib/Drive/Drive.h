#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "Odometry.h"

class Drive {
public:

    // コンストラクタ
    Drive(Odometry* odometry, uint16_t drive_pulse, float wheelRadius);

    // 限界設定
    void driveSetMax(uint8_t PWM_MAX, float MOTOR_SPD_MAX, float MOTOR_ACC_MAX);
    // 上限設定 (限界 > 上限 > 0)
    void driveSetLimit(float motorSpdLimit, float robotAccLimit);

    void imadake(int _loopCount);

    //bool driveSimple(float targetX, float targetY, float targetRadians);
    //bool driveSimple(float targetX, float targetY);
    //bool driveSimple(float targetRadians);

    //bool drive(float targetX, float targetY, float targetRadians);
    bool drive(float targetX, float targetY);
    bool drive(float targetRadians);
    bool stop();
    bool quickStop();

private:
    uint8_t _motorPIN[4][2]; // {{A, B}, {A, B}, ...}
    uint16_t _drive_pulse;
    uint8_t _PWM_MAX;
    float _MOTOR_SPD_MAX;
    float _MOTOR_ACC_MAX;
    float _motorSpdLimit;
    float _motorAccLimit;
    float _wheelRadius;
    float _currentSpeed;

    Odometry* _odometry;

    void motorWrite(uint8_t motorNum, float motorSpd);


};


#endif