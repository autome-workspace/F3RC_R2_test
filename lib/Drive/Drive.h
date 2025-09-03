#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "Odometry.h"

class Drive {
public:

    // コンストラクタ
    Drive(Odometry* odometry, uint16_t drive_pulse, uint8_t PWM_MAX);

    // 限界設定
    void driveSetMax(float MOTOR_SPD_MAX, float MOTOR_ACC_MAX);
    // 上限設定 (限界 > 上限 > 0)
    void driveSetLimit(float ROBOT_SPD_LIMIT, float ROBOT_ACC_LIMIT);


    bool drive(float targetX, float targetY, float targetYAW);
    //bool drive(float targetX, float targetY);
    //bool drive(float targetRadians);
    //bool stop();
    //bool quickStop();

private:
    uint8_t _motorPIN[4][2]; // {{A, B}, {A, B}, ...}
    uint16_t _DRIVE_PULSE;
    uint8_t _PWM_MAX;
    float _MOTOR_SPD_MAX;
    float _MOTOR_ACC_MAX;
    float _ROBOT_SPD_LIMIT;
    float _ROBOT_ACC_LIMIT;
    float _WHEEL_RADIUS;
    float cvx; //current v_X
    float cvy; //current v_y

    Odometry* _odometry;

    void motorWrite(uint8_t motorNum, float motorSpd);

    // 
    float K_rad()

};


#endif