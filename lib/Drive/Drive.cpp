#include "Drive.h"

Drive::Drive(Odometry* odometry, uint16_t drive_pulse, float wheelRadius)
:   _odometry{odometry},
    _drive_pulse(drive_pulse),
    _wheelRadius(wheelRadius) {}

void Drive::driveSetMax(uint8_t PWM_MAX, float MOTOR_SPD_MAX, float MOTOR_ACC_MAX) {
    //PWM_MAX: PWM最大値
    //MOTOR_SPD_MAX: モーター最大速度  
    //MOTOR_ACC_MAX: モーター最大加速度
    _PWM_MAX = PWM_MAX;
    _MOTOR_SPD_MAX = MOTOR_SPD_MAX;
    _MOTOR_ACC_MAX = MOTOR_ACC_MAX;
}

void Drive::driveSetLimit(float MOTOR_SPD_LIMIT, float MOTOR_ACC_LIMIT) {
    //motorSpdLimit: 速度制限
    //motorAccLimit: 加速度制限
    _motorSpdLimit = MOTOR_SPD_LIMIT;
    _motorAccLimit = MOTOR_ACC_LIMIT;
}

//bool Drive::drive(float targetX, float targetY, float targetRadians) {}

bool Drive::drive(float targetX, float targetY) {
    float _current_x = _odometry->getX();
    float _current_y = _odometry->getY();
    float dx = targetX - _current_x;
    float dy = targetY - _current_y;
    return true;

    
}

bool Drive::drive(float targetRadians) {return true;}

void Drive::motorWrite(uint8_t motorNum, float motorSpd) {
    int PWM = _PWM_MAX * motorSpd / _motorSpdLimit;
    if(PWM > 0) {
        ledcWrite(motorNum, PWM);
        ledcWrite(motorNum + 4, PWM);
    } else {
        ledcWrite(motorNum, 0);
        ledcWrite(motorNum + 4, PWM);
    }
}