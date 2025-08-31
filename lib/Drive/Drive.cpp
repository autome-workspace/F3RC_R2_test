#include "Drive.h"

void Drive::driveSetMotor(uint8_t motorNumber, uint8_t motorPinA, uint8_t motorPinB) {
    //motorNumber: モーター番号(0~3)
    //motorPinA: モーターAピン
    //motorPinB: モーターBピン
    if (motorNumber < 0 || motorNumber > 3) return;
    _motorPIN[motorNumber][0] = motorPinA;
    _motorPIN[motorNumber][1] = motorPinB;
    pinMode(motorPinA, OUTPUT);
    pinMode(motorPinB, OUTPUT);
}

void Drive::driveSetMax(uint8_t PWM_MAX, float MOTOR_SPD_MAX, float MOTOR_ACC_MAX) {
    //PWM_MAX: PWM最大値
    //MOTOR_SPD_MAX: モーター最大速度  
    //MOTOR_ACC_MAX: モーター最大加速度
    _PWM_MAX = PWM_MAX;
    _MOTOR_SPD_MAX = MOTOR_SPD_MAX;
    _MOTOR_ACC_MAX = MOTOR_ACC_MAX;
}

void Drive::driveSetLimit(float motorSpdLimit, float motorAccLimit) {
    //motorSpdLimit: 速度制限
    //motorAccLimit: 加速度制限
    _motorSpdLimit = motorSpdLimit;
    _motorAccLimit = motorAccLimit;
}
bool Drive::drive(float distance, float radians, float turnRadians) {
    //distance: 移動距離
    //radians: 進行方向
    //turnRadians: 回転量

}