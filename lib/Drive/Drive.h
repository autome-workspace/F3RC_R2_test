#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

class Drive {
public:
    Drive();
    void driveSetMotor(uint8_t motorNumber, uint8_t motorPinA, uint8_t motorPinB);
    void driveSetMax(uint8_t PWM_MAX, float MOTOR_SPD_MAX, float MOTOR_ACC_MAX);
    void driveSetLimit(float motorSpdLimit, float motorAccLimit);
    
    //bool driveLinear(float distance, float radians);
    //bool driveTurn(float radians);
    bool drive(float distance, float radians, float turnRadians);
    bool stop();

private:
    uint8_t _motorPIN[4][2];
    uint8_t _PWM_MAX;
    float _MOTOR_SPD_MAX;
    float _MOTOR_ACC_MAX;
    float _motorSpdLimit;
    float _motorAccLimit;


};


#endif