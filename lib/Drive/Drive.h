#ifndef DRIVE_LIBRARY_H
#define DRIVE_LIBRARY_H

#include <Arduino.h>

class DriveLibrary {
    public:
        void driveSetMotor(int8_t motorNumber, int8_t motorPinA, int8_t motorPinB);
        void driveSetMax(int8_t PWM_MAX, float MOTOR_SPD_MAX, float MOTOR_ACC_MAX);
        void driveSetLimit(float motorSpdLimit, float motorAccLimit);
        
        //bool driveLinear(float distance, float radians);
        //bool driveTurn(float radians);
        bool drive(float distance, float radians, float turnRadians);
        bool stop();

    private:
        int8_t _motorPIN[4][2];
        int8_t _PWM_MAX;
        float _MOTOR_SPD_MAX;
        float _MOTOR_ACC_MAX;
        float _motorSpdLimit;
        float _motorAccLimit;


};


#endif