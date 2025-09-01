#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

class Drive {
public:

    // コンストラクタ
    Drive(uint8_t motor0_pinA, uint8_t motor0_pinB, bool rev0,
          uint8_t motor1_pinA, uint8_t motor1_pinB, bool rev1,
          uint8_t motor2_pinA, uint8_t motor2_pinB, bool rev2,
          uint8_t motor3_pinA, uint8_t motor3_pinB, bool rev3,
          uint16_t drive_pulse, float wheelRadius);

    // 限界設定
    void driveSetMax(uint8_t PWM_MAX, float MOTOR_SPD_MAX, float MOTOR_ACC_MAX);
    // 上限設定 (限界 > 上限 > 0)
    void driveSetLimit(float motorSpdLimit, float motorAccLimit);

    bool driveSimple(float distance, float radians, float turnRadians);
    bool driveSimple(float distance, float radians);
    bool driveSimple(float turnRadians);

    bool drive(float distance, float radians, float turnRadians);
    bool drive(float distance, float radians);
    bool drive(float turnRadians);
    bool stop();
    bool quickStop;

private:
    uint8_t _motorPIN[4][2]; // {{A, B}, {A, B}, ...}
    uint8_t _PWM_MAX;
    float _MOTOR_SPD_MAX;
    float _MOTOR_ACC_MAX;
    float _motorSpdLimit;
    float _motorAccLimit;
    uint16_t _drive_pulse;
    float _wheelRadius;

    void motorWrite(uint8_t motorNum, float motorSpd);

};


#endif