#include "Drive.h"

Drive::Drive(uint8_t motor0_pinA, uint8_t motor0_pinB, bool rev0,
             uint8_t motor1_pinA, uint8_t motor1_pinB, bool rev1,
             uint8_t motor2_pinA, uint8_t motor2_pinB, bool rev2,
             uint8_t motor3_pinA, uint8_t motor3_pinB, bool rev3,
             uint16_t drive_pulse, float wheelRadius)
:   _motorPIN{{motor0_pinA, motor0_pinB}, 
              {motor1_pinA, motor1_pinB},
              {motor2_pinA, motor2_pinB},
              {motor3_pinA, motor3_pinB}},
    _drive_pulse(drive_pulse),
    _wheelRadius(wheelRadius)
    {
        for(int i = 0; i < 4; i++) {
            pinMode(_motorPIN[i][0], OUTPUT);
            pinMode(_motorPIN[i][1], OUTPUT);
        }
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