#include "Drive.h"
#include <math.h>

Drive::Drive(Odometry* odometry, uint16_t DRIVE_PULSE, uint8_t PWM_MAX)
:   _odometry{odometry}, 
    _DRIVE_PULSE(DRIVE_PULSE),
    _PWM_MAX(PWM_MAX) {}

void Drive::driveSetMax( float MOTOR_SPD_MAX, float MOTOR_ACC_MAX) {
    _MOTOR_SPD_MAX = MOTOR_SPD_MAX;
    _MOTOR_ACC_MAX = MOTOR_ACC_MAX;
}

void Drive::driveSetLimit(float ROBOT_SPD_LIMIT, float ROBOT_ACC_LIMIT) {
    _ROBOT_SPD_LIMIT = ROBOT_SPD_LIMIT;
    _ROBOT_ACC_LIMIT = ROBOT_ACC_LIMIT;
}

bool Drive::drive(float targetX, float targetY, float targetYAW) {

    // グローバル座標取得
    float cX, cY,  cYAW; // current
    _odometry->getAll(&cX, &cY, &cYAW);

    // グローバル座標系で見た目標位置
    float dX = targetX - cX;
    float dY = targetY - cY;
    float dRAD = atan2(dY, dX);

    // 機体座標系で見た目標位置
    float dx = dX*cosf(cYAW) + dY*sinf(cYAW);
    float dy = -dX*sinf(cYAW) + dY*cosf(cYAW);
    float drad = dRAD - cYAW;

    // cvx, cvy 現在の速度
    // 機体から見た各方向への速度、加速度比(最高速時を１としてそれ以下)
    // メカナム,4輪オムニ基準, 範囲はひし形 (上) | オムニ３輪基準, 範囲は六角形 (下)
    float K = 1 / (abs(sinf(drad)) + abs(cosf(drad)));
    //float K = 1 / (cosf(fmodf(drad, M_PI/3)) + sinf(fmodf(drad, M_PI/3)/sqrtf(3.0f));

    // 方向ごとのの限界値設定
    float SPD_LIMIT = K*_MOTOR_SPD_MAX, ACC_LIMIT = K*_MOTOR_ACC_MAX;
    
    // 制限を超える場合(ついでに余裕を保存)
    float spd_left = SPD_LIMIT - _ROBOT_SPD_LIMIT;
    float acc_left = ACC_LIMIT - _ROBOT_ACC_LIMIT;
    if (spd_left > 0) SPD_LIMIT = _ROBOT_SPD_LIMIT;
    if (acc_left > 0) ACC_LIMIT = _ROBOT_ACC_LIMIT;

    float cv_squared = cvx*cvx + cvy*cvy; // current v
    float cv = sqrtf(cv_squared);
    float p = SPD_LIMIT / cv;

    // 止まれる距離でない場合、頑張って減速
    if (cv_squared / (2*ACC_LIMIT) > sqrtf(dX*dX + dY*dY) - 10) 
        cv -= ACC_LIMIT / _DRIVE_PULSE;
        
    // 十分遠い場合、加速
    else cv += ACC_LIMIT / _DRIVE_PULSE;
    
    //　最高速に到達
    if (cv >= _ROBOT_SPD_LIMIT)
        cv = _ROBOT_SPD_LIMIT;

    // 余裕が残っている場合、方向を合わせたい
    if ((spd_left > 0 && acc_left > 0) || (cv == _ROBOT_SPD_LIMIT && acc_left > 0)) {
        if (targetYAW >= cYAW) {
        }
        
    }

    





    return false; // 移動未完了
    return true; // 移動完了

}

//bool Drive::drive(float targetX, float targetY) {return true;}
//bool Drive::drive(float targetRadians) {return true;}

void Drive::motorWrite(uint8_t motorNum, float motorSpd) {
    int PWM = _PWM_MAX * motorSpd / _MOTOR_SPD_MAX;
    if (PWM > 0) {
        ledcWrite(motorNum, PWM);
        ledcWrite(motorNum + 4, PWM);
    } else {
        ledcWrite(motorNum, 0);
        ledcWrite(motorNum + 4, PWM);
    }
}