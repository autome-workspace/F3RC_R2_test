#include "Drive.h"
#include <math.h>

Drive::Drive(Odometry* odometry, int16_t DRIVE_PULSE, int16_t PWM_MAX)
:   _odometry{odometry}, 
    PULSE(DRIVE_PULSE),
    _PWM_MAX(PWM_MAX) {}

void Drive::driveSetMecanumX(float DRIVE_WHEEL_X, float DRIVE_WHEEL_Y) {
    _DRIVE_WHEEL_X = DRIVE_WHEEL_X;
    _DRIVE_WHEEL_Y = DRIVE_WHEEL_Y;
    _DRIVE_WHEEL_DISTANCE = 0.0f;
    _DRIVE_WHEEL_NUMBER = 4;
    MODE = 1;
}

void Drive::driveSetMecanumO(float DRIVE_WHEEL_X, float DRIVE_WHEEL_Y) {
    _DRIVE_WHEEL_X = DRIVE_WHEEL_X;
    _DRIVE_WHEEL_Y = DRIVE_WHEEL_Y;
    _DRIVE_WHEEL_DISTANCE = 0.0f;
    _DRIVE_WHEEL_NUMBER = 4;
    MODE = 2;
}

void Drive::driveSetOmni(float DRIVE_WHEEL_DISTANCE, float DRIVE_WHEEL_NUMBER) {
    _DRIVE_WHEEL_X = 0.0f;
    _DRIVE_WHEEL_Y = 0.0f;
    _DRIVE_WHEEL_DISTANCE = DRIVE_WHEEL_DISTANCE;
    _DRIVE_WHEEL_NUMBER = DRIVE_WHEEL_NUMBER;
    MODE = DRIVE_WHEEL_NUMBER;
}


void Drive::driveSetMax( float MOTOR_SPD_MAX, float MOTOR_ACC_MAX) {
    _MOTOR_SPD_MAX = MOTOR_SPD_MAX;
    _MOTOR_ACC_MAX = MOTOR_ACC_MAX;
}

void Drive::driveSetLinearLimit(float ROBOT_LSPD_LIMIT, float ROBOT_LACC_LIMIT) {
    _ROBOT_LSPD_LIMIT = ROBOT_LSPD_LIMIT;
    _ROBOT_LACC_LIMIT = ROBOT_LACC_LIMIT;
 
}

void Drive::driveSetTurnLimit(float ROBOT_TSPD_LIMIT, float ROBOT_TACC_LIMIT) {
    _ROBOT_TSPD_LIMIT = ROBOT_TSPD_LIMIT;
    _ROBOT_TACC_LIMIT = ROBOT_TACC_LIMIT;
}

bool Drive::drive(float targetX, float targetY, float targetYAW) {

    // 自身のグローバル座標系 - 速度
    float cv_squared = cv_X * cv_X + cv_Y * cv_Y; // current v
    float cv = sqrtf(cv_squared);
    float cv_RAD = atan2f(cv_Y, cv_X);

    // 自身のグローバル座標取得
    _odometry->getAll(&cX, &cY, &cYAW);

    // グローバル座標系 - 目標変位
    float dX = targetX - cX;
    float dY = targetY - cY;
    float dRAD = atan2(dY, dX);
    float dL = sqrtf(dX * dX + dY * dY);

    float spd_left;
    float acc_left;
    float ratio;

    // まだ目標地点じゃないなら
    if (dL > distanceTolerance) {

        // 方向ごとのの限界値設定(機体の進行方向によって異なる)
        float K = K_rad(dRAD - cYAW);
        float SPD_LIMIT = K*_MOTOR_SPD_MAX;
        float ACC_LIMIT = K*_MOTOR_ACC_MAX;
        
        // 制限を超える場合(ついでにモーターの余裕を保存)
        spd_left = (SPD_LIMIT - _ROBOT_LSPD_LIMIT) / K;
        acc_left = (ACC_LIMIT - _ROBOT_LACC_LIMIT) / K;
        if (spd_left > 0) SPD_LIMIT = _ROBOT_LSPD_LIMIT;
        if (acc_left > 0) ACC_LIMIT = _ROBOT_LACC_LIMIT;

        float target_v = sqrtf(dL * 2 * ACC_LIMIT); // v^2 / 2a = L 停止距離
        if (target_v > SPD_LIMIT) target_v = SPD_LIMIT;
        float target_RAD = dRAD;
        float target_v_X = target_v * -sinf(target_RAD);
        float target_v_Y = target_v * cosf(target_RAD);

        // もし接近速度が非常識なら
        if ( cv * cosf(target_RAD - cv_RAD) ) {
        // 速度ベクトルの、目標速度ベクトルに平行な成分を求める。
        // このとき、目標速度を超越している場合、
        // それは目標地点を通り過ぎるほどの速さということになる。

            // 目標と反対方向に全力減速
            cv_X -= ACC_LIMIT * -sinf(target_RAD) / PULSE;
            cv_Y -= ACC_LIMIT * cosf(target_RAD) / PULSE;
        
        //　接近速度が常識の範囲内なら
        } else {
        // 速度ベクトルを目標値に合わせていく

            // 速度差
            float tc_v_X = target_v_X - cv_X;
            float tc_v_Y = target_v_Y - cv_Y;
            float tc_v_L = sqrtf(tc_v_X * tc_v_X + tc_v_Y * tc_v_Y);

            //　加速度制限の範囲内で現在の速度を変更
            ratio = tc_v_L * PULSE / ACC_LIMIT;
            if (ratio > 1) {
                cv_X += tc_v_X / ratio;
                cv_Y += tc_v_Y / ratio;
                drive_status = 1;
            } else {
                cv_X = target_v_X;
                cv_Y = target_v_Y;
                
                // もし現在の速度が最大なら
                if (target_v == SPD_LIMIT) {
                    drive_status = 2;
                }

            }

        }

        cv_squared = cv_X * cv_X + cv_Y * cv_Y;
        cv = sqrtf(cv_squared);
        cv_RAD = atan2f(cv_Y, cv_X);

    // 目標地点だったら
    } else {

        spd_left = _MOTOR_SPD_MAX;
        acc_left = _MOTOR_ACC_MAX;

    }

    // ここから回転関連 rを各加速度とする

    if (spd_left > 0 && acc_left > 0) {
        
        float w_limit, r_limit;
        if (MODE = 1) {
            // メカナムX
            w_limit = spd_left / (_DRIVE_WHEEL_X + _DRIVE_WHEEL_Y);
            r_limit = acc_left / (_DRIVE_WHEEL_X + _DRIVE_WHEEL_Y);
        } else {
            // オムニ
            w_limit = spd_left / _DRIVE_WHEEL_DISTANCE;
            r_limit = acc_left / _DRIVE_WHEEL_DISTANCE;
        }

        float target_w = sqrtf(abs(targetYAW - cYAW) * 2 * r_limit);
        if (targetYAW - cYAW < M_PI || targetYAW - cYAW > -M_PI) target_w *= -1;

        float dw = target_w - cw;

        ratio = abs(dw) * PULSE / r_limit;

        if (ratio > 1) {
            cw += dw / ratio;
        } else {
            cw = target_w;
        }

        if (abs(cw) > w_limit) cw *= w_limit / cw;

    }

    motorWrite(cv, cv_RAD - cYAW, cw);

    if( dL < distanceTolerance && abs(targetYAW - cYAW) < 0.1) {
        return true; // 移動完了
    }
    return false; // 移動未完了
    

}

void Drive::motorWrite(float v, float rad, float w) {
    for (int8_t i=0; i < _DRIVE_WHEEL_NUMBER; i++) {
        int16_t PWM = 0;
        if (MODE = 1) { // メカナムX
            PWM = _PWM_MAX / _MOTOR_SPD_MAX * v * (sinf(rad) + ((i%2)*2-1) * cosf(rad));

        } else if (MODE == 2) { // メカナムO　必要なし
            PWM = 0;  
        } else if (MODE >= 3) {
            // オムニn輪
            PWM = _PWM_MAX / _MOTOR_SPD_MAX * (v * sinf(rad + i * 2*M_PI / _DRIVE_WHEEL_NUMBER) + _DRIVE_WHEEL_DISTANCE);
        
        }

        if (PWM > 0) {
            ledcWrite(i, 1023 - PWM);
            ledcWrite(i + 4, 1023);
        } else {
            ledcWrite(i, 1023);
            ledcWrite(i + 4, 0);
        }
    }
}

float Drive::K_rad(float rad) {
    // 機体から見た各方向への速度、加速度比(最高速時を１としてそれ以下)

    float K = 0;
    if (MODE == 1) { // メカナム, 範囲はひし形 (上)
        float K = 1 / (abs(sinf(rad)) + abs(cosf(rad)));

    } else if (MODE == 3) { // オムニ３輪基準, 範囲は六角形 (下)
        float K = 1 / (cosf(fmodf(rad, M_PI/3)) + sinf(fmodf(rad, M_PI/3))/sqrtf(3.0f));

    } else if (MODE == 4) { // オムニ４輪, 範囲はひし形 (上)
        float K = 1 / (abs(sinf(rad+M_PI/2)) + abs(cosf(rad+M_PI/2)));

    }
    return K;
}