#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <math.h>
#include "Encoder.h"
#include "BNO055.h"

class Odometry {
public:

    // wheel_Distance: 車輪の直線経路からロボット中心までの距離(mm)
    Odometry(Encoder* encoder, BNO055* bno055, 
             float wheelx_Distance, float wheely_Distance);

    // 初期化
    void begin();
    void update();

    // グローバル座標系取得
    float getX();
    float getY();
    float getYAW();
    void getAll(float* X, float* Y, float* YAW);

    // 移動距離計測用、２回の呼び出しの間にどれだけ移動したか
    // unitには0~3の区別をつける。
    float measure(uint8_t unit);
    
    // だんだん方位がずれるため、停止時に呼び出して補正
    void fixYAW();

private:
    Encoder* _encoder;
    BNO055* _bno;
    
    const float _wheelx_Distance;
    const float _wheely_Distance;
    
    float _current_X = 0.0f; // X position(mm)
    float _current_Y = 0.0f; // Y position(mm)
    float _current_YAW = 0.0f; // Orientation angle in radians
    float _prev_YAW = 0.0f; // Previous yaw angle in radians
    float driveDistance[4] = {0.0f}; // 移動距離を保存
    
};



#endif