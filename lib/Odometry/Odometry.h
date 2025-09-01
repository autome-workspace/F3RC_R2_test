#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <math.h>
#include "Encoder.h"
#include "BNO055.h"

class Odometry {
public:
    // コンストラクタ 1はｘ軸に垂直、2はy軸に平行
    // wheel_Distance: 車輪の直線経路からロボット中心までの距離(mm)
    Odometry(Encoder* encoder, BNO055* bno055, 
             float wheelx_Distance, float wheely_Distance);

    // 初期化
    void begin();
    void update();

    float getX();
    float getY();
    float getYaw();
    void getAll(float* X, float* Y, float* YAW);

private:
    const float _wheelx_Distance;
    const float _wheely_Distance;

    Encoder* _encoder;
    BNO055* _bno;

    float _current_X = 0.0; // X position(mm)
    float _current_Y = 0.0; // Y position(mm)
    float _current_YAW = 0.0; // Orientation angle in radians
    float _prev_YAW = 0.0; // Previous yaw angle in radians
};



#endif