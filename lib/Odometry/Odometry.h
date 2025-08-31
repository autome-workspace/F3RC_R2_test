#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "Encoder.h"

class Odometry {
public:
    //コンストラクタ
    Odometry(float wheel1_x, float wheel1_y, float wheel2_x, float wheel2_y);

    //初期化
    void begin();


    void odometrySetWheel(float x, float y, float r);
    void odometrySetWheel(uint8_t wheelNumber, float wheelDiameter);
    void odometrySetMax(float ODOMETRY_ACC_MAX);
    void odometrySetLimit(float odometryAccLimit);
    
    void odometryUpdate();
    float getX();
    float getY();
    float getTheta();

private:
    float _wheelDiameter[2]; // 0: left wheel, 1: right wheel
    float _wheelDistance; // Distance between the two wheels
    float _ODOMETRY_ACC_MAX;
    float _odometryAccLimit;

    float _x = 0.0; // X position
    float _y = 0.0; // Y position
    float _theta = 0.0; // Orientation angle in radians
};



#endif