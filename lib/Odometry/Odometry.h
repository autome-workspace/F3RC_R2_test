#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

class Odometry {
public:



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