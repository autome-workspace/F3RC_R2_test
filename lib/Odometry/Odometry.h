#ifndef ODOMETRY_LIBRARY_H
#define ODOMETRY_LIBRARY_H

#include <Arduino.h>

class OdometryLibrary {
    public:
        void odometrySetWheel(int8_t wheelNumber, float wheelDiameter, float wheelDistance);
        void odometrySetMax(float ODOMETRY_ACC_MAX);
        void odometrySetLimit(float odometryAccLimit);
        
        void odometryUpdate(float leftWheelSpeed, float rightWheelSpeed, float deltaTime);
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