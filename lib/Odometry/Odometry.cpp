#include "Odometry.h"
#include <math.h>

Odometry::Odometry(Encoder* encoder, BNO055* bno, 
                   float wheelx_Distance, float wheely_Distance)                   
:   _encoder(encoder),
    _bno(bno),
    _wheelx_Distance(wheelx_Distance),
    _wheely_Distance(wheely_Distance)
{}
void Odometry::begin() {
    _current_x = 0.0;
    _current_y = 0.0;
    _current_yaw = 0.0;
}

void Odometry::update() {
    // 1. エンコーダから変位を取得
    float dx_raw = _encoder->getEncoder1Displacement();
    float dy_raw = _encoder->getEncoder2Displacement();
    _current_yaw = _bno->getYaw();
    float dyaw = (_current_yaw - _prev_yaw);
    _prev_yaw = _current_yaw;

    // 回転による見かけ上の並進移動を補正
    float dx_corrected = dx_raw - dyaw * _wheelx_Distance;
    float dy_corrected = dy_raw - dyaw * _wheely_Distance;
    
    // 4. グローバル座標系での変位を計算
    float dx_global = dx_corrected * cos(_current_yaw) - dy_corrected * sin(_current_yaw);
    float dy_global = dx_corrected * sin(_current_yaw) + dy_corrected * cos(_current_yaw);

    // 5. 自己位置を更新
    _current_x += dx_global;
    _current_y += dy_global;
    // _current_yawはBNO055から直接取得済み
}

float Odometry::getX() { return _current_x; }
float Odometry::getY() { return _current_y; }
float Odometry::getYaw() { return _current_yaw; }