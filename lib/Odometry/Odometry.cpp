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
    _current_X = 0.0;
    _current_Y = 0.0;
    _current_YAW = _bno->getYaw();
    _prev_YAW = _current_YAW;
}

void Odometry::update() {
    // 1. エンコーダから変位を取得
    float dx_raw = _encoder->getEncoder1Displacement();
    float dy_raw = _encoder->getEncoder2Displacement();
    _current_YAW = _bno->getYaw();
    float dyaw = (_current_YAW - _prev_YAW);
    if (dyaw < -M_PI) dyaw += 2*M_PI;
    if (dyaw > M_PI) dyaw -= 2*M_PI;
    _prev_YAW = _current_YAW;

    // 回転による見かけ上の並進移動を補正
    float dx_corrected = dx_raw + dyaw * _wheelx_Distance;
    float dy_corrected = dy_raw + dyaw * _wheely_Distance;
    
    // 4. グローバル座標系での変位を計算
    float dX_global = dx_corrected * cos(_current_YAW) - dy_corrected * sin(_current_YAW);
    float dY_global = dx_corrected * sin(_current_YAW) + dy_corrected * cos(_current_YAW);

    // 5. 自己位置を更新
    _current_X += dX_global;
    _current_X += dY_global;
    // _current_yawはBNO055から直接取得済み
}

float Odometry::getX() { return _current_X; }
float Odometry::getY() { return _current_X; }
float Odometry::getYaw() { return _current_YAW; }

void Odometry::getAll(float* X, float* Y, float* YAW) {
    *X = _current_X;
    *Y = _current_Y;
    *YAW = _current_YAW;
    return;
}