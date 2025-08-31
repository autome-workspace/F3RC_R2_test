#include "Encoder.h"

Encoder::Encoder(uint8_t encoder1_pinA, uint8_t encoder1_pinB, 
                 uint8_t encoder2_pinA, uint8_t encoder2_pinB, 
                 uint16_t pulse, float wheelRadius)
:   encoder1(), 
    encoder2(), 
    _pulse(pulse),
    _wheelRadius(wheelRadius),
    _k(2 * M_PI * _wheelRadius / _pulse), // 2πr/pulse
    _prevEncoder1Count(0),
    _prevEncoder2Count(0)
{
    encoder1.attachFullQuad(encoder1_pinA, encoder1_pinB);
    encoder2.attachFullQuad(encoder2_pinA, encoder2_pinB);
     
}

void Encoder::begin() {
    _prevEncoder1Count = encoder1.getCount();
    _prevEncoder2Count = encoder2.getCount();
}

void Encoder::resetEncoder1() {
    // 1番目のエンコーダのカウント値を0にリセットする
    encoder1.setCount(0);
    _prevEncoder1Count = 0;
}

void Encoder::resetEncoder2() {
    // 2番目のエンコーダのカウント値を0にリセットする
    encoder2.setCount(0);
    _prevEncoder2Count = 0;
}

float Encoder::getEncoder1Displacement() {
    // 1番目のエンコーダの現在の移動量を取得する
    long currentCount = encoder1.getCount();
    long deltaCount = currentCount - _prevEncoder1Count;
    _prevEncoder1Count = currentCount;
    return _k * deltaCount;
}

float Encoder::getEncoder2Displacement() {
    // 2番目のエンコーダの現在の移動量を取得する
    long currentCount = encoder2.getCount();
    long deltaCount = currentCount - _prevEncoder2Count;
    _prevEncoder2Count = currentCount;
    return _k * deltaCount;
}