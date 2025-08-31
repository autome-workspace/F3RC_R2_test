#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <ESP32Encoder.h>

class Encoder {
public:
    // コンストラクタ: 2つのエンコーダのA相/B相ピンを受け取る
    Encoder(uint8_t encoder1_pinA, uint8_t encoder1_pinB, 
            uint8_t encoder2_pinA, uint8_t encoder2_pinB, 
            uint16_t pulse, float wheelRadius);
    
    // 初期化
    void begin();
    
    // 1番目のエンコーダの値をリセット
    void resetEncoder1();
    
    // 2番目のエンコーダの値をリセット
    void resetEncoder2();
    
    // 1番目のエンコーダの変位を取得
    float getEncoder1Displacement();
    
    // 2番目のエンコーダの変位を取得
    float getEncoder2Displacement();

private:
    const uint16_t _pulse; // エンコーダ一周のパルス数(パルス/回転)
    const float _wheelRadius; // エンコーダに取り付けられた車輪の半径(mm)
    const float _k; // エンコーダのパルス数から距離への変換係数
    ESP32Encoder encoder1;
    ESP32Encoder encoder2;

    long _prevEncoder1Count; // 前回の1番目のエンコーダのカウント値
    long _prevEncoder2Count; // 前回の2番目のエンコーダのカウント値
};

#endif