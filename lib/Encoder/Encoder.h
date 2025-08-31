#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <ESP32Encoder.h>

class Encoder {
public:
    // コンストラクタ: 2つのエンコーダのA相/B相ピンを受け取る
    Encoder(int encoder1_pinA, int encoder1_pinB, int encoder2_pinA, int encoder2_pinB);
    
    // ライブラリの初期化
    void begin();
    
    // 1番目のエンコーダの値をリセット
    void resetEncoder1();
    
    // 2番目のエンコーダの値をリセット
    void resetEncoder2();
    
    // 1番目のエンコーダの現在値を取得
    long getEncoder1Value();
    
    // 2番目のエンコーダの現在値を取得
    long getEncoder2Value();

private:
    ESP32Encoder encoder1;
    ESP32Encoder encoder2;
};

#endif