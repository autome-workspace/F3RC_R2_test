#include "Encoder.h"

Encoder::Encoder(int encoder1_pinA, int encoder1_pinB, int encoder2_pinA, int encoder2_pinB)
: encoder1(), encoder2() 
{
    encoder1.attachHalfQuad(encoder1_pinA, encoder1_pinB);
    encoder2.attachHalfQuad(encoder2_pinA, encoder2_pinB);
}

void Encoder::begin() {
    // PCNTライブラリはインスタンス化とattachHalfQuad()で初期化が完了するため空でよい
}

void Encoder::resetEncoder1() {
    // 1番目のエンコーダのカウント値を0にリセットする
    encoder1.setCount(0);
}

void Encoder::resetEncoder2() {
    // 2番目のエンコーダのカウント値を0にリセットする
    encoder2.setCount(0);
}

long Encoder::getEncoder1Value() {
    // 1番目のエンコーダの現在のカウント値を取得する
    return encoder1.getCount();
}

long Encoder::getEncoder2Value() {
    // 2番目のエンコーダの現在のカウント値を取得する
    return encoder2.getCount();
}