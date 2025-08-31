#include "BNO055.h"

BNO055::BNO055() {}

bool BNO055::begin() {

    // レジスタアドレスやモード設定はデータシートを参照
    // 1. CONFIGモードに設定
    _writeRegister(0x3D, 0x00);
    delay(20);

    // 2. NDOFモードに設定
    _writeRegister(0x3D, 0x0C);
    delay(20);

    // BNO055の接続確認
    byte chip_id;
    _readRegister(0x00, &chip_id, 1);
    
    if (chip_id == 0xA0) {
        // 初期化が成功したら、現在のヨー角を読み取ってオフセットとして保存
        byte data[2];
        _readRegister(0x1A, data, 2);
        int16_t yawRaw = (data[1] << 8) | data[0];
        _yawOffsetRaw = yawRaw;
        return true;
    }
    return false;
}

float BNO055::getYaw() {
    // ヨー角レジスタは2バイト（LSBとMSB）
    byte data[2];
    // ヨー角のレジスタアドレス（データシート参照）
    _readRegister(0x1A, data, 2); 

    // 16ビットの符号付き整数に変換
    int16_t yawRaw = (data[1] << 8) | data[0]; 

    // 単位変換（データシートによると16LSB/度）さらにラジアンに変換
    return ((yawRaw - _yawOffsetRaw)/ 16.0f / 360.0 * 2 * M_PI);
}

void BNO055::_writeRegister(byte reg, byte value) {
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void BNO055::_readRegister(byte reg, byte* data, int len) {
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(BNO055_I2C_ADDR, (uint8_t)len);
    for (int i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
}

// センサーのキャリブレーション度を取得する
// 各センサーのキャリブレーション状態を0〜3で返す
void BNO055::getCalibrationStatus(int* sys, int* gyr, int* acc, int* mag) {
    byte calibStatus;
    _readRegister(0x35, &calibStatus, 1);

    // ビットシフトとマスク処理で各センサーの値を抽出
    // データシートによると、レジスタのビット配置は以下の通り
    // [7:6] SYSTEM, [5:4] GYRO, [3:2] ACCEL, [1:0] MAG
    
    *sys = (calibStatus >> 6) & 0x03; // 上位2ビットをシステムとして抽出
    *gyr = (calibStatus >> 4) & 0x03; // 次の2ビットをジャイロとして抽出
    *acc = (calibStatus >> 2) & 0x03; // 次の2ビットを加速度計として抽出
    *mag = calibStatus & 0x03;      // 下位2ビットを磁力計として抽出
}