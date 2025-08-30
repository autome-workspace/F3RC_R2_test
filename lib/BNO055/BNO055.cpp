#include "BNO055.h"

BNO055::BNO055() {}

bool BNO055::begin() {
    Wire.begin(); // I2C通信を開始

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
    return (chip_id == 0xA0); // 正しいチップIDか確認
}

float BNO055::getYaw() {
    // ヨー角レジスタは2バイト（LSBとMSB）
    byte data[2];
    // ヨー角のレジスタアドレス（データシート参照）
    _readRegister(0x1A, data, 2); 

    // 16ビットの符号付き整数に変換
    int16_t yawRaw = (data[1] << 8) | data[0]; 

    // 単位変換（データシートによると16LSB/度）
    float yaw = yawRaw / 16.0f;
    return yaw;
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
    Wire.endTransmission();

    Wire.requestFrom(BNO055_I2C_ADDR, len);
    for (int i = 0; i < len; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        }
    }
}