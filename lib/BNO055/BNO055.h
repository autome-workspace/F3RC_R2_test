#ifndef BNO055_H
#define BNO055_H

#include <Arduino.h>
#include <Wire.h>

#define BNO055_I2C_ADDR 0x28 // BNO055のI2Cアドレス

class BNO055 {
public:
    // コンストラクタ
    BNO055();

    // 初期化関数
    bool begin();

    // ヨー角を読み出す関数
    float getYaw();

private:
    // 内部で使用する関数
    void _writeRegister(byte reg, byte value);
    void _readRegister(byte reg, byte* data, int len);
};

#endif