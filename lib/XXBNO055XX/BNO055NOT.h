#ifndef BNO055_H
#define BNO055_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define BNO055_I2C_ADDR 0x28 // BNO055のI2Cアドレス

class BNO055 {
public:
    // コンストラクタ
    BNO055();

    // 初期化関数
    bool begin();

    // ヨー角を読み出す関数
    float getYaw();
    float getRawYaw();

    // SYS (システム), GYR (ジャイロ), ACC (加速度), MAG (磁力計)
    void getCalibrationStatus(int* sys, int* gyr, int* acc, int* mag);

    bool loadCalibration(); // キャリブレーションデータを読み込む関数
    bool saveCalibration(); // キャリブレーションデータを保存する関数

private:
    void _writeRegister(byte reg, byte value);
    void _readRegister(byte reg, byte* data, int len);
    int16_t _yawOffsetRaw; // ヨー角のオフセット値
};

#endif