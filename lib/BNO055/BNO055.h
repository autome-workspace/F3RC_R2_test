#ifndef BNO055_H
#define BNO055_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define BNO055_I2C_ADDR 0x28 // BNO055のI2Cアドレス

// キャリブレーションデータを格納する構造体
typedef struct {
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
  int16_t mag_offset_x;
  int16_t mag_offset_y;
  int16_t mag_offset_z;
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;
  int16_t accel_radius;
  int16_t mag_radius;
} bno055_offsets_t;

class BNO055 {
public:
    // コンストラクタ
    BNO055();

    // 初期化関数
    bool begin();

    // キャリブレーションフラグ
    void isFliped();

    // ヨー角を読み出す関数
    float getYaw();
    //float getRawYaw();

    float getAccZ();

    // SYS (システム), GYR (ジャイロ), ACC (加速度), MAG (磁力計)
    void getCalibrationStatus(int* sys, int* gyr, int* acc, int* mag);

    bool loadCalibration(); // キャリブレーションデータを読み込む関数
    bool saveCalibration(); // キャリブレーションデータを保存する関数

private:
    void _writeRegister(byte reg, byte value);
    void _readRegister(byte reg, byte* data, uint8_t len);
    int16_t _yawOffsetRaw; // ヨー角のオフセット値

    // キャリブレーションデータを取得・設定するプライベートヘルパー関数
    bool _getSensorOffsets(bno055_offsets_t &calibData);
    void _setSensorOffsets(const bno055_offsets_t &calibData);
};

#endif