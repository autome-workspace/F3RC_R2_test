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

    BNO055(uint8_t I2C_SDA, uint8_t I2C_SCL);
    bool begin();
    bool changeToCONFIG();
    bool changeToNDOF();
    bool changeToIMU();
    void isFliped();
    float getYAW();
    float getAccZ();
    uint8_t getOperatingMode();
    void fixYAW();
    void getCalibrationStatus(int* sys, int* gyr, int* acc, int* mag);
    bool loadCalibration(); // キャリブレーションデータを読み込む関数
    bool saveCalibration(); // キャリブレーションデータを保存する関数
    bool _getSensorOffsets(bno055_offsets_t &calibData);
    void _setSensorOffsets();
    
private:
    bool _writeRegister(byte reg, byte value);
    bool _readRegister(byte reg, byte* data, uint8_t len);
    void i2cReset();
    int16_t _yawOffsetRaw; // ヨー角のオフセット値
    bno055_offsets_t calibData;
    uint8_t SDA, SCL;
    int16_t last_YAW = 0;

};

#endif