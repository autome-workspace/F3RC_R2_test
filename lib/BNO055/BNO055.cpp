#include "BNO055.h"

#define BNO055_CALIBRATION_DATA_SIZE 22 // 22バイトのキャリブレーションデータ
#define EEPROM_OFFSET 0 // EEPROMの先頭アドレスから保存

BNO055::BNO055() {
    EEPROM.begin(BNO055_CALIBRATION_DATA_SIZE);
}

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
        delay(300);
        // 初期化が成功したら、現在のヨー角を読み取ってオフセットとして保存
        byte data[2];
        _readRegister(0x1A, data, 2);
        int16_t yawRaw = (data[1] << 8) | data[0];
        _yawOffsetRaw = yawRaw;
        return true;
    }
    return false;
}

// 起動時裏返しを検出すると、キャリブレーション開始
void BNO055::isFliped() {
    delay(50);
    if (getAccZ() < -8.0) {
        Serial.println("\n\n裏返しが検出されました。キャリブレーションしない場合、５秒以内に表向きにしてください");
        Serial.printf("0 _ _ 1 _ _ 2 _ _ 3 _ _ 4 _ _ 5\n|");
        for (int i = 0; i < 10; i++) {
            delay(500);
            Serial.printf("**|");
            if (getAccZ() > 5.0) return;
        }
        Serial.println("");
        if (getAccZ() < -8.0) {
            Serial.println("裏返しのままのため、キャリブレーションを開始します。");
            Serial.println("６面静止、８時の字に動かすなどしてキャリブレーション度を高めてください。");
            Serial.println("*******************************");
            _writeRegister(0x3F, 0x20); // RST_SYS ビットを1に設定(BNO055初期化)
            delay(1000);
            _writeRegister(0x3D, 0x0C); // NDOFモードに戻す
            delay(20);
            while (!saveCalibration()) {
                int sys, gyr, acc, mag;
                getCalibrationStatus(&sys, &gyr, &acc, &mag);
                Serial.printf("sys:%d  gyr:%d  acc:%d  mag:%d\n", sys, gyr, acc, mag);
                delay(800);
            }
            Serial.println("保存しました。再起動してください");
            while (1) delay(1000);
        }
    }
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

float BNO055::getAccZ() {
    // Z軸加速度レジスタは2バイト（LSBとMSB）
    byte data[2];
    // Z軸加速度のレジスタアドレス（データシート参照）
    _readRegister(0x0C, data, 2);

    // 16ビットの符号付き整数に変換
    int16_t accZRaw = (data[1] << 8) | data[0]; 

    // 単位変換（データシートによると100LSB/g)
    return (accZRaw/ 100.0f);
}

void BNO055::_writeRegister(byte reg, byte value) {
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void BNO055::_readRegister(byte reg, byte* data, uint8_t len) {
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BNO055_I2C_ADDR, (uint8_t)len);
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

// キャリブレーションデータをEEPROMから読み込む
bool BNO055::loadCalibration() {
    bno055_offsets_t calibData;
    EEPROM.get(EEPROM_OFFSET, calibData);
    
    // データが有効か簡単なチェック
    // デフォルト値(0xFF)が残っていないことを確認
    if (calibData.accel_radius == -1 || calibData.mag_radius == -1) {
      return false;
    }

    _setSensorOffsets(calibData);
    return true;
}

// キャリブレーションデータをEEPROMに保存する
bool BNO055::saveCalibration() {
    int system, gyro, accel, mag;
    getCalibrationStatus(&system, &gyro, &accel, &mag);

    // すべてのセンサーが完全にキャリブレーションされているか確認
    if (system == 3 && gyro == 3 && accel == 3 && mag == 3) {
        bno055_offsets_t calibData;
        _getSensorOffsets(calibData);
        EEPROM.put(EEPROM_OFFSET, calibData);
        EEPROM.commit();
        return true;
    }
    return false;
}

// センサーからキャリブレーションデータを読み取るプライベート関数
bool BNO055::_getSensorOffsets(bno055_offsets_t &calibData) {
    // 1. NDOFモードを一時的にCONFIGモードに変更
    _writeRegister(0x3D, 0x00);
    delay(30);

    // 2. オフセットレジスタから22バイトのデータを読み取る
    byte data[BNO055_CALIBRATION_DATA_SIZE];
    _readRegister(0x55, data, BNO055_CALIBRATION_DATA_SIZE);

    // 3. 読み取ったバイトを構造体にマッピング
    calibData.accel_offset_x = (int16_t)((data[1] << 8) | data[0]);
    calibData.accel_offset_y = (int16_t)((data[3] << 8) | data[2]);
    calibData.accel_offset_z = (int16_t)((data[5] << 8) | data[4]);
    calibData.mag_offset_x = (int16_t)((data[7] << 8) | data[6]);
    calibData.mag_offset_y = (int16_t)((data[9] << 8) | data[8]);
    calibData.mag_offset_z = (int16_t)((data[11] << 8) | data[10]);
    calibData.gyro_offset_x = (int16_t)((data[13] << 8) | data[12]);
    calibData.gyro_offset_y = (int16_t)((data[15] << 8) | data[14]);
    calibData.gyro_offset_z = (int16_t)((data[17] << 8) | data[16]);
    calibData.accel_radius = (int16_t)((data[19] << 8) | data[18]);
    calibData.mag_radius = (int16_t)((data[21] << 8) | data[20]);
    
    // 4. 再度NDOFモードに戻す
    _writeRegister(0x3D, 0x0C);
    delay(30);

    return true;
}

// センサーにキャリブレーションデータを書き込むプライベート関数
void BNO055::_setSensorOffsets(const bno055_offsets_t &calibData) {
    // 1. NDOFモードを一時的にCONFIGモードに変更
    _writeRegister(0x3D, 0x00);
    delay(30);

    // 2. オフセットレジスタにデータを書き込む
    _writeRegister(0x55, (uint8_t)(calibData.accel_offset_x & 0xFF));
    _writeRegister(0x56, (uint8_t)(calibData.accel_offset_x >> 8));
    _writeRegister(0x57, (uint8_t)(calibData.accel_offset_y & 0xFF));
    _writeRegister(0x58, (uint8_t)(calibData.accel_offset_y >> 8));
    _writeRegister(0x59, (uint8_t)(calibData.accel_offset_z & 0xFF));
    _writeRegister(0x5A, (uint8_t)(calibData.accel_offset_z >> 8));
    _writeRegister(0x5B, (uint8_t)(calibData.mag_offset_x & 0xFF));
    _writeRegister(0x5C, (uint8_t)(calibData.mag_offset_x >> 8));
    _writeRegister(0x5D, (uint8_t)(calibData.mag_offset_y & 0xFF));
    _writeRegister(0x5E, (uint8_t)(calibData.mag_offset_y >> 8));
    _writeRegister(0x5F, (uint8_t)(calibData.mag_offset_z & 0xFF));
    _writeRegister(0x60, (uint8_t)(calibData.mag_offset_z >> 8));
    _writeRegister(0x61, (uint8_t)(calibData.gyro_offset_x & 0xFF));
    _writeRegister(0x62, (uint8_t)(calibData.gyro_offset_x >> 8));
    _writeRegister(0x63, (uint8_t)(calibData.gyro_offset_y & 0xFF));
    _writeRegister(0x64, (uint8_t)(calibData.gyro_offset_y >> 8));
    _writeRegister(0x65, (uint8_t)(calibData.gyro_offset_z & 0xFF));
    _writeRegister(0x66, (uint8_t)(calibData.gyro_offset_z >> 8));
    _writeRegister(0x67, (uint8_t)(calibData.accel_radius & 0xFF));
    _writeRegister(0x68, (uint8_t)(calibData.accel_radius >> 8));
    _writeRegister(0x69, (uint8_t)(calibData.mag_radius & 0xFF));
    _writeRegister(0x6A, (uint8_t)(calibData.mag_radius >> 8));

    // 3. 再度NDOFモードに戻す
    _writeRegister(0x3D, 0x0C);
    delay(30);
}