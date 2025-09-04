#include "BNO055.h"

#define BNO055_CALIBRATION_DATA_SIZE 22 // 22バイトのキャリブレーションデータ
#define EEPROM_OFFSET 0 // EEPROMの先頭アドレスから保存

BNO055::BNO055(uint8_t I2C_SDA, uint8_t I2C_SCL)
:   SDA(I2C_SDA), SCL(I2C_SCL)
{
    EEPROM.begin(BNO055_CALIBRATION_DATA_SIZE);
}

bool BNO055::begin() {
    // I2Cバスの初期化をここで行う
    Wire.begin(SDA, SCL);
    
    // データシートによると、電源投入後にブートアップが完了するまで
    // 少なくとも650msかかります。ここでは少し余裕を持たせます。
    delay(800);

    // BNO055をソフトウェアリセットし、通信をクリーンな状態に保つ
    _writeRegister(0x3F, 0x20);
    delay(100);

    // BNO055の接続確認
    byte chip_id = 0;
    int retry_count = 0;
    const int max_retries = 10;
    
    // センサーIDが正しく読み取れるまでリトライ
    while(chip_id != 0xA0 && retry_count < max_retries) {
        if(_readRegister(0x00, &chip_id, 1)) {
            if (chip_id == 0xA0) {
                break; // 成功
            }
        }
        Serial.printf("BNO055 chip ID read failed. Retrying... (%d/%d)\n", retry_count + 1, max_retries);
        delay(100);
        retry_count++;
    }
    
    if (chip_id == 0xA0) {
        Serial.println("BNO055 detected!");
        
        changeToCONFIG(); // 設定モードへ
        
        // EEPROMに保存されたキャリブレーションデータを読み込む
        if(loadCalibration()) {
             Serial.println("EEPROMからキャリブレーションデータを読み込みました。");
        } else {
            Serial.println("EEPROMに有効なキャリブレーションデータが見つかりませんでした。");
        }
        
        changeToNDOF();   // NDOFモードへ
        
        // 初期化が成功したら、現在のヨー角をオフセットとして保存
        byte data[2];
        _readRegister(0x1A, data, 2);
        int16_t yawRaw = (data[1] << 8) | data[0];
        _yawOffsetRaw = yawRaw;
        last_YAW = 0.0f;
        Serial.println("Yaw offset have set");
        return true;
    }
    Serial.println("BNO055 not found!");
    return false;
}

bool BNO055::changeToCONFIG() {
    // CONFIGモードに設定
    bool error = _writeRegister(0x3D, 0x00);
    delay(10); // 推奨7ms
    return error;
}

bool BNO055::changeToNDOF() {
    // NDOFモードに設定
    bool error = _writeRegister(0x3D, 0x0C);
    delay(20); // 推奨19ms
    return error;
}

bool BNO055::changeToIMU() {
    // IMUモードに設定
    int error = _writeRegister(0x3D, 0x08);
    delay(20);
    return error;
}

// 起動時裏返しを検出すると、キャリブレーション開始
void BNO055::isFliped() {
    delay(50);
    if (getAccZ() < -8.0) {
        Serial.println("\n\n裏返しが検出されました。キャリブレーションしない場合、5秒以内に表向きにしてください");
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
            delay(800); // 約650ミリ秒
            changeToNDOF();
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
    int16_t yawRaw;
    
    // ヨー角のレジスタアドレス（データシート参照）
    if(_readRegister(0x1A, data, 2)) {
        // 16ビットの符号付き整数に変換
        yawRaw = (data[1] << 8) | data[0];
        last_YAW = yawRaw; // 保持値を更新
    } else {
        // 問題発生のため前回の値を返す
        yawRaw = last_YAW;
    }

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

bool BNO055::_writeRegister(byte reg, byte value) {
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    byte status = Wire.endTransmission();
    if (status != 0) {
        Serial.printf("I2C write failed. Status: %d\n", status);
        return false;
    }
    return true;
}

bool BNO055::_readRegister(byte reg, byte* data, uint8_t len) {
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(reg);
    byte status = Wire.endTransmission(false);
    
    if (status != 0) {
        Serial.printf("I2C read begin failed. Status: %d\n", status);
        return false;
    }
    
    uint8_t bytesReceived = Wire.requestFrom((uint8_t)BNO055_I2C_ADDR, len);
    if (bytesReceived != len) {
        Serial.printf("I2C request failed. Expected %d bytes, received %d\n", len, bytesReceived);
        return false;
    }

    for (int i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return true;
}

// センサーのキャリブレーション度を取得する
void BNO055::getCalibrationStatus(int* sys, int* gyr, int* acc, int* mag) {
    byte calibStatus;
    _readRegister(0x35, &calibStatus, 1);
    *sys = (calibStatus >> 6) & 0x03; // 上位2ビットをシステムとして抽出
    *gyr = (calibStatus >> 4) & 0x03; // 次の2ビットをジャイロとして抽出
    *acc = (calibStatus >> 2) & 0x03; // 次の2ビットを加速度計として抽出
    *mag = calibStatus & 0x03;      // 下位2ビットを磁力計として抽出

}

// キャリブレーションデータをEEPROMから読み込む
bool BNO055::loadCalibration() {
    EEPROM.get(EEPROM_OFFSET, calibData);
    if (calibData.accel_radius == -1 || calibData.mag_radius == -1) {
      return false;
    }
    _setSensorOffsets();
    return true;
}

// キャリブレーションデータをEEPROMに保存する
bool BNO055::saveCalibration() {
    int system, gyro, accel, mag;
    getCalibrationStatus(&system, &gyro, &accel, &mag);

    // すべてのセンサーが完全にキャリブレーションされているか確認
    if (system == 3 && gyro == 3 && accel == 3 && mag == 3) {
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
    while(!changeToCONFIG()) {
        Serial.println("CONFIGに変更できません");
        delay(100);
    }
    _writeRegister(0x07, 0x01);
    byte data[BNO055_CALIBRATION_DATA_SIZE];
    _readRegister(0x55, data, BNO055_CALIBRATION_DATA_SIZE);
    
    _writeRegister(0x07, 0x00);
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
    
    while(!changeToNDOF()) {
        delay(100);
    }
    return true;
}

// センサーにキャリブレーションデータを書き込むプライベート関数
void BNO055::_setSensorOffsets() {
    while (!changeToCONFIG()) {
        delay(100);
    }
    
    if (!_writeRegister(0x07, 0x01)) {
        Serial.println("Page 1への切り替えに失敗しました。");
        return;
    }
    delay(10);
    
    // I2C通信を一度のトランザクションで完了させるように修正
    // これにより、通信の安定性が向上します
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(0x55); // 開始レジスタアドレス
    
    // 加速度計のオフセット
    Wire.write((uint8_t)(calibData.accel_offset_x & 0xFF));
    Wire.write((uint8_t)(calibData.accel_offset_x >> 8));
    Wire.write((uint8_t)(calibData.accel_offset_y & 0xFF));
    Wire.write((uint8_t)(calibData.accel_offset_y >> 8));
    Wire.write((uint8_t)(calibData.accel_offset_z & 0xFF));
    Wire.write((uint8_t)(calibData.accel_offset_z >> 8));

    // 磁力計のオフセット
    Wire.write((uint8_t)(calibData.mag_offset_x & 0xFF));
    Wire.write((uint8_t)(calibData.mag_offset_x >> 8));
    Wire.write((uint8_t)(calibData.mag_offset_y & 0xFF));
    Wire.write((uint8_t)(calibData.mag_offset_y >> 8));
    Wire.write((uint8_t)(calibData.mag_offset_z & 0xFF));
    Wire.write((uint8_t)(calibData.mag_offset_z >> 8));

    // ジャイロのオフセット
    Wire.write((uint8_t)(calibData.gyro_offset_x & 0xFF));
    Wire.write((uint8_t)(calibData.gyro_offset_x >> 8));
    Wire.write((uint8_t)(calibData.gyro_offset_y & 0xFF));
    Wire.write((uint8_t)(calibData.gyro_offset_y >> 8));
    Wire.write((uint8_t)(calibData.gyro_offset_z & 0xFF));
    Wire.write((uint8_t)(calibData.gyro_offset_z >> 8));
    
    // 半径
    Wire.write((uint8_t)(calibData.accel_radius & 0xFF));
    Wire.write((uint8_t)(calibData.accel_radius >> 8));
    Wire.write((uint8_t)(calibData.mag_radius & 0xFF));
    Wire.write((uint8_t)(calibData.mag_radius >> 8));
    
    byte status = Wire.endTransmission();
    if (status != 0) {
        Serial.printf("I2C multi-byte write failed. Status: %d\n", status);
    }

    _writeRegister(0x07, 0x00);
    delay(10);
    Serial.println("calib data have set");
}

// i2cをリセットする(BNOのエラー原因判明したから使う機会はなさそう)
void BNO055::i2cReset() {
    Serial.println("I2C bus reset initiated.");

    // I2Cのハードウェアを一時的に停止
    Wire.end();

    // SCLとSDAをGPIOとして設定
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);

    // SDAがLOWに固着しているか確認
    if (digitalRead(SDA) == LOW) {
        // SCLをHIGH/LOWさせて、SDAを解放させる
        pinMode(SCL, OUTPUT);
        for (int i = 0; i < 9; i++) {
        digitalWrite(SCL, HIGH);
        delayMicroseconds(10);
        digitalWrite(SCL, LOW);
        delayMicroseconds(10);
        }
    }

    // ストップコンディションを生成
    pinMode(SDA, OUTPUT);
    pinMode(SCL, OUTPUT);
    digitalWrite(SDA, LOW); // SDAをLOWに
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH); // SCLをHIGHに
    delayMicroseconds(10);
    digitalWrite(SDA, HIGH); // SDAをHIGHに
    delayMicroseconds(10);
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);

    // 再びI2C通信を開始
    Wire.begin(SDA, SCL);
    Serial.println("I2C bus reset complete.");
}