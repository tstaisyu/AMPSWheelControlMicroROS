/* Copyright 2024 Taisyu Shibata
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "IMUManager.h"
#include <BMM150Compass.h>

BMM150Compass compass;

IMUManager::IMUManager() : lpf_beta(0.1), accX_filtered(0.0), gyroX_filtered(0.0), ahrsX_filtered(0.0) {
    // 初期化処理
}

void IMUManager::initialize() {
    M5.IMU.Init();
/*    
    compass = BMM150Compass();

    // センサーの初期化
    while (compass.initialize() != BMM150_OK)
    {
        M5.Lcd.setCursor(0, 0);    
        M5.Lcd.printf("BMM150 initialization failed! Retrying...");
        delay(1000); // 1秒待って再試行
    }
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.printf("BMM150 initialized successfully.");

    // オフセットの読み込み
    compass.offset_load();

    // キャリブレーションの実行（10秒間）
    M5.Lcd.setCursor(0, 30);    
    M5.Lcd.printf("Starting calibration...");
    compass.calibrate(10000);
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf("Calibration complete.");
*/
    calibrateSensors();
}

bool IMUManager::update() {
    
    // IMUからデータを読み取る
    M5.IMU.getAccelData(&ax, &ay, &az);
    M5.IMU.getGyroData(&gx, &gy, &gz);
    M5.IMU.getAhrsData(&mx, &my, &mz);

    // オフセット適用
    ax -= accOffset[0];
    ay -= accOffset[1];
    az -= accOffset[2]; // 重力加速度を考慮

    gx -= gyroOffset[0];
    gy -= gyroOffset[1];
    gz -= gyroOffset[2];

    mx -= ahrsOffset[0];
    my -= ahrsOffset[1];
    mz -= ahrsOffset[2];

    applyLowPassFilter();
/*
    // 地磁気データを更新
    updateMagneticField();
*/    
    return true;
}

void IMUManager::updateMagneticField() {
    int16_t mag_data[3];
    compass.getXYZ(mag_data);  // 地磁気データを取得

    // 地磁気データをフィルタリング
    mx = mag_data[0];
    my = mag_data[1];
    mz = mag_data[2];

    // LCDに表示
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.printf("X: %d", mag_data[0]);
    M5.Lcd.setCursor(0, 80);
    M5.Lcd.printf("Y: %d", mag_data[1]);
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.printf("Z: %d", mag_data[2]);

}

void IMUManager::applyLowPassFilter() {
    // 加速度とジャイロスコープのデータをフィルタリング
    accX_filtered = accX_filtered + lpf_beta * (ax - accX_filtered);
    accY_filtered = accY_filtered + lpf_beta * (ay - accY_filtered);
    accZ_filtered = accZ_filtered + lpf_beta * (az - accZ_filtered);    
    gyroX_filtered = gyroX_filtered + lpf_beta * (gx - gyroX_filtered);
    gyroY_filtered = gyroY_filtered + lpf_beta * (gy - gyroY_filtered);
    gyroZ_filtered = gyroZ_filtered + lpf_beta * (gz - gyroZ_filtered);
    ahrsX_filtered = ahrsX_filtered + lpf_beta * (mx - ahrsX_filtered);
    ahrsY_filtered = ahrsY_filtered + lpf_beta * (my - ahrsY_filtered);
    ahrsZ_filtered = ahrsZ_filtered + lpf_beta * (mz - ahrsZ_filtered);
}

void IMUManager::getCalibratedData(float &aX, float &aY, float &aZ, float &gX, float &gY, float &gZ, float &mX, float &mY, float &mZ) {
    aX = accX_filtered;
    aY = accY_filtered;
    aZ = accZ_filtered;
    gX = gyroX_filtered;
    gY = gyroY_filtered;
    gZ = gyroZ_filtered;
    mX = ahrsX_filtered;
    mY = ahrsY_filtered;
    mZ = ahrsZ_filtered; 
}

void IMUManager::calibrateSensors() {
    float sumAx = 0, sumAy = 0, sumAz = 0;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    float sumMx = 0, sumMy = 0, sumMz = 0;    
    const int samples = 500;
    for (int i = 0; i < samples; i++) {
        M5.IMU.getAccelData(&ax, &ay, &az);
        M5.IMU.getGyroData(&gx, &gy, &gz);
        M5.IMU.getAhrsData(&mx, &my, &mz);
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        sumMx += mx;
        sumMy += my;
        sumMz += mz;
        delay(2);
    }

    accOffset[0] = sumAx / samples;
    accOffset[1] = sumAy / samples;
    accOffset[2] = (sumAz / samples) - 1.0; // M5スティックが水平であると仮定
    gyroOffset[0] = sumGx / samples;
    gyroOffset[1] = sumGy / samples;
    gyroOffset[2] = sumGz / samples;
    ahrsOffset[0] = sumMx / samples;
    ahrsOffset[1] = sumMy / samples;
    ahrsOffset[2] = sumMz / samples;
}
