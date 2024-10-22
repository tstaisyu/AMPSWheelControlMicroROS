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

#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <M5Stack.h>
#include "MadgwickAHRS.h"

class IMUManager {
public:
    IMUManager();
    void initialize();
    bool update();

    // Ahrsを使用しない場合
    void getCalibratedData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
    void getQuaternion(float& qw, float& qx, float& qy, float& qz);

/*
    // Ahrsを使用する場合
    void getCalibratedData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz);
*/

private:
    Madgwick filter;

    float ax, ay, az; // 加速度センサのデータ
    float gx, gy, gz; // ジャイロセンサのデータ
    float mx, my, mz;  // 地磁気データ
    float roll, pitch, yaw; // オイラー角
    float accOffset[3], gyroOffset[3], ahrsOffset[3]; // キャリブレーションオフセット

    float lpf_beta; // フィルタの係数
    float accX_filtered, accY_filtered, accZ_filtered, gyroX_filtered, gyroY_filtered, gyroZ_filtered, ahrsX_filtered, ahrsY_filtered, ahrsZ_filtered; // フィルタリングされたデータ

    void calibrateSensors(); // センサのキャリブレーションを行う
    void applyLowPassFilter(); // ローパスフィルタを適用する
    void updateMagneticField();
};

#endif // IMU_MANAGER_H
