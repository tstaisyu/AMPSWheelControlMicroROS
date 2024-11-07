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

// Manages interactions with the IMU sensors on the M5Stack, including initialization,
// data updates, and sensor calibration. It provides both raw and calibrated data access
// methods, and applies filtering to the sensor data to improve accuracy.
class IMUManager {
public:
    IMUManager();  // Constructor
    void initialize();  // Initializes the IMU sensors
    bool update();  // Updates sensor data, returns true if new data is available

    // Retrieves calibrated acceleration and gyroscope data.
    // This function assumes that calibration offsets are already applied.
    void getCalibratedData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

private:
    float ax, ay, az; // Accelerometer data
    float gx, gy, gz; // Gyroscope data
    float accOffset[3], gyroOffset[3]; // Calibration offsets for accelerometer and gyroscope

    float lpf_beta; // Coefficient for the low-pass filter
    float accX_filtered, accY_filtered, accZ_filtered; // Low-pass filtered accelerometer data
    float gyroX_filtered, gyroY_filtered, gyroZ_filtered; // Low-pass filtered gyroscope data

    void calibrateSensors(); // Calibrates the sensors to adjust for drift and bias
    void applyLowPassFilter(); // Applies a low-pass filter to smooth out sensor data
};

#endif // IMU_MANAGER_H
