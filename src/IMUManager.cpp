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

const float sampleFreq = 256.0f;  // Sampling rate in Hz

IMUManager::IMUManager() : lpf_beta(0.1), accX_filtered(0.0), gyroX_filtered(0.0) {
    // Initial setup for IMUManager with default low-pass filter coefficients
}

void IMUManager::initialize() {
    M5.IMU.Init();  // Initialize the IMU hardware
    calibrateSensors();  // Calibrate sensors to remove initial bias
}

bool IMUManager::update() {
    // Fetch the latest data from the IMU
    M5.IMU.getAccelData(&ax, &ay, &az);
    M5.IMU.getGyroData(&gx, &gy, &gz);

    // Apply the calibration offsets to raw data
    ax -= accOffset[0];
    ay -= accOffset[1];
    az -= accOffset[2]; // Consider gravity acceleration

    gx -= gyroOffset[0];
    gy -= gyroOffset[1];
    gz -= gyroOffset[2];

    applyLowPassFilter();  // Apply a low-pass filter to smooth the sensor data
  
    return true;  // Always returns true - consider adding error handling
}

void IMUManager::applyLowPassFilter() {
    // Low-pass filter the accelerometer and gyroscope data to reduce noise
    accX_filtered = accX_filtered + lpf_beta * (ax - accX_filtered);
    accY_filtered = accY_filtered + lpf_beta * (ay - accY_filtered);
    accZ_filtered = accZ_filtered + lpf_beta * (az - accZ_filtered);    
    gyroX_filtered = gyroX_filtered + lpf_beta * (gx - gyroX_filtered);
    gyroY_filtered = gyroY_filtered + lpf_beta * (gy - gyroY_filtered);
    gyroZ_filtered = gyroZ_filtered + lpf_beta * (gz - gyroZ_filtered);
}

void IMUManager::getCalibratedData(float &aX, float &aY, float &aZ, float &gX, float &gY, float &gZ) {
    // Return the filtered and calibrated sensor data
    aX = accX_filtered;
    aY = accY_filtered;
    aZ = accZ_filtered;
    gX = gyroX_filtered;
    gY = gyroY_filtered;
    gZ = gyroZ_filtered;
}

void IMUManager::calibrateSensors() {
    // Average several readings for calibration
    float sumAx = 0, sumAy = 0, sumAz = 0;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    const int samples = 500;  // Number of samples for averaging
    for (int i = 0; i < samples; i++) {
        M5.IMU.getAccelData(&ax, &ay, &az);
        M5.IMU.getGyroData(&gx, &gy, &gz);
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        delay(2);  // Small delay between samples
    }

    // Calculate and store the offsets from average values
    accOffset[0] = sumAx / samples;
    accOffset[1] = sumAy / samples;
    accOffset[2] = (sumAz / samples) - 1.0; // Assuming IMU unit is level
    gyroOffset[0] = sumGx / samples;
    gyroOffset[1] = sumGy / samples;
    gyroOffset[2] = sumGz / samples;
}
