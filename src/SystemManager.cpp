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

#include <M5Stack.h>
#include <WiFi.h>
#include "time.h"
#include "SystemManager.h"
#include "MotorController.h"
#include "IMUManager.h"
#include "config.h"

IMUManager imuManager;

// Configuration constants
const char* ssid       = WIFI_SSID;
const char* password   = WIFI_PASSWORD;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600 * 9;  // GMT+9 (JST) 
const int   daylightOffset_sec = 0;

// Initializes M5Stack device, WiFi and time settings
void setupM5stack() {
    M5.begin();
    delay(500);

#ifdef LEFT_WHEEL  
    // Initialize IMU if compiling for the left wheel configuration
    imuManager.initialize();
#endif

    // Set text size and initial cursor position for LCD display
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);  // Set cursor for title

    // Start serial communication
    Serial.begin(BAUD_RATE);
    while (!Serial);  // Wait for the serial port to connect. Needed for native USB

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("WiFi connected.");

    // Setup and sync time with NTP server
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Checks if data has not been received for a specified timeout and restarts if necessary
void checkDataTimeout() {
  if (!initial_data_received && (millis() - last_receive_time > RECEIVE_TIMEOUT)) {
    Serial.printf("No data received for %d seconds, restarting...\n", RECEIVE_TIMEOUT / 1000);
    ESP.restart();
  }
}