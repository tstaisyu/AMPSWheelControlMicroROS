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

const char* ssid       = WIFI_SSID;
const char* password   = WIFI_PASSWORD;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600 * 9;  // JSTのGMTオフセット
const int   daylightOffset_sec = 0;

void setupM5stack() {
  M5.begin();
  delay(500);
  #ifdef LEFT_WHEEL  
  imuManager.initialize();
  #endif
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);  // LCD表示初期位置

  Serial.begin(BAUD_RATE);
  while (!Serial);  // シリアルポートが開くのを待つ

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected.");

  // 時刻設定
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

}

void checkDataTimeout() {
  if (!initial_data_received && (millis() - last_receive_time > RECEIVE_TIMEOUT)) {
    Serial.printf("No data received for %d seconds, restarting...\n", RECEIVE_TIMEOUT / 1000);
    ESP.restart();
  }
}