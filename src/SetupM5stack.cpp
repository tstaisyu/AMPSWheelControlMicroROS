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
#include "SetupM5stack.h"
#include "MotorController.h"

void setupM5stack() {
  M5.begin();
  delay(500);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);  // LCD表示初期位置
}

void initializeUART() {
    Serial.begin(BAUD_RATE);
    while (!Serial);  // シリアルポートが開くのを待つ
    motorSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("Setup complete. Ready to read high resolution speed data.");
    initMotor(motorSerial, MOTOR_ID);
    M5.Lcd.print("micro ROS2 M5Stack START\n");    
}
