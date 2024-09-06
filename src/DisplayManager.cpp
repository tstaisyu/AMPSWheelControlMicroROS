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
#include "DisplayManager.h"

void updateDisplay(const geometry_msgs__msg__Twist* msg_sub) {
  M5.Lcd.clear();  // LCD画面をクリア
  M5.Lcd.setCursor(0, 20);  // テキスト表示位置を設定
  M5.Lcd.print("Callback triggered");

  M5.Lcd.setCursor(0, 40);
  M5.Lcd.print("Linear.x: ");
  M5.Lcd.println(msg->linear.x);

  M5.Lcd.setCursor(0, 60);
  M5.Lcd.print("Angular.z: ");
  M5.Lcd.println(msg->angular.z);
}