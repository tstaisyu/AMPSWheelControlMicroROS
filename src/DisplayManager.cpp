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

// Updates the M5Stack display with twist message data
void updateDisplay(const geometry_msgs__msg__Twist* msg) {
    // Clear the display to prepare for new data
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 20);  // Set cursor for title
    M5.Lcd.print("Callback triggered");

    // Display linear x component of the twist message
    M5.Lcd.setCursor(0, 40);  // Set cursor for linear x data
    M5.Lcd.print("Linear.x: ");
    M5.Lcd.println(msg->linear.x);

    // Display angular z component of the twist message
    M5.Lcd.setCursor(0, 60);  // Set cursor for linear z data
    M5.Lcd.print("Angular.z: ");
    M5.Lcd.println(msg->angular.z);
}