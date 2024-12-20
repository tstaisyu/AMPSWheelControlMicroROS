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

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <geometry_msgs/msg/twist.h>

// Updates the device's display with the current velocity data
// This function takes a Twist message, which includes linear and angular velocity components,
// and displays these values on the device's screen.
void updateDisplay(const geometry_msgs__msg__Twist* msg);

#endif // DISPLAY_MANAGER_H
