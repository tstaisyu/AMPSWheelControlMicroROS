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
#include "SystemManager.h"
#include "MotorController.h"
#include "RosCommunications.h"

// Initializes the system on startup
void setup() {
    // Initialize M5Stack hardware configurations
    setupM5stack();

    // Initialize UART communication for motors
    initializeUART();

    // Set up micro-ROS environment and node
    setupMicroROS();

    // Record the last time data was received to monitor timeouts
    last_receive_time = millis();
}

// Main loop to handle routine operations
void loop() {
    // Process ROS 2 executor callbacks
    handleExecutorSpin();

    // Check for data reception timeouts and handle if necessary
    checkDataTimeout();
}
