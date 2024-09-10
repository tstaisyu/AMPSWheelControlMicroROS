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
#include "RosCommunications.h"

void setup() {

  setupM5stack();
  initializeUART();
  setupMicroROS();
  last_receive_time = millis();
}

void loop() {
  handleDataPublishing();
  handleExecutorSpin();
  checkDataTimeout();
}

void handleDataPublishing() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastReadTime >= readInterval) {
    publishSpeedData();
    lastReadTime = currentMillis;
  }
}

void handleExecutorSpin() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  if (rcl_error_is_set()) {
    RCL_SET_ERROR_MSG("rclc_executor_spin_some failed");
    printf("Error in rclc_executor_spin_some: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
  }
}

void checkDataTimeout() {
  if (!initial_data_received && (millis() - last_receive_time > RECEIVE_TIMEOUT)) {
    Serial.printf("No data received for %d seconds, restarting...\n", RECEIVE_TIMEOUT / 1000);
    ESP.restart();
  }
}

