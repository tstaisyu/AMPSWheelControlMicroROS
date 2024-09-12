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
#include "RosCommunications.h"
#include "MotorController.h"
#include "DisplayManager.h"
#include "SerialManager.h"

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
geometry_msgs__msg__Twist msg_pub;
rcl_publisher_t vel_publisher;
nav_msgs__msg__Odometry odom_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void setupMicroROS() {
	set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	//init_options = rcl_get_zero_initialized_init_options();
	//RCCHECK(rcl_init_options_init(&init_options, allocator));
	//RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));		// ドメインIDの設定
	//RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)); // 前のrclc_support_initは削除する
  #ifdef LEFT_WHEEL
  RCCHECK(rclc_node_init_default(&node, "left_wheel_node", "", &support));
  #elif defined(RIGHT_WHEEL)
  RCCHECK(rclc_node_init_default(&node, "right_wheel_node", "", &support));
  #endif

  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  ));

  #ifdef LEFT_WHEEL
  // 左輪用の処理
  RCCHECK(rclc_publisher_init_best_effort(
      &vel_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/left_vel"
  ));
  #elif defined(RIGHT_WHEEL)
  // 右輪用の処理
  RCCHECK(rclc_publisher_init_best_effort(
      &vel_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/right_vel"
  ));
  #endif

  // タイマーの初期化
  rcl_timer_t publish_timer;
  const unsigned int timer_timeout = 20;  // 20ms
  RCCHECK(rclc_timer_init_default2(
      &publish_timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback,
      NULL
  ));

	int callback_size = 1;	// コールバックを行う数
//	executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
}

void subscription_callback(const void * msgin) {

  const geometry_msgs__msg__Twist * msg_sub = (const geometry_msgs__msg__Twist *)msgin;
  last_receive_time = millis();
  if (!initial_data_received) {
    initial_data_received = true;
  }
  updateDisplay(msg_sub);
  logReceivedData(msg_sub);
  sendMotorCommands(msg_sub->linear.x, msg_sub->angular.z);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.print("Timer callback");
  float wheelSpeed = readSpeedData(motorSerial, MOTOR_ID);

  geometry_msgs__msg__Twist msg;
  msg.linear.x = wheelSpeed;
  msg.angular.z = 0.0;
    
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&vel_publisher, &msg, NULL));
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