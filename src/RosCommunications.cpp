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

#include "RosCommunications.h"
#include "MotorController.h"
#include "DisplayManager.h"
#include "SerialManager.h"
#include "SystemManager.h"
#include "IMUManager.h"

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
geometry_msgs__msg__TwistStamped vel_msg;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t vel_publisher;
rcl_publisher_t imu_publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_time_point_value_t current_time;
rcl_clock_t ros_clock;

void setupMicroROS() {
	set_microros_transports();
  allocator = rcl_get_default_allocator();
  rcl_ret_t rc = rcl_clock_init(RCL_ROS_TIME, &ros_clock, &allocator);
  if (rc != RCL_RET_OK) {
      Serial.println("Failed to initialize ROS clock");
      return;
  }
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
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
      "/left_vel"
  ));
  #elif defined(RIGHT_WHEEL)
  // 右輪用の処理
  RCCHECK(rclc_publisher_init_best_effort(
      &vel_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
      "/right_vel"
  ));
  #endif


  vel_msg.twist.linear.x = 0.0;
  vel_msg.twist.linear.y = 0.0;
  vel_msg.twist.linear.z = 0.0;
  vel_msg.twist.angular.x = 0.0;
  vel_msg.twist.angular.y = 0.0;
  vel_msg.twist.angular.z = 0.0;

  static char frame_id_buffer[256]; // 十分なサイズを確保
  vel_msg.header.frame_id.data = frame_id_buffer; // ポインタをバッファに設定

  #ifdef LEFT_WHEEL
  const char* frame_id = "l_w";
  #elif defined(RIGHT_WHEEL)
  const char* frame_id = "r_w";
  #endif
  strncpy(vel_msg.header.frame_id.data, frame_id, sizeof(vel_msg.header.frame_id.data));
  vel_msg.header.frame_id.size = strlen(frame_id);


  #ifdef LEFT_WHEEL
  // Initialize IMU data publisher for left wheel
  RCCHECK(rclc_publisher_init_best_effort(
      &imu_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/left_wheel_imu"
  ));

  imu_msg.orientation_covariance[0] = -1;

  // 角速度の共分散行列設定
  imu_msg.angular_velocity_covariance[0] = 0.0025;  // x軸の分散
  imu_msg.angular_velocity_covariance[1] = 0.0;
  imu_msg.angular_velocity_covariance[2] = 0.0;
  imu_msg.angular_velocity_covariance[3] = 0.0;
  imu_msg.angular_velocity_covariance[4] = 0.0025;  // y軸の分散
  imu_msg.angular_velocity_covariance[5] = 0.0;
  imu_msg.angular_velocity_covariance[6] = 0.0;
  imu_msg.angular_velocity_covariance[7] = 0.0;
  imu_msg.angular_velocity_covariance[8] = 0.0025;  // z軸の分散

  // 線形加速度の共分散行列設定
  imu_msg.linear_acceleration_covariance[0] = 0.0004;  // x軸の分散
  imu_msg.linear_acceleration_covariance[1] = 0.0;
  imu_msg.linear_acceleration_covariance[2] = 0.0;
  imu_msg.linear_acceleration_covariance[3] = 0.0;
  imu_msg.linear_acceleration_covariance[4] = 0.0004;  // y軸の分散
  imu_msg.linear_acceleration_covariance[5] = 0.0;
  imu_msg.linear_acceleration_covariance[6] = 0.0;
  imu_msg.linear_acceleration_covariance[7] = 0.0;
  imu_msg.linear_acceleration_covariance[8] = 0.0004;  // z軸の分散

  #endif

  // タイマーの初期化
  const unsigned int timer_timeout = 20;  // 20ms
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback
  ));

	int callback_size = 2;	// コールバックを行う数
	executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, callback_size, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
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
//  M5.Lcd.clear();
//  M5.Lcd.setCursor(0, 0);
//  M5.Lcd.print("Timer callback triggered");

  rcl_ret_t rc = rcl_clock_get_now(&ros_clock, &current_time);
  if (rc != RCL_RET_OK) {
      // エラー処理
      Serial.println("Failed to get current time");
  } else {
      // Read IMU data
    #ifdef LEFT_WHEEL
    if (imuManager.update()) {
        float ax, ay, az, gx, gy, gz;
        imuManager.getCalibratedData(ax, ay, az, gx, gy, gz);

        // IMUメッセージのタイムスタンプを設定
        imu_msg.header.stamp.sec = current_time / 1000000000;  // 秒
        imu_msg.header.stamp.nanosec = current_time % 1000000000;  // ナノ秒
        imu_msg.linear_acceleration.x = ax * GRAVITY;
        imu_msg.linear_acceleration.y = ay * GRAVITY;
        imu_msg.linear_acceleration.z = az * GRAVITY;
        imu_msg.angular_velocity.x = gx * DEG2RAD;
        imu_msg.angular_velocity.y = gy * DEG2RAD;
        imu_msg.angular_velocity.z = gz * DEG2RAD;

        // IMUデータの表示
//        M5.Lcd.setCursor(0, 20);
//        M5.Lcd.printf("Accel: %.2f, %.2f, %.2f", ax, ay, az);
//        M5.Lcd.setCursor(0, 40);
//        M5.Lcd.printf("Gyro: %.2f, %.2f, %.2f", gx, gy, gz);            
    }
    #endif

      // Read wheel speed data
      float wheelSpeed = readSpeedData(motorSerial, MOTOR_ID);
      vel_msg.header.stamp.sec = current_time / 1000000000;  // 秒
      vel_msg.header.stamp.nanosec = current_time % 1000000000;  // ナノ秒
      #ifdef LEFT_WHEEL
      vel_msg.twist.linear.x = -wheelSpeed;
      #elif defined(RIGHT_WHEEL)
      vel_msg.twist.linear.x = wheelSpeed;
      #endif
/*
      M5.Lcd.clear();
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.print("Timer Callback Triggered");
      M5.Lcd.setCursor(0, 20);
      M5.Lcd.printf("Linear X: %f", vel_msg.twist.linear.x);
      M5.Lcd.setCursor(0, 40);
      M5.Lcd.printf("Linear Y: %f", vel_msg.twist.linear.y);
      M5.Lcd.setCursor(0, 60);
      M5.Lcd.printf("Linear Z: %f", vel_msg.twist.linear.z);
      M5.Lcd.setCursor(0, 80);
      M5.Lcd.printf("Angular X: %f", vel_msg.twist.angular.x);
      M5.Lcd.setCursor(0, 100);
      M5.Lcd.printf("Angular Y: %f", vel_msg.twist.angular.y);
      M5.Lcd.setCursor(0, 120);
      M5.Lcd.printf("Angular Z: %f", vel_msg.twist.angular.z);
*/
  }

  // Publish IMU and velocity data
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&vel_publisher, &vel_msg, NULL));
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