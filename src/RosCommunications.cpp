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

// Communication Check: Publisher and Subscriber for integrity check messages
rcl_subscription_t com_check_subscriber;  // Subscriber for communication check requests
rcl_publisher_t com_check_publisher;      // Publisher for communication check responses
std_msgs__msg__Int32 com_req_msg;          // Message for incoming communication requests
std_msgs__msg__Int32 com_res_msg;          // Message for outgoing communication responses

// Reboot service: Handles requests to reboot the system safely
rcl_service_t reboot_service;              // Service to manage reboot requests
std_srvs__srv__Trigger_Request req;        // Reboot request message
std_srvs__srv__Trigger_Response res;       // Reboot response message

// cmd_vel subscriber: Subscribes to velocity commands for the robot
rcl_subscription_t cmd_vel_subscriber;     // Subscriber for velocity commands
geometry_msgs__msg__Twist msg_sub;         // Message type for subscribing to velocity commands

// Velocity publisher: Publishes velocity commands as stamped messages
rcl_publisher_t vel_publisher;             // Publisher for velocity data
geometry_msgs__msg__TwistStamped vel_msg;  // Stamped message for velocity data

// IMU publisher: Publishes IMU data to other components in the system
rcl_publisher_t imu_publisher;             // Publisher for IMU data
sensor_msgs__msg__Imu imu_msg;             // IMU message type

// Timer callback: Manages timing for regular updates in the system
rcl_timer_t timer;                         // Timer for periodic updates
rcl_time_point_value_t current_time;       // Stores the current time point
rcl_clock_t ros_clock;                     // Clock to manage system time

// microROS node and executor: Core components for managing ROS 2 nodes and callbacks
rclc_executor_t executor;                  // Executor for managing callbacks
rclc_support_t support;                    // Support structure for the node
rcl_allocator_t allocator;                 // Allocator for the node's resources
rcl_node_t node;                           // The node itself


void setupMicroROS() {
    // Initialize micro-ROS transports
    set_microros_transports();

    // Initialize the default allocator for memory management
    allocator = rcl_get_default_allocator();

    // Initialize ROS clock with ROS time and the default allocator
    rcl_ret_t rc = rcl_clock_init(RCL_ROS_TIME, &ros_clock, &allocator);
    if (rc != RCL_RET_OK) {
        Serial.println("Failed to initialize ROS clock");
        return;
    }

    // Initialize micro-ROS support structure
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    //init_options = rcl_get_zero_initialized_init_options();
    //RCCHECK(rcl_init_options_init(&init_options, allocator));
    //RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));		// ドメインIDの設定
    //RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)); // 前のrclc_support_initは削除する
    
    // Initialize the ROS node based on the wheel type (left or right)
    #ifdef LEFT_WHEEL
    RCCHECK(rclc_node_init_default(&node, "left_wheel_node", "", &support));
    #elif defined(RIGHT_WHEEL)
    RCCHECK(rclc_node_init_default(&node, "right_wheel_node", "", &support));
    #endif

    // Call initialization functions of the ROS executor and the components for the node
    initializePublishers(&node);
    initializeSubscribers(&node);
    initializeServices(&node);
    #ifdef LEFT_WHEEL
        initializeIMU(&node);
    #endif
    initializeTimer(&timer, &support);
    initializeExecutor(&executor, &support, &allocator);
}

// Initialize Publishers
void initializePublishers(rcl_node_t *node) {
    // Initialize Communication Check Publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &com_check_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "connection_response"
    ));

    // Initialize Velocity Publisher based on wheel type
    #ifdef LEFT_WHEEL
        RCCHECK(rclc_publisher_init_best_effort(
            &vel_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
            "/left_vel"
        ));
    #elif defined(RIGHT_WHEEL)
        RCCHECK(rclc_publisher_init_best_effort(
            &vel_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
            "/right_vel"
        ));
    #endif

    // Initialize Velocity Message with default values
    vel_msg.twist.linear.x = 0.0;
    vel_msg.twist.linear.y = 0.0;
    vel_msg.twist.linear.z = 0.0;
    vel_msg.twist.angular.x = 0.0;
    vel_msg.twist.angular.y = 0.0;
    vel_msg.twist.angular.z = 0.0;

    // Allocate buffer for Velocity Frame ID and set it
    static char vel_frame_id_buffer[256]; // Ensure sufficient size
    vel_msg.header.frame_id.data = vel_frame_id_buffer; // Point to buffer

    #ifdef LEFT_WHEEL
        const char* vel_frame_id = "l_w";
    #elif defined(RIGHT_WHEEL)
        const char* vel_frame_id = "r_w";
    #endif
    strncpy(vel_msg.header.frame_id.data, vel_frame_id, sizeof(vel_msg.header.frame_id.data));
    vel_msg.header.frame_id.size = strlen(vel_frame_id);
}

// Initialize Subscribers
void initializeSubscribers(rcl_node_t *node) {
    // Initialize Communication Check Subscriber
    RCCHECK(rclc_subscription_init_best_effort(
        &com_check_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "connection_check"
    ));

    // Initialize cmd_vel Subscriber for velocity commands 
    RCCHECK(rclc_subscription_init_best_effort(
        &cmd_vel_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    ));
}

// Initialize Reboot Service Server
void initializeServices(rcl_node_t *node) {
    // Reboot Service Server の初期化
    RCCHECK(rclc_service_init_best_effort(
        &reboot_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "/reboot_service"
    ));
}

#ifdef LEFT_WHEEL
// Initialize IMU Publisher and IMU Message for Left Wheel
void initializeIMU(rcl_node_t *node) {
    // Initialize IMU data publisher for left wheel
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data_raw"
    ));

    // Set default covariance values for IMU data
    imu_msg.orientation_covariance[0] = -1; // Uncomment if using orientation

    // Uncomment and set if using orientation covariance
    /*  
    imu_msg.orientation_covariance[0] = 0.0001; // Variance for x-axis
    imu_msg.orientation_covariance[1] = 0.0;
    imu_msg.orientation_covariance[2] = 0.0;
    imu_msg.orientation_covariance[3] = 0.0;
    imu_msg.orientation_covariance[4] = 0.0001; // Variance for y-axis
    imu_msg.orientation_covariance[5] = 0.0;
    imu_msg.orientation_covariance[6] = 0.0;
    imu_msg.orientation_covariance[7] = 0.0;
    imu_msg.orientation_covariance[8] = 0.0001; // Variance for z-axis
    */

    // Set covariance for angular velocity
    imu_msg.angular_velocity_covariance[0] = 0.0025;  // Variance for x-axis
    imu_msg.angular_velocity_covariance[1] = 0.0;
    imu_msg.angular_velocity_covariance[2] = 0.0;
    imu_msg.angular_velocity_covariance[3] = 0.0;
    imu_msg.angular_velocity_covariance[4] = 0.0025;  // Variance for y-axis
    imu_msg.angular_velocity_covariance[5] = 0.0;
    imu_msg.angular_velocity_covariance[6] = 0.0;
    imu_msg.angular_velocity_covariance[7] = 0.0;
    imu_msg.angular_velocity_covariance[8] = 0.0025;  // Variance for z-axis

    // Set covariance for linear acceleration
    imu_msg.linear_acceleration_covariance[0] = 0.0004;  // Variance for x-axis
    imu_msg.linear_acceleration_covariance[1] = 0.0;
    imu_msg.linear_acceleration_covariance[2] = 0.0;
    imu_msg.linear_acceleration_covariance[3] = 0.0;
    imu_msg.linear_acceleration_covariance[4] = 0.0004;  // Variance for y-axis
    imu_msg.linear_acceleration_covariance[5] = 0.0;
    imu_msg.linear_acceleration_covariance[6] = 0.0;
    imu_msg.linear_acceleration_covariance[7] = 0.0;
    imu_msg.linear_acceleration_covariance[8] = 0.0004;  // Variance for z-axis

    // Allocate buffer for IMU Frame ID and set it
    static char imu_frame_id_buffer[256];
    imu_msg.header.frame_id.data = imu_frame_id_buffer; // Point to buffer

    const char* imu_frame_id = "imu";
    strncpy(imu_msg.header.frame_id.data, imu_frame_id, sizeof(imu_msg.header.frame_id.data));
    imu_msg.header.frame_id.size = strlen(imu_frame_id);
}
#endif

// Initialize Timer for periodic callbacks
void initializeTimer(rcl_timer_t *timer, rclc_support_t *support) {
    const unsigned int timer_timeout = 20;  // 20ms interval
    RCCHECK(rclc_timer_init_default(
        timer,
        support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback
    ));
}

// Initialize the Executor with the number of callbacks
void initializeExecutor(rclc_executor_t *executor, rclc_support_t *support, rcl_allocator_t *allocator) {
    int callback_size = 4;	// Number of callbacks to handle
    *executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(executor, &support->context, callback_size, allocator));

    // Add Communication Check Subscriber to Executor
    RCCHECK(rclc_executor_add_subscription(
        executor,
        &com_check_subscriber,
        &com_req_msg,
        &com_check_callback,
        ON_NEW_DATA
    ));

    // Add Reboot Service to Executor
    RCCHECK(rclc_executor_add_service(
        executor,
        &reboot_service,
        &req,
        &res,
        &reboot_callback
    ));

    // Add cmd_vel Subscriber to Executor
    RCCHECK(rclc_executor_add_subscription(
        executor,
        &cmd_vel_subscriber,
        &msg_sub,
        &subscription_callback,
        ON_NEW_DATA
    ));

    // Add Timer to Executor
    RCCHECK(rclc_executor_add_timer(
        executor,
        &timer
    ));
}

void reboot_callback(const void * request, void * response) {

  std_srvs__srv__Trigger_Request *_req = (std_srvs__srv__Trigger_Request *)request;
	std_srvs__srv__Trigger_Response *_res = (std_srvs__srv__Trigger_Response *)response;
    
  Serial.println("Reboot command received.");
  
  // レスポンスの設定
  _res->success = true;
  if (!ROSIDL_RUNTIME_C__STRING_H_(&_res->message, "Rebooting in 5 seconds...")) {
      Serial.println("Failed to assign reboot message.");
  }
  
  // 再起動のタイミングを遅延
  delay(5000);
  ESP.restart();

}

void com_check_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * com_req_msg = (const std_msgs__msg__Int32 *)msgin;
    Serial.print("Received connection check: ");
    Serial.println(com_req_msg->data);

    last_receive_time = millis();
    if (!initial_data_received) {
      initial_data_received = true;
    }

    // 受信したら応答メッセージを送る
    com_res_msg.data = 1;
   
    rcl_ret_t ret = rcl_publish(&com_check_publisher, &com_res_msg, NULL);
    if (ret != RCL_RET_OK) {
        Serial.print("Failed to publish message: ");
        Serial.println(rcl_get_error_string().str);
        rcl_reset_error();
    }
    Serial.println("Published connection response: connection_established");
}

void subscription_callback(const void * msgin) {

  const geometry_msgs__msg__Twist * msg_sub = (const geometry_msgs__msg__Twist *)msgin;
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

        // Ahrsを使用しない場合
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
        imu_msg.orientation.x = 0.1;
        imu_msg.orientation.y = 0.1;
        imu_msg.orientation.z = 0.1;
        imu_msg.orientation.w = 0.1;

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