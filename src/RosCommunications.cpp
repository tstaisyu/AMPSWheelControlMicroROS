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

// Define wheel-specific suffix based on the wheel type
#ifdef LEFT_WHEEL
    #define WHEEL_SUFFIX "left_wheel"
#elif defined(RIGHT_WHEEL)
    #define WHEEL_SUFFIX "right_wheel"
#endif

// Define ROS2 node names based on the wheel type
#define NODE_NAME WHEEL_SUFFIX "_micro_ros_node"

// Constants for ROS 2 topic and service names
#define REBOOT_SERVICE_NAME "/" WHEEL_SUFFIX "/reboot_service"
#define CONNECTION_RESPONSE_TOPIC WHEEL_SUFFIX "/connection_response"
#define VELOCITY_TOPIC "/" WHEEL_SUFFIX "/velocity"

// Common topics not specific to any wheel
#define CONNECTION_CHECK_TOPIC "connection_check_request"
#define CMD_VEL_TOPIC "/cmd_vel"
#define IMU_DATA_TOPIC "/imu/data_raw"

// Constants for ROS 2 frame IDs
#define IMU_FRAME_ID "imu"
#define VELOCITY_FRAME_ID WHEEL_SUFFIX "_v"

// Communication Check: Publisher and Subscriber for integrity check messages
rcl_subscription_t com_check_subscriber;  // Subscriber for communication check requests
rcl_publisher_t com_check_publisher;      // Publisher for communication check responses
std_msgs__msg__Int32 com_req_msg;          // Message for incoming communication requests
std_msgs__msg__Int32 com_res_msg;          // Message for outgoing communication responses

// Reboot service: Handles requests to reboot the system safely
rcl_service_t reboot_service;              // Service to manage reboot requests
std_srvs__srv__Trigger_Request request;        // Reboot request message
std_srvs__srv__Trigger_Response response;       // Reboot response message

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

// Initialize microROS components and setup the ROS 2 node
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
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

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
        CONNECTION_RESPONSE_TOPIC
    ));

    // Initialize Velocity Publisher based on wheel type
    RCCHECK(rclc_publisher_init_best_effort(
        &vel_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        VELOCITY_TOPIC
    ));

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

    const char* vel_frame_id = VELOCITY_FRAME_ID;

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
        CONNECTION_CHECK_TOPIC
    ));

    // Initialize cmd_vel Subscriber for velocity commands 
    RCCHECK(rclc_subscription_init_best_effort(
        &cmd_vel_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC
    ));
}

// Initialize Reboot Service Server
void initializeServices(rcl_node_t *node) {
    // Reboot Service Server の初期化
    RCCHECK(rclc_service_init_best_effort(
        &reboot_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        REBOOT_SERVICE_NAME
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
        IMU_DATA_TOPIC
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
    imu_msg.angular_velocity_covariance[0] = 0.05;  // Variance for x-axis
    imu_msg.angular_velocity_covariance[1] = 0.0;
    imu_msg.angular_velocity_covariance[2] = 0.0;
    imu_msg.angular_velocity_covariance[3] = 0.0;
    imu_msg.angular_velocity_covariance[4] = 0.05;  // Variance for y-axis
    imu_msg.angular_velocity_covariance[5] = 0.0;
    imu_msg.angular_velocity_covariance[6] = 0.0;
    imu_msg.angular_velocity_covariance[7] = 0.0;
    imu_msg.angular_velocity_covariance[8] = 0.05;  // Variance for z-axis

    // Set covariance for linear acceleration
    imu_msg.linear_acceleration_covariance[0] = 0.2;  // Variance for x-axis
    imu_msg.linear_acceleration_covariance[1] = 0.0;
    imu_msg.linear_acceleration_covariance[2] = 0.0;
    imu_msg.linear_acceleration_covariance[3] = 0.0;
    imu_msg.linear_acceleration_covariance[4] = 0.2;  // Variance for y-axis
    imu_msg.linear_acceleration_covariance[5] = 0.0;
    imu_msg.linear_acceleration_covariance[6] = 0.0;
    imu_msg.linear_acceleration_covariance[7] = 0.0;
    imu_msg.linear_acceleration_covariance[8] = 0.2;  // Variance for z-axis

    // Allocate buffer for IMU Frame ID and set it
    static char imu_frame_id_buffer[256];
    imu_msg.header.frame_id.data = imu_frame_id_buffer; // Point to buffer

    const char* imu_frame_id = IMU_FRAME_ID;
    strncpy(imu_msg.header.frame_id.data, imu_frame_id, sizeof(imu_msg.header.frame_id.data));
    imu_msg.header.frame_id.size = strlen(imu_frame_id);
}
#endif

// Initialize Timer for periodic callbacks
void initializeTimer(rcl_timer_t *timer, rclc_support_t *support) {
    const unsigned int timer_timeout = TIMER_INTERVAL;
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
        &request,
        &response,
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

// Reboot device upon receiving a reboot command
void reboot_callback(const void * request, void * response) {
    // Cast request and response to appropriate service message types
    std_srvs__srv__Trigger_Request *req = (std_srvs__srv__Trigger_Request *)request;
    std_srvs__srv__Trigger_Response *res = (std_srvs__srv__Trigger_Response *)response;

    // Log receipt of the reboot command    
    Serial.println("Reboot command received.");
    
    // Attempt to set the response message
    res->success = true;
    if (!ROSIDL_RUNTIME_C__STRING_H_(&res->message, "Rebooting in 5 seconds...")) {
        Serial.println("Failed to assign reboot message.");
        res->success = false; // Ensure the response reflects the failure
    }
    
    // Delay before reboot to allow message transmission
    delay(5000);
    ESP.restart(); // Perform system restart
}

// Handles the reception of connection check messages and sends a response
void com_check_callback(const void * msgin) {
    // Cast the incoming message to the appropriate type
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

    // Log the received connection check value
    Serial.print("Received connection check: ");
    Serial.println(msg->data);

    // Update the last time a message was received
    last_receive_time = millis();

    // Mark the initial data as received if it's the first message
    if (!initial_data_received) {
      initial_data_received = true;
    }

    // Prepare the response message
    com_res_msg.data = 1; // Set the data to indicate connection is established

    // Attempt to publish the response message   
    rcl_ret_t ret = rcl_publish(&com_check_publisher, &com_res_msg, NULL);
    if (ret != RCL_RET_OK) {
        // Log any failures to publish the response
        Serial.print("Failed to publish message: ");
        Serial.println(rcl_get_error_string().str);
        rcl_reset_error();  // Clear the error to avoid propagation
    }

    // Confirm the publication of the response
    Serial.println("Published connection response: connection_established");
}

// Callback function for handling received Twist messages
void subscription_callback(const void *msgin) {
    // Cast the incoming message to the appropriate message type
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    // Update the display with the new data
    updateDisplay(msg);

    // Log the received data for debugging and monitoring purposes
    logReceivedData(msg);

    // Send commands to the motor based on the received Twist message
    sendMotorCommands(msg->linear.x, msg->angular.z);
}

// Timer callback function to handle periodic tasks
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);

    // Get current time from ROS clock
    rcl_ret_t rc = rcl_clock_get_now(&ros_clock, &current_time);
    if (rc != RCL_RET_OK) {
        Serial.println("Failed to get current time");
        return;
    }

    // Update IMU data if available
#ifdef LEFT_WHEEL
    if (imuManager.update()) {
        updateIMUData(); // Function to update and publish IMU data
    }
#endif

    // Read and publish wheel speed data
    updateWheelSpeed(); // Function to update and publish wheel speed

    // Ensure the timer is not null before publishing data
    if (timer != NULL) {
      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
      RCSOFTCHECK(rcl_publish(&vel_publisher, &vel_msg, NULL));
    }
}

// Function to update IMU data
void updateIMUData() {
    float ax, ay, az, gx, gy, gz;
    imuManager.getCalibratedData(ax, ay, az, gx, gy, gz);

    // Set IMU message timestamps
    imu_msg.header.stamp.sec = current_time / 1000000000;  // seconds
    imu_msg.header.stamp.nanosec = current_time % 1000000000;  // nanoseconds
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
}

// Function to update and publish wheel speed data
void updateWheelSpeed() {
    float wheelSpeed = readSpeedData(motorSerial, MOTOR_ID);
    vel_msg.header.stamp.sec = current_time / 1000000000;  // seconds
    vel_msg.header.stamp.nanosec = current_time % 1000000000;  // nanoseconds
#ifdef LEFT_WHEEL
    vel_msg.twist.linear.x = -wheelSpeed;
#elif defined(RIGHT_WHEEL)
    vel_msg.twist.linear.x = wheelSpeed;
#endif
}

// Executes the ROS 2 executor for a specified duration and handles any occurring errors
void handleExecutorSpin() {
    // Spin the executor for 10 milliseconds
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    if (ret != RCL_RET_OK) {
        // If an error occurs, retrieve and print the error message
        printf("Error in rclc_executor_spin_some: %s\n", rcl_get_error_string().str);
        rcl_reset_error();  // Reset the error state to prevent propagation
    }
}