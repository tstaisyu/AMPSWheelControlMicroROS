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

#ifndef ROS_COMMUNICATIONS_H
#define ROS_COMMUNICATIONS_H

#include <M5Stack.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rcutils/time.h"
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>

// Constants for system-wide parameters
#define TIMER_INTERVAL 20 // Timer callback interval in milliseconds
#define GRAVITY 9.81f // Earth's gravity in m/s^2
#define DEG2RAD 0.0174533f // Degrees to radians conversion factor

// ROS 2 Communication Interfaces
extern rcl_subscription_t com_check_subscriber;  // Handles incoming communication check requests
extern rcl_publisher_t com_check_publisher;      // Responds to communication check requests
extern std_msgs__msg__Int32 com_req_msg;         // Stores incoming communication request data
extern std_msgs__msg__Int32 com_res_msg;         // Stores outgoing communication response data

extern rcl_service_t reboot_service;             // Service for processing reboot requests
extern std_srvs__srv__Trigger_Request request;   // Stores incoming reboot request data
extern std_srvs__srv__Trigger_Response response; // Stores outgoing reboot response data

extern rcl_subscription_t cmd_vel_subscriber;    // Receives velocity commands for the robot
extern geometry_msgs__msg__Twist msg_sub;        // Stores subscribed velocity command data

extern rcl_publisher_t vel_publisher;            // Publishes velocity data as stamped messages
extern geometry_msgs__msg__TwistStamped vel_msg; // Stores velocity data to be published

extern rcl_publisher_t imu_publisher;            // Publishes IMU data for system components
extern sensor_msgs__msg__Imu imu_msg;            // Stores IMU data to be published

extern rcl_publisher_t heartbeat_publisher;     // Publishes heartbeat messages for system monitoring
extern rcl_subscription_t heartbeat_subscriber; // Receives heartbeat messages for system monitoring
extern std_msgs__msg__Int32 heartbeat_msg;     // Stores heartbeat data to be published

// Timing and Scheduling Interfaces
extern rcl_timer_t timer;                        // Manages timing for regular system updates
extern rcl_time_point_value_t current_time;      // Stores the current system time point
extern rcl_clock_t ros_clock;                    // Provides ROS system time

extern rclc_executor_t executor;                 // Manages callback execution for ROS nodes
extern rclc_support_t support;                   // Provides context support for the ROS node
extern rcl_allocator_t allocator;                // Allocates memory for node operations
extern rcl_node_t node;                          // Represents the micro-ROS node

//rcl_init_options_t init_options; // Humble
//size_t domain_id = 117;

// Error handling macros for ROS 2 operations
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        Serial.println("Error in " #fn); \
        return; \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
}

// Function prototypes for ROS 2 initialization and operations
void setupMicroROS();
void initializePublishers(rcl_node_t *node);
void initializeSubscribers(rcl_node_t *node);
void initializeServices(rcl_node_t *node);
#ifdef LEFT_WHEEL
void initializeIMU(rcl_node_t *node);
#endif
void initializeTimer(rcl_timer_t *timer, rclc_support_t *support);
void initializeExecutor(rclc_executor_t *executor, rclc_support_t *support, rcl_allocator_t *allocator);

void com_check_callback(const void * msgin);
void heartbeat_callback(const void * msgin);
void reboot_callback(const void * request, void * response);
void subscription_callback(const void * msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void updateIMUData();
void updateWheelSpeed();
void handleExecutorSpin();

#endif // ROS_COMMUNICATIONS_H