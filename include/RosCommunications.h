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

#define INTERVAL 20 // Interval for timer callback in milliseconds

#define GRAVITY 9.81f
#define DEG2RAD 0.0174533f

// Communication Check: Publisher and Subscriber for integrity check messages
extern rcl_subscription_t com_check_subscriber;  // Subscriber for communication check requests
extern rcl_publisher_t com_check_publisher;      // Publisher for communication check responses
extern std_msgs__msg__Int32 com_req_msg;          // Message for incoming communication requests
extern std_msgs__msg__Int32 com_res_msg;          // Message for outgoing communication responses

// Reboot service: Handles requests to reboot the system safely
extern rcl_service_t reboot_service;              // Service to manage reboot requests
extern std_srvs__srv__Trigger_Request request;        // Reboot request message
extern std_srvs__srv__Trigger_Response response;       // Reboot response message

// cmd_vel subscriber: Subscribes to velocity commands for the robot
extern rcl_subscription_t cmd_vel_subscriber;     // Subscriber for velocity commands
extern geometry_msgs__msg__Twist msg_sub;         // Message type for subscribing to velocity commands

// Velocity publisher: Publishes velocity commands as stamped messages
extern rcl_publisher_t vel_publisher;             // Publisher for velocity data
extern geometry_msgs__msg__TwistStamped vel_msg;  // Stamped message for velocity data

// IMU publisher: Publishes IMU data to other components in the system
extern rcl_publisher_t imu_publisher;             // Publisher for IMU data
extern sensor_msgs__msg__Imu imu_msg;             // IMU message type

// Timer callback: Manages timing for regular updates in the system
extern rcl_timer_t timer;                         // Timer for periodic updates
extern rcl_time_point_value_t current_time;       // Stores the current time point
extern rcl_clock_t ros_clock;                     // Clock to manage system time

// microROS node and executor: Core components for managing ROS 2 nodes and callbacks
extern rclc_executor_t executor;                  // Executor for managing callbacks
extern rclc_support_t support;                    // Support structure for the node
extern rcl_allocator_t allocator;                 // Allocator for the node's resources
extern rcl_node_t node;                           // The node itself

//rcl_init_options_t init_options; // Humble
//size_t domain_id = 117;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {Serial.println("Error in " #fn); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void setupMicroROS();
// Prototype declaration for each initialization function
void initializePublishers(rcl_node_t *node);
void initializeSubscribers(rcl_node_t *node);
void initializeServices(rcl_node_t *node);
#ifdef LEFT_WHEEL
void initializeIMU(rcl_node_t *node);
#endif
void initializeTimer(rcl_timer_t *timer, rclc_support_t *support);
void initializeExecutor(rclc_executor_t *executor, rclc_support_t *support, rcl_allocator_t *allocator);
void com_check_callback(const void * msgin);
void reboot_callback(const void * request, void * response);
void subscription_callback(const void * msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void updateIMUData();
void updateWheelSpeed();
void handleExecutorSpin();

#endif