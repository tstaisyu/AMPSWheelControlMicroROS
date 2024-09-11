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

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rcutils/time.h"
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

extern rcl_subscription_t subscriber;
extern geometry_msgs__msg__Twist msg_sub;
extern geometry_msgs__msg__Twist msg_pub;
extern rcl_publisher_t vel_publisher;
extern nav_msgs__msg__Odometry odom_msg;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rcl_timer_t timer;
//rcl_init_options_t init_options; // Humble
//size_t domain_id = 117;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {Serial.println("Error in " #fn); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void setupMicroROS();
void subscription_callback(const void * msgin);
/*
void logReceivedData(const geometry_msgs__msg__Twist *msg);
void updateDisplay(const geometry_msgs__msg__Twist *msg);
void initMotor(HardwareSerial& serial, byte motorID);
void updateOdometry(float rightWheelSpeed, float leftWheelSpeed);
void prepareAndPublishOdometry(double x, double y, double theta, double linear_velocity, double angular_velocity);
void setQuaternionFromYaw(double yaw, geometry_msgs__msg__Quaternion *orientation);
*/

void handleDataPublishing();
void handleExecutorSpin();

#endif