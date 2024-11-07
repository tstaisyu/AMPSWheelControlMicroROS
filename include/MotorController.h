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

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <M5Stack.h>
#include <HardwareSerial.h>

// Class to manage motor commands through UART
class MotorController {
private:
    HardwareSerial& motorSerial; // Reference to the hardware serial port used by the motor controller

public:
    // Constructor to initialize the motor controller with a specific serial port
    MotorController(HardwareSerial& serial) : motorSerial(serial) {}

    // Sends a command to the motor controller
    void sendCommand(byte motorID, uint16_t address, byte command, uint32_t data);
};

// Structure to store velocity commands with linear and angular components
struct VelocityCommand {
    float linear_x;  // Linear velocity in meters per second
    float angular_z; // Angular velocity in radians per second
};

// Global variables for system state tracking
extern HardwareSerial motorSerial;           // Global instance of hardware serial for motor communications
extern MotorController motorController;      // Global instance of the motor controller

extern double x_position;                    // Current x position of the robot
extern double y_position;                    // Current y position of the robot
extern double theta;                         // Current orientation of the robot in radians

extern bool initial_data_received;           // Flag to check if initial data has been received
extern unsigned long last_receive_time;      // Timestamp of the last received data

extern VelocityCommand currentCommand;       // Current velocity command being processed

// Function prototypes for UART and motor initialization and command transmission
void initializeUART();                                   // Initializes UART for communication
void initMotor(HardwareSerial& serial, byte motorID);    // Initializes motor controller settings
void sendMotorCommands(float linearVelocity, float angularVelocity);  // Sends velocity commands to the motor
uint32_t velocityToDEC(float velocityMPS);                // Converts velocity from m/s to a DEC value
void sendVelocityDEC(HardwareSerial& serial, int velocityDec, byte motorID); // Sends velocity in DEC format

float readSpeedData(HardwareSerial& serial, byte motorID);    // Reads speed data from motor
uint32_t reverseBytes(uint32_t value);                       // Utility function to reverse byte order
float calculateVelocityMPS(int32_t dec);                     // Calculates velocity in m/s from DEC value

// Constants and macros for motor control parameters
constexpr byte MOTOR_ID = 0x01;  // Default motor ID

// Pin configuration for UART
constexpr int RX_PIN = 16;  // RX pin for UART
constexpr int TX_PIN = 17;  // TX pin for UART

// Object addresses for various motor control functions
constexpr uint16_t OPERATION_MODE_ADDRESS = 0x7017;
constexpr uint16_t EMERGENCY_STOP_ADDRESS = 0x701F;
constexpr uint16_t CONTROL_WORD_ADDRESS = 0x7019;
constexpr uint16_t TARGET_VELOCITY_DEC_ADDRESS = 0x70B2;
constexpr uint16_t ACTUAL_SPEED_DEC_ADDRESS = 0x7077;

// Command definitions for motor control
constexpr byte MOTOR_SETUP_COMMAND = 0x51;
constexpr byte MOTOR_ENABLE_COMMAND = 0x52;
constexpr byte VEL_SEND_COMMAND = 0x54;
constexpr byte READ_COMMAND = 0x52;
constexpr byte READ_DEC_COMMAND = 0xA0;
constexpr byte READ_DEC_SUCCESS = 0xA4;

// Default values for motor operations
constexpr uint32_t OPERATION_MODE_SPEED_CONTROL = 0x00000003;
constexpr uint32_t DISABLE_EMERGENCY_STOP = 0x00000000;
constexpr uint32_t ENABLE_MOTOR = 0x0000000F;
constexpr uint32_t NO_DATA = 0x00000000;

// Communication settings
constexpr int BAUD_RATE = 115200;  // UART baud rate
constexpr byte ERROR_BYTE = 0x00;  // Default error byte, adjust as needed

// Timing settings for motor commands
constexpr uint16_t COMMAND_DELAY = 100;           // Delay between commands in milliseconds
constexpr uint32_t SEND_INTERVAL = 1000;          // Interval for sending speed commands in milliseconds

// Motor specifications
constexpr float WHEEL_RADIUS = 0.055;            // Radius of the wheel in meters
constexpr float WHEEL_DISTANCE = 0.202;          // Distance between wheels in meters

#define RECEIVE_TIMEOUT 5000 // Timeout value in milliseconds for receiving data
#define SCALE_FACTOR 1000    // Factor to scale values for integer calculations

const float WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * PI / 60.0 * SCALE_FACTOR; // Calculates wheel circumference

#endif // MOTOR_CONTROLLER_H