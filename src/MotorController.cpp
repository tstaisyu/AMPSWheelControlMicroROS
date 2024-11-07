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

#include "MotorController.h"

HardwareSerial motorSerial(2); // Using the second hardware serial interface
MotorController motorController(motorSerial); // Initializing the motor controller

double x_position = 0.0; // X position of the robot
double y_position = 0.0; // Y position of the robot
double theta = 0.0; // Orientation angle in radians
bool initial_data_received = false; // Flag to track if initial data has been received
unsigned long last_receive_time = 0; // Timestamp of the last data received
VelocityCommand currentCommand; // Struct to hold the current velocity command

void initializeUART() {
    motorSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN); // Start UART with defined pins and baud rate
    Serial.println("Setup complete. Ready to read high resolution speed data.");
    initMotor(motorSerial, MOTOR_ID); // Initialize motor with settings
    M5.Lcd.print("micro ROS2 M5Stack START\n"); // Display initialization message on M5Stack    
}

void initMotor(HardwareSerial& serial, byte motorID) {
    // Sending a series of setup commands to the motor
    motorController.sendCommand(motorID, OPERATION_MODE_ADDRESS, MOTOR_SETUP_COMMAND, OPERATION_MODE_SPEED_CONTROL);
    delay(COMMAND_DELAY);
    motorController.sendCommand(motorID, EMERGENCY_STOP_ADDRESS, MOTOR_SETUP_COMMAND, DISABLE_EMERGENCY_STOP);
    delay(COMMAND_DELAY);
    motorController.sendCommand(motorID, CONTROL_WORD_ADDRESS, MOTOR_ENABLE_COMMAND, ENABLE_MOTOR);
    delay(COMMAND_DELAY);
}

void MotorController::sendCommand(byte motorID, uint16_t address, byte command, uint32_t data) {
    byte packet[] = {motorID, command, highByte(address), lowByte(address), ERROR_BYTE, (byte)(data >> 24), (byte)(data >> 16), (byte)(data >> 8), (byte)data};
    byte checksum = 0; // Calculate checksum for error checking
    for (int i = 0; i < sizeof(packet); i++) {
        checksum += packet[i];
    }
    
    // Display the entire packet on the LCD only if specific conditions are met
    if ((packet[1] == 0xA4 && packet[3] == 0x77)) {
        M5.Lcd.setCursor(0, 80);
        M5.Lcd.print("Packet: ");
        for (int i = 0; i < sizeof(packet); i++) {
            M5.Lcd.printf("%02X ", packet[i]);
        }
        M5.Lcd.println();
        M5.Lcd.setCursor(0, 120);
        M5.Lcd.printf("Checksum: %02X", checksum);
        M5.Lcd.println();
    }

    motorSerial.write(packet, sizeof(packet)); // Send the packet over serial
    motorSerial.write(checksum); // Send the calculated checksum
}

void sendMotorCommands(float linearVelocity, float angularVelocity) {
    float wheelSpeed;
#ifdef LEFT_WHEEL
    wheelSpeed = (-1) * (linearVelocity - (WHEEL_DISTANCE * angularVelocity / 2)); // Calculate speed for left wheel
#elif defined(RIGHT_WHEEL)
    wheelSpeed = linearVelocity + (WHEEL_DISTANCE * angularVelocity / 2); // Calculate speed for right wheel
#endif
    int wheelDec = velocityToDEC(wheelSpeed); // Convert speed to DEC
    sendVelocityDEC(motorSerial, wheelDec, MOTOR_ID); // Send DEC speed to motor
}

uint32_t velocityToDEC(float velocityMPS) {
    float wheelCircumference = WHEEL_RADIUS * 2 * PI;
    float rpm = (velocityMPS * 60.0) / wheelCircumference; // Convert m/s to RPM
    return static_cast<uint32_t>((rpm * 512.0 * 4096.0) / 1875.0); // Convert RPM to DEC format
}

void sendVelocityDEC(HardwareSerial& serial, int velocityDec, byte motorID) {
    motorController.sendCommand(motorID, TARGET_VELOCITY_DEC_ADDRESS, VEL_SEND_COMMAND, velocityDec); // Send velocity command to motor
}

float readSpeedData(HardwareSerial& serial, byte motorID) {
    motorController.sendCommand(motorID, ACTUAL_SPEED_DEC_ADDRESS, READ_DEC_COMMAND, NO_DATA); // Request current speed data
    if (serial.available() >= 10) { // Check if enough data is available
        uint8_t response[10];
        serial.readBytes(response, 10); // Read the response from the motor
        uint16_t responseAddress = ((uint16_t)response[2] << 8) | response[3];
        if (response[0] == motorID && response[1] == READ_DEC_SUCCESS && responseAddress == ACTUAL_SPEED_DEC_ADDRESS) {
            int32_t receivedDec;
            memcpy(&receivedDec, &response[5], sizeof(receivedDec));
            receivedDec = reverseBytes(receivedDec); // Correct the byte order
            return calculateVelocityMPS(receivedDec); // Convert DEC to m/s and return
        }
    }
    return 0.0; // Return zero if no valid data received
}

uint32_t reverseBytes(uint32_t value) {
    // Reverse byte order for 32-bit unsigned integers
    return ((value & 0x000000FF) << 24) |
           ((value & 0x0000FF00) << 8) |
           ((value & 0x00FF0000) >> 8) |
           ((value & 0xFF000000) >> 24);
}

float calculateVelocityMPS(int32_t dec) {
    int scaledRPM = (dec * 1875) / (512 * 4096); // Convert DEC to RPM
    return (scaledRPM * WHEEL_CIRCUMFERENCE) / SCALE_FACTOR; // Convert RPM to m/s and return
}
