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

HardwareSerial motorSerial(2);
MotorController motorController(motorSerial);

double x_position = 0.0;
double y_position = 0.0;
double theta = 0.0;
bool initial_data_received = false;
unsigned long last_receive_time = 0;
VelocityCommand currentCommand;
unsigned long lastReadTime = 0;

void initializeUART() {
    Serial.begin(BAUD_RATE);
    while (!Serial);  // シリアルポートが開くのを待つ
    motorSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("Setup complete. Ready to read high resolution speed data.");
    initMotor(motorSerial, MOTOR_ID);
    M5.Lcd.print("micro ROS2 M5Stack START\n");    
}

void initMotor(HardwareSerial& serial, byte motorID) {
    motorController.sendCommand(motorID, OPERATION_MODE_ADDRESS, MOTOR_SETUP_COMMAND, OPERATION_MODE_SPEED_CONTROL);
    delay(COMMAND_DELAY);
    motorController.sendCommand(motorID, EMERGENCY_STOP_ADDRESS, MOTOR_SETUP_COMMAND, DISABLE_EMERGENCY_STOP);
    delay(COMMAND_DELAY);
    motorController.sendCommand(motorID, CONTROL_WORD_ADDRESS, MOTOR_ENABLE_COMMAND, ENABLE_MOTOR);
    delay(COMMAND_DELAY);
}

void MotorController::sendCommand(byte motorID, uint16_t address, byte command, uint32_t data) {
    byte packet[] = {motorID, command, highByte(address), lowByte(address), ERROR_BYTE, (byte)(data >> 24), (byte)(data >> 16), (byte)(data >> 8), (byte)data};
    byte checksum = 0;
    for (int i = 0; i < sizeof(packet); i++) {
        checksum += packet[i];
    }
    
    // 条件に基づいて特定のバイトが特定の値の場合にのみパケット全体を表示
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

    if ((packet[1] == 0xA4 && packet[3] == 0x77)) {
        M5.Lcd.setCursor(0, 140);
        M5.Lcd.print("Packet: ");
        for (int i = 0; i < sizeof(packet); i++) {
            M5.Lcd.printf("%02X ", packet[i]);
        }
        M5.Lcd.println();
        M5.Lcd.setCursor(0, 180);
        M5.Lcd.printf("Checksum: %02X", checksum);
        M5.Lcd.println();
    }

    motorSerial.write(packet, sizeof(packet));
    motorSerial.write(checksum);
}

void sendMotorCommands(float linearVelocity, float angularVelocity) {
    #ifdef LEFT_WHEEL
    // 左輪用の設定
    float wheelSpeed = (-1) * (linearVelocity + (WHEEL_DISTANCE * angularVelocity / 2));
    #elif defined(RIGHT_WHEEL)
    // 右輪用の設定
    float wheelSpeed = linearVelocity - (WHEEL_DISTANCE * angularVelocity / 2);
    #endif

  int wheelDec = velocityToDEC(wheelSpeed);

  // 右輪と左輪に速度指令を送信
  sendVelocityDEC(motorSerial, wheelDec, MOTOR_ID);
}

uint32_t velocityToDEC(float velocityMPS) {
    float wheelCircumference = WHEEL_RADIUS * 2 * PI;
    float rpm = (velocityMPS * 60.0) / wheelCircumference;
    return static_cast<uint32_t>((rpm * 512.0 * 4096.0) / 1875.0);
}

void sendVelocityDEC(HardwareSerial& serial, int velocityDec, byte motorID) {
  motorController.sendCommand(motorID, TARGET_VELOCITY_DEC_ADDRESS, VEL_SEND_COMMAND, velocityDec);
}

float readSpeedData(HardwareSerial& serial, byte motorID) {
    motorController.sendCommand(motorID, ACTUAL_SPEED_DEC_ADDRESS, READ_DEC_COMMAND, 0);
    if (serial.available() >= 10) {
        uint8_t response[10];
        serial.readBytes(response, 10);
        uint16_t responseAddress = ((uint16_t)response[2] << 8) | response[3];
        if (response[0] == motorID && response[1] == READ_DEC_SUCCESS && responseAddress == ACTUAL_SPEED_DEC_ADDRESS) {
            int32_t receivedDec;
            memcpy(&receivedDec, &response[5], sizeof(receivedDec));
            receivedDec = reverseBytes(receivedDec);
            float velocityMPS = calculateVelocityMPS(receivedDec);

            // LCDに生のDECデータと速度MPSを表示
//            M5.Lcd.setCursor(0, 80);  // 表示位置設定
//            M5.Lcd.printf("DEC: %ld, Speed: %.2f m/s", receivedDec, velocityMPS);

            return velocityMPS;
        }
    }
    // データがない場合の表示
//    M5.Lcd.setCursor(0, 80);
//    M5.Lcd.print("No data available");
    return 0.0;
}

uint32_t reverseBytes(uint32_t value) {
    return ((value & 0x000000FF) << 24) |
           ((value & 0x0000FF00) << 8) |
           ((value & 0x00FF0000) >> 8) |
           ((value & 0xFF000000) >> 24);
}

float calculateVelocityMPS(int32_t dec) {
    int scaledRPM = (dec * 1875) / (512 * 4096);
    return (scaledRPM * WHEEL_CIRCUMFERENCE) / SCALE_FACTOR;
}
