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

class MotorController {
private:
    HardwareSerial& motorSerial;

public:
    MotorController(HardwareSerial& serial)
    : motorSerial(serial) {}

    void sendCommand(byte motorID, uint16_t address, byte command, uint32_t data);
};

struct VelocityCommand {
  float linear_x;
  float angular_z;
};


extern HardwareSerial motorSerial;
extern MotorController motorController;

// 現在の位置と姿勢
extern double x_position;
extern double y_position;
extern double theta; // ロボットの向き（ラジアン）

extern bool initial_data_received; // データ受信の有無を追跡
extern unsigned long last_receive_time; // 最後にデータを受信した時刻

extern VelocityCommand currentCommand;

//extern unsigned long lastReadTime;




void initializeUART();
void initMotor(HardwareSerial& serial, byte motorID);
void sendMotorCommands(float linearVelocity, float angularVelocity);
uint32_t velocityToDEC(float velocityMPS);
void sendVelocityDEC(HardwareSerial& serial, int velocityDec, byte motorID);

float readSpeedData(HardwareSerial& serial, byte motorID);
uint32_t reverseBytes(uint32_t value);
float calculateVelocityMPS(int32_t dec);

/*byte calculateChecksum(byte *data, int len);
*/


constexpr byte MOTOR_ID = 0x01;

// UARTピン設定
constexpr int RX_PIN = 16; // UARTのRXピン
constexpr int TX_PIN = 17; // UARTのTXピン

// オブジェクトアドレス
constexpr uint16_t OPERATION_MODE_ADDRESS = 0x7017;
constexpr uint16_t EMERGENCY_STOP_ADDRESS = 0x701F;
constexpr uint16_t CONTROL_WORD_ADDRESS = 0x7019;
constexpr uint16_t TARGET_VELOCITY_DEC_ADDRESS = 0x70B2;
constexpr uint16_t ACTUAL_SPEED_DEC_ADDRESS = 0x7077;

// コマンド定義
constexpr byte MOTOR_SETUP_COMMAND = 0x51;
constexpr byte MOTOR_ENABLE_COMMAND = 0x52;
constexpr byte VEL_SEND_COMMAND = 0x54;
constexpr byte READ_COMMAND = 0x52;
constexpr byte READ_DEC_COMMAND = 0xA0;
constexpr byte READ_DEC_SUCCESS = 0xA4;

// デフォルト値
constexpr uint32_t OPERATION_MODE_SPEED_CONTROL = 0x00000003;
constexpr uint32_t DISABLE_EMERGENCY_STOP = 0x00000000;
constexpr uint32_t ENABLE_MOTOR = 0x0000000F;
constexpr uint32_t NO_DATA = 0x00000000;

// 通信設定
constexpr int BAUD_RATE = 115200;
constexpr byte ERROR_BYTE = 0x00; // エラーバイトは必要に応じて調整

// ディレイ設定
constexpr uint16_t COMMAND_DELAY = 100; // コマンド間のディレイ
constexpr uint32_t SEND_INTERVAL = 1000; // 速度コマンドの送信間隔 (ミリ秒)

// モーター仕様
constexpr float WHEEL_RADIUS = 0.055; // 車輪の半径 (メートル)
constexpr float WHEEL_DISTANCE = 0.202; // ホイール間の距離 (メートル)

#define RECEIVE_TIMEOUT 5000 // タイムアウト値（ミリ秒）

#define SCALE_FACTOR 1000 // 1000倍して整数演算を行う

const float WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * PI / 60.0 * SCALE_FACTOR; // 60で割るのもここで行う
/*
BluetoothSerial SerialBT;
*/

//const unsigned int readInterval = 40; 

#endif