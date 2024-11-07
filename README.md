<!--
  Copyright 2024 Taisyu Shibata

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
-->

# AMPS Hub Wheel Motor Control with microROS

このリポジトリは、microROSを活用してAMPS社のIWS45L1D1350B-MCAFCハブホイールモータを制御し、マイクロコントローラーボードM5Stackを使用してホイールの速度を制御するものです。また、エンコーダからの速度情報を読み取り、ROS 2のトピックを介して速度情報を送信します。このシステムは自動的にホイールの速度を調整し、エンコーダーからのフィードバックに基づいてROS 2ネットワーク上でデータを共有します。

## 特徴

- **高精度なモーター制御**: AMPS社のハブホイールモータを使用し、精密な速度制御を実現。
- **リアルタイムデータ共有**: ROS 2トピックを通じて速度情報やIMUデータをリアルタイムで共有。
- **デバッグとモニタリング**: M5StackのLCDディスプレイを活用してシステムのステータスやデータをリアルタイムで表示。
- **柔軟な構成**: 左右輪ごとにトピック名やサービス名を条件分岐させることで、複数のホイールを独立して制御可能。
- **エラーハンドリング**: データ受信のタイムアウトを監視し、必要に応じてデバイスをリスタート。
- **IMUデータのフィルタリング**: ローパスフィルタを適用してセンサーデータのノイズを低減し、精度を向上。

## ディレクトリ構成

```plaintext
├── include
│   ├── DisplayManager.h
│   ├── IMUManager.h
│   ├── MotorController.h
│   ├── RosCommunications.h
│   ├── SerialManager.h
│   └── SystemManager.h
├── src
│   ├── DisplayManager.cpp
│   ├── IMUManager.cpp
│   ├── MotorController.cpp
│   ├── RosCommunications.cpp
│   ├── SerialManager.cpp
│   └── SystemManager.cpp
├── test
│   └── (ユニットテストファイル)
├── platformio.ini
├── README.md
└── LICENSE
```

- `include`: ヘッダーファイルが含まれるディレクトリ
- `src`: 各モジュールのソースコードを含むディレクトリ
- `test`: ユニットテストを含むディレクトリ
- `platformio.ini`: PlatformIOの構成ファイル
- `README.md`: プロジェクトの概要と使用方法を記載したファイル。
- `LICENSE`: プロジェクトのライセンス情報。

## ビルドと実行

PlatformIOを使用してビルドとデプロイを行います。`platformio.ini`ファイルで環境設定を管理しており、各種依存関係やビルドオプションが定義されています。

```bash
platformio run --target upload
```

上記コマンドでビルドからデバイスへの書き込みまで自動で行われます。テストの実行には別途テスト環境が必要です。

## ソースモジュール

### DisplayManager.cpp / DisplayManager.h

- **概要**: M5StackのLCDディスプレイを管理し、システムのステータスとデータをリアルタイムで表示します。
- **主な機能**:
  - `updateDisplay`: 受信した速度データをLCDに表示します。

### MotorController.cpp / MotorController.h

- **概要**: `MotorController` クラスは、ハブホイールモータの速度制御命令を生成し、モータへの命令送信を担当します。エンコーダデータの読み取りもこのモジュールで行います。
- **主な機能**:
  - `sendCommand`: モータに対して特定のコマンドを送信します。
  - `sendMotorCommands`: 線形および角速度を基にモータへの速度指令を送信します。
  - `velocityToDEC`: 速度をDEC形式に変換します。
  - `readSpeedData`: モータから速度データを読み取ります。
  - `reverseBytes`: バイト順を逆転させます。
  - `calculateVelocityMPS`: DEC値から速度（m/s）を計算します。

### RosCommunications.cpp / RosCommunications.h

- **概要**: microROSを使用してROS 2トピックへの速度情報のパブリッシュと、コマンド速度のサブスクライブを管理します。システムの中核を担う通信処理がここに集約されています。
- **主な機能**:
  - ROS 2のノード、パブリッシャ、サブスクライバ、サービスの初期化と管理。
  - コールバック関数の定義と実装。

### SerialManager.cpp / SerialManager.h

- **概要**: シリアル通信を通じてデバッグ情報やエラーメッセージを出力するためのモジュールです。トラブルシューティング時の情報提供に重要な役割を果たします。
- **主な機能**:
  - `logReceivedData`: 受信した速度データをシリアルコンソールにログ出力します。

### SystemManager.cpp / SystemManager.h

- **概要**: システム全体の初期設定やエラーハンドリングを担当。特にM5Stackの初期設定やリスタート処理が含まれます。
- **主な機能**:
  - `setupM5stack`: M5Stackの初期設定、WiFi接続、時刻同期を行います。
  - `checkDataTimeout`: データ受信のタイムアウトを監視し、必要に応じてデバイスをリスタートします。

### IMUManager.cpp / IMUManager.h

- **概要**: `IMUManager` クラスは、M5Stackの内蔵IMUから加速度とジャイロスコープのデータを取得し、センサのキャリブレーション、フィルタリング、データの取得を担当します。
- **主な機能**:
  - `initialize`: IMUセンサーの初期化とキャリブレーションを行います。
  - `update`: IMUデータの更新とフィルタリングを行います。
  - `getCalibratedData`: フィルタリングされた加速度およびジャイロデータを取得します。
  - `calibrateSensors`: センサーのキャリブレーションを実施します。
  - `applyLowPassFilter`: センサーデータにローパスフィルタを適用します。

## microROSノードに関する説明

- **Reboot service**: システムの安全な再起動を管理するサービスです。
- **cmd_vel subscriber**: ロボットの速度コマンドを購読するためのサブスクライバーです。このサブスクライバーは速度コマンドを受け取り、ロボットの動きを制御します。
- **Velocity publisher**: 速度データをタイムスタンプ付きメッセージとして公開するパブリッシャーです。これにより、システム内の他のコンポーネントは速度データを正確な時間情報と共に利用できます。
- **IMU publisher**: IMU（Inertial Measurement Unit）データをシステム内の他のコンポーネントに公開するパブリッシャーです。IMUデータには、加速度、ジャイロスコープ、時には地磁気データなどが含まれることがあります。
- **Timer callback**: 定期的な更新を管理するためのタイマーです。特定の間隔でシステムの状態を更新する際に使用されます。

## ライセンス

このプロジェクトはApache License 2.0の下で提供されています。詳細については、リポジトリに含まれる`LICENSE`ファイルを参照してください。

## 追加情報

### 環境設定

- **PlatformIO**: プロジェクトのビルドとデプロイにPlatformIOを使用しています。`platformio.ini`ファイルで依存関係やビルドオプションを管理しています。
- **microROS**: ROS 2との通信を行うためにmicroROSを使用しています。microROSのセットアップとノード管理は`RosCommunications`モジュールで行われます。
- **M5Stack**: デバッグ情報やシステムステータスの表示にM5StackのLCDディスプレイを使用しています。

### 使用方法

1. **環境の準備**:
   - PlatformIOをインストールし、必要な依存関係を取得します。
   - M5Stackとモータコントローラーを接続します。

2. **ビルドとデプロイ**:
    ```bash
    platformio run --target upload
    ```
    上記コマンドでビルドからデバイスへの書き込みまで自動で行われます。

3. **システムの監視**:
   - M5StackのLCDディスプレイでリアルタイムの速度データやシステムステータスを確認します。
   - シリアルモニターを使用してデバッグ情報を確認します。

4. **ROS 2ネットワークとの連携**:
   - ROS 2ノードを起動し、速度データやIMUデータをパブリッシュ/サブスクライブします。
   - `cmd_vel`トピックを通じてロボットの動きを制御します。

### トラブルシューティング

- **WiFi接続の問題**:
  - `setupM5stack`関数でWiFiのSSIDとパスワードが正しく設定されていることを確認してください。
  - WiFiが接続されない場合、ルーターの設定や信号強度を確認してください。

- **IMUデータの不正確さ**:
  - `IMUManager`クラスのキャリブレーション機能を再度実行し、センサーのオフセットが正しく設定されていることを確認してください。
  - センサーの位置が水平であることを確認してください。

- **モータ制御の問題**:
  - モータコントローラーとの接続が正しいことを確認してください。
  - シリアル通信の設定（ボーレート、RX/TXピン）が正しく行われていることを確認してください。

### 参考資料

- [microROS公式ドキュメント](https://micro.ros.org/)
- [PlatformIO公式サイト](https://platformio.org/)
- [M5Stack公式ドキュメント](https://docs.m5stack.com/)
