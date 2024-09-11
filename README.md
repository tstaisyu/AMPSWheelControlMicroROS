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

# AMPS IWS45L1D1350B-MCAFC Wheel Motor Control with microROS

このプロジェクトは、AMPS社のIWS45L1D1350B-MCAFCハブホイールモータを制御し、M5Stackマイクロコントローラーボードを使用してホイールの速度を制御するものです。microROSを活用してエンコーダからの速度情報を読み取り、ROS 2のトピックを介して速度情報を送信します。このシステムは自動的にホイールの速度を調整し、エンコーダーからのフィードバックに基づいてROS 2ネットワーク上でデータを共有します。

## ディレクトリ構成

- `include`: ヘッダーファイルが含まれるディレクトリ
- `src`: 各モジュールのソースコードを含むディレクトリ
- `test`: ユニットテストを含むディレクトリ
- `platformio.ini`: PlatformIOの構成ファイル

## ソースモジュール

- `DisplayManager.cpp` / `DisplayManager.h`:
  - M5StackのLCDディスプレイを管理し、システムのステータスとデータをリアルタイムで表示します。
- `MotorController.cpp` / `MotorController.h`:
  - ハブホイールモータの速度制御命令を生成し、モータへの命令送信を担当します。エンコーダデータの読み取りもこのモジュールで行います。
- `RosCommunications.cpp` / `RosCommunications.h`:
  - microROSを使用してROS 2トピックへの速度情報のパブリッシュと、コマンド速度のサブスクライブを管理します。システムの中核を担う通信処理がここに集約されています。
- `SerialManager.cpp` / `SerialManager.h`:
  - シリアル通信を通じてデバッグ情報やエラーメッセージを出力するためのモジュールです。トラブルシューティング時の情報提供に重要な役割を果たします。
- `SystemManager.cpp` / `SystemManager.h`:
  - システム全体の初期設定やエラーハンドリングを担当。特にM5Stackの初期設定やリスタート処理が含まれます。

## ビルドと実行

PlatformIOを使用してビルドとデプロイを行います。`platformio.ini`ファイルで環境設定を管理しており、各種依存関係やビルドオプションが定義されています。

```bash
platformio run --target upload
```

上記コマンドでビルドからデバイスへの書き込みまで自動で行われます。テストの実行には別途テスト環境が必要です。

## ライセンス

このプロジェクトはApache License 2.0の下で提供されています。詳細については、リポジトリに含まれるLICENSEファイルを参照してください。