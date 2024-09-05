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

#include <M5Stack.h>
#include "SetupM5stack.h"
#include <unity.h>

void test_setupM5stack_function() {
    setupM5stack();  // テスト対象の関数を呼び出し
    // アサーションを使って、期待する結果を確認する（必要な場合）
}

void setup() {
    delay(2000); // テスト開始前に少し待つ
    UNITY_BEGIN(); // テストランナーを開始
}

void loop() {
    RUN_TEST(test_setupM5stack_function);
    UNITY_END(); // テストランナーを終了
}
