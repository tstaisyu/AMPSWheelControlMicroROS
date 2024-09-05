#include <M5Stack.h>
#include "SetupM5stack.h"

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
