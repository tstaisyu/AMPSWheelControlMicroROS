#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MockM5Stack.h" 
#include "SetupM5stack.h"

// テストケース
TEST(M5StackTest, SetupM5stackFunction) {
    MockM5Stack mockM5;

    // モックの振る舞いを設定
    ON_CALL(mockM5, begin()).WillByDefault(testing::Return(true));
    EXPECT_CALL(mockM5, begin()).Times(1);
    EXPECT_CALL(mockM5, setTextSize(testing::Eq(2))).Times(1);
    EXPECT_CALL(mockM5, setCursor(testing::Eq(0), testing::Eq(0))).Times(1);

    // setupM5stack() をテスト実行
    setupM5stack();
}

// main関数（Google Testのエントリポイント）
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}