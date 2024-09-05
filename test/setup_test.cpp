#include <ArduinoFake.h>
#include <AUnit.h>
#include "SetupM5stack.h" // 定義された関数をインクルード

using namespace aunit;

test(setupM5stackFunction) {
    ArduinoFakeReset();

    // M5Stack関連のメソッドをモック化
    When(Method(ArduinoFake(M5Stack), begin)).AlwaysReturn(true);
    When(Method(ArduinoFake(M5Stack), Lcd.setTextSize)).AlwaysReturn();
    When(Method(ArduinoFake(M5Stack), Lcd.setCursor)).AlwaysReturn();

    // setupM5stack() をテスト実行
    setupM5stack();

    // モックの呼び出しが期待通りに行われたか検証
    Verify(Method(ArduinoFake(M5Stack), begin)).Once();
    Verify(Method(ArduinoFake(M5Stack), Lcd.setTextSize).Using(2)).Once();
    Verify(Method(ArduinoFake(M5Stack), Lcd.setCursor).Using(0, 0)).Once();
}
