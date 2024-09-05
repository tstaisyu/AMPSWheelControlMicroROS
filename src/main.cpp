#ifdef NATIVE_TEST
#include "mocks/MockM5Stack.h"
#else
#include <M5Stack.h>
#endif
#include "SetupM5stack.h"

void setup() {

  setupM5stack();  

}

void loop() {

}

