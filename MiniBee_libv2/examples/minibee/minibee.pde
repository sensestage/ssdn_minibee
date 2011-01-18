#include <Wire.h>
#include <ADXL345.h>
#include <MiniBeeV2.h>

MiniBeeV2 Bee = MiniBeeV2();

void setup() {
  // board revision A - 8MHz clock --- maybe can now also be faster??
//    Bee.begin(19200);
 // board revision B and up - 12 MHz clock (or higher)
   Bee.begin(115200);
}

void loop() {
  Bee.doLoopStep();
}
