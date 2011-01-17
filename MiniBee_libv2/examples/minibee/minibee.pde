#include <Wire.h>
#include <ADXL345.h>
#include <MiniBeeV2.h>

MiniBeeV2 Bee = MiniBeeV2();

void setup() {
  // board revision A - 8MHz clock
//   Bee.begin(19200);
 // board revision B and up - 12 MHz clock (or higher)
  Bee.begin(57600);
}

void loop() {
  Bee.doLoopStep();
}
