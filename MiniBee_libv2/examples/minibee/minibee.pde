#include <Wire.h>
#include <ADXL345.h>
#include <MiniBeeV2.h>

MiniBeeV2 Bee = MiniBeeV2();

void setup() {
  Bee.begin(19200);
}

void loop() {
  Bee.doLoopStep();
}
