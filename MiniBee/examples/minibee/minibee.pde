#include <Wire.h>
#include <ADXL345.h>
#include <MiniBee.h>

MiniBee Bee = MiniBee();

void setup() {
  Bee.begin(19200);
}

void loop() {
  Bee.doLoopStep();
}
