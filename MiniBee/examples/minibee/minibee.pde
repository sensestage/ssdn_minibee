#include <Wire.h>
#include <LIS302DL.h>
#include <ADXL345.h>
#include <MiniBee.h>

MiniBee Bee = MiniBee();

LIS302DL accelLIS;

void setup() {
  Bee.begin(57600);
}

void loop() {
  Bee.doLoopStep();
}
