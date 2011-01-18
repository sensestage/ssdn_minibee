#include <Wire.h>

#include <LIS302DL.h>
#include <ADXL345.h>
#include <TMP102.h>
#include <BMP085.h>

#include <MiniBee.h>

MiniBee Bee = MiniBee();

void setup() {
  Bee.begin(115200);
}

void loop() {
  Bee.doLoopStep();
}
