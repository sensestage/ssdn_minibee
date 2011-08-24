#include <Wire.h>

#include <LIS302DL.h>
#include <ADXL345.h>
#include <TMP102.h>
#include <BMP085.h>
#include <HMC5843.h>

#include <MBSerial.h>

MBSerial Bee = MBSerial();

void setup() {
  Bee.begin(57600);
}

void loop() {
  Bee.doLoopStep();
}
