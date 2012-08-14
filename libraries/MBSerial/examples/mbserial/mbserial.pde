/**
 * Copyright (c) 2011 Marije Baalman. All rights reserved
 *
 * This file is part of the MBSerial library.
 *
 * MBSerial is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MBSerial is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MBSerial.  If not, see <http://www.gnu.org/licenses/>.
 */

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
