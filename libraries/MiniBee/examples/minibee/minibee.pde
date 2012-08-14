/**
 * Copyright (c) 2009-11 Marije Baalman, Vincent de Belleval. All rights reserved
 *
 * This file is part of the MiniBee library.
 *
 * MiniBee is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MiniBee is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MiniBee.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Wire.h>

#include <LIS302DL.h>
#include <ADXL345.h>
#include <TMP102.h>
#include <BMP085.h>
#include <HMC5843.h>

#include <MiniBee.h>

MiniBee Bee = MiniBee();

void setup() {
  Bee.begin(57600);
}

void loop() {
  Bee.doLoopStep();
}
