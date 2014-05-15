/**
 * Copyright (c) 2011 Marije Baalman. All rights reserved
 *
 * This file is part of the MiniBee API library.
 *
 * MiniBee_API is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MiniBee_API is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MiniBee_API.  If not, see <http://www.gnu.org/licenses/>.
 */

/// Wire needs to be included if TWI is enabled
#include <Wire.h>
/// in the header file of the MiniBee you can disable some options to save
/// space on the MiniBee. If you don't the board may not work as it runs
/// out of RAM.
#include <LIS302DL.h>
#include <ADXL345.h>
#include <TMP102.h>
#include <BMP085.h>
#include <HMC5843.h>

/// includes needed in all cases
#include <XBee.h>
#include <MiniBee_APIn.h>

MiniBee_API Bee = MiniBee_API();

uint8_t privateData[5];

uint8_t count;

void setup() {
  Bee.setup(57600,'D');
  count = 0;
  
  // our meaningful private data - this could for example be data stored in and read from the eeprom
  privateData[0] = 5;
  privateData[1] = 15;
  privateData[2] = 25;
  privateData[3] = 35;
  privateData[4] = 45;
}

void loop() {
  /// increase our counter
  count++;
  
  if ( count > 200 ){ // every 200 steps, we want to send our privately defined data
    bool succes = Bee.sendPrivateData( privateData, 5, true ); // 
    count = 0;
  }
  
  // do a loop step of the remaining firmware:
  Bee.loopStep();
}
