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

/// Wire needs to be included if TWI is enabled
#include <Wire.h>

#include <LIS302DL.h>
#include <ADXL345.h>
#include <TMP102.h>
#include <BMP085.h>
#include <HMC5843.h>

/// in the header file of the MiniBee you can disable some options to save
/// space on the MiniBee. If you don't the board may not work as it runs
/// out of RAM.
#include <MiniBee.h>

MiniBee Bee = MiniBee();

int compassAddress = 0x32 >> 1; // From datasheet compass address is 0x32 for write operations, 
                                // or 0x33 for read operations.
                                // shift the address 1 bit right, the Wire library only needs the 7
                                // most significant bits for the address

// int heading = 0; // variable to hold the heading angle
// int tilt = 0;    // variable to hold the tilt angle
// int roll = 0;    // variable to hold the roll angle

byte responseBytes[6];  // for holding the sensor response bytes
int results[3];

void setup() {
  Bee.begin(57600);

  Wire.begin();
    
  Bee.setCustomInput( 3, 2 );  
}

void loop() {
  readHMC6343();
  // add our customly measured data to the data package:
  Bee.addCustomData( results, 3 );
  // do a loop step of the remaining firmware:
  Bee.doLoopStep();
}

void readHMC6343() 
{ 
  // step 1: instruct sensor to read echoes 
  Wire.beginTransmission(compassAddress); // transmit to device
                          // the address specified in the datasheet is 66 (0x42) 
                          // but i2c adressing uses the high 7 bits so it's 33 
  Wire.send(byte(0x50)); // Send a "Post Heading Data" (0x50) command to the HMC6343  
  Wire.endTransmission(); // stop transmitting 
 
  // step 2: wait for readings to happen 
  delay(2); // datasheet suggests at least 1 ms 
  
  // step 3: request reading from sensor 
  Wire.requestFrom(compassAddress, 6); // request 6 bytes from slave device #33 
 
  // step 4: receive reading from sensor 
  if (6 <= Wire.available()) // if six bytes were received 
  {
    for (int i = 0; i<6; i++) {
      responseBytes[i] = Wire.receive();
    }
  }
  
  results[0] = ((int)responseBytes[0]<<8) | ((int)responseBytes[1]); // heading MSB and LSB
  results[1] = (((int)responseBytes[2]<<8) | ((int)responseBytes[3]));  // tilt MSB and LSB
  results[2] = (((int)responseBytes[4]<<8) | ((int)responseBytes[5]));  // roll MSB and LSB
}

