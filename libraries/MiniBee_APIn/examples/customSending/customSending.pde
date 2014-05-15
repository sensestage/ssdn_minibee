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

/// in our example we are using the capacitive sensing library to use sensors
/// not supported by default in our library
#include <CapSense.h>

MiniBee_API Bee = MiniBee_API();

/// using pin 11 as send pin, and pins 11, 12 and 13 as sensing pins
 // 10M resistor between pins 4 & 6, pin 6 is sensor pin, add a wire and or foil
 CapSense   cs_11_10 = CapSense(11,10);       
 CapSense   cs_11_9 = CapSense(11,9); 
 CapSense   cs_11_8 = CapSense(11,8); 

/// variables for sensing our data
int capData[3];
long total[3];

void setup() {
  Bee.setup(57600,'D');

  // define which pins we will be using for our custom functionality:
  // arguments are: pin number, size of data they will produce (in bytes)
  /// in our case we use pin 10 (no data)
  /// and pins 11, 12, and 13, each 2 bytes, as we are going to send integers
  Bee.setCustomPin( 11, 0 );
  Bee.setCustomPin( 10, 2);
  Bee.setCustomPin( 9, 2 );
  Bee.setCustomPin( 8, 2 );
  
  // if you generate data without a pin associated, set the pin number to 0.
  // Bee.setCustomPin( 0, 2 );
  // if you generate data without a pin associated, use setCustomInput
//   Bee.setCustomInput( 1, 1 ); //uint8_t noInputs, uint8_t size );
}

void loop() {
  /// do our measurements
  total[0] =  cs_11_10.capSense(30);
  total[1] =  cs_11_9.capSense(30);
  total[2] =  cs_11_8.capSense(30);

  for ( uint8_t j=0; j<3; j++ ){
    capData[j] = (int) total[j];
  }
  // add our customly measured data to the data package:
  Bee.addCustomData( capData, 3 );
  // do a loop step of the remaining firmware:
  Bee.loopStep();
}
