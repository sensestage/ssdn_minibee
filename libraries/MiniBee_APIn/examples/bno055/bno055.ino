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
#include <ADXL345.h>

// not actually used
#include <LIS302DL.h>
#include <TMP102.h>
#include <BMP085.h>
#include <HMC5843.h>

/// includes needed in all cases
#include <XBee.h>
#include <MiniBee_APIn.h>

#include <BNO055.h>


// #include <Adafruit_NeoPixel.h>
// #ifdef __AVR__
//   #include <avr/power.h>
// #endif

// // Which pin on the Arduino is connected to the NeoPixels?
// // On a Trinket or Gemma we suggest changing this to 1
// #define NEOPIN            3

// // How many NeoPixels are attached to the Arduino?
// #define NUMPIXELS      2



// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
// Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

BNO055 bno = BNO055();
MiniBee_API Bee = MiniBee_API();

boolean bnoActive = true;

/// this will be our parser for the custom messages we will send:
/// msg[0] and msg[1] will be msg type ('E') and message ID
/// the remainder are the actual contents of the message
/// if you want to send several kinds of messages, you can e.g.
/// switch based on msg[2] for message type
// void customMsgParser( uint8_t * msg, uint8_t size, uint16_t source ){
// 
//     pixels.setPixelColor(0, pixels.Color( msg[2], msg[3], msg[4]));
//     pixels.setPixelColor(1, pixels.Color( msg[5], msg[6], msg[7]));
// 
//     pixels.show(); // This sends the updated pixel color to the hardware.
// 
// }


void setup() {
  Bee.setup(57600,'F');

//   pixels.begin(); // This initializes the NeoPixel library.
  // neopixel
//   Bee.setCustomInput( 3, 0 ); //uint8_t noInputs, uint8_t size );
  // set the custom message function
//   Bee.setCustomCall( &customMsgParser );
  
  if(!bno.begin())
  {
    bnoActive = false;
  }
  
  if ( bnoActive ){
    bno.setExtCrystalUse(true);
    // if you generate data without a pin associated, use setCustomInput
    Bee.setCustomInput( 22, 2 ); //uint8_t noInputs, uint8_t size );
    // if you generate data without a pin associated, use setCustomInput
    Bee.setCustomInput( 2, 1 ); //uint8_t noInputs, uint8_t size );
  }

}

int16_t data[24];
uint8_t data8[2];

void loop() {
  if ( bnoActive ){
    IntVector<3> acc = bno.getVector(BNO055::VECTOR_ACCELEROMETER);

    data[0] = acc.x();
    data[1] = acc.y();
    data[2] = acc.z();
    
    IntVector<3> linacc = bno.getVector(BNO055::VECTOR_LINEARACCEL);
    data[3] = linacc.x();
    data[4] = linacc.y();
    data[5] = linacc.z();

    IntVector<3> grav = bno.getVector(BNO055::VECTOR_GRAVITY);
    data[6] = grav.x();
    data[7] = grav.y();
    data[8] = grav.z();

    IntVector<3> magv = bno.getVector(BNO055::VECTOR_MAGNETOMETER);
    data[9] = magv.x();
    data[10] = magv.y();
    data[11] = magv.z();
  
    IntVector<3> gyrov = bno.getVector(BNO055::VECTOR_GYROSCOPE);
    data[12] = gyrov.x();
    data[13] = gyrov.y();
    data[14] = gyrov.z();

    IntVector<3> euler = bno.getVector(BNO055::VECTOR_EULER);
    data[15] = euler.x();
    data[16] = euler.y();
    data[17] = euler.z();

    Quaternion quat = bno.getQuat();
    data[18] = quat.w();
    data[19] = quat.x();
    data[20] = quat.y();
    data[21] = quat.z();
    
    data8[0] = bno.getTemp();
    data8[1] = bno.getCalibrationRaw();
    
    // add our customly measured data to the data package:
    Bee.addCustomData( data, 22 );
    // add our customly measured data to the data package:
    Bee.addCustomData( data8, 2 );
  }
  // do a loop step of the remaining firmware:
  Bee.loopStep();
}
