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


/// in the header file of the MiniBee you can disable some options to save
/// space on the MiniBee. If you don't the board may not work as it runs
/// out of RAM.

/// Wire needs to be included if TWI is enabled

#include <Wire.h>

#include <LIS302DL.h>
#include <ADXL345.h>
#include <TMP102.h>
#include <BMP085.h>
#include <HMC5843.h>

#include <XBee.h>
#include <MiniBee_APIn.h>


/// this example shows how to use a stepper motor to be controlled from custom pins
#include <Stepper.h>

/// our motor has 7.5 degree steps, so 48 per full rotation
#define STEPS 48

MiniBee_API Bee = MiniBee_API();

/// stepper motor will be attached to pin 9 and 10
Stepper stepper(STEPS, 12, 13);

/// this will be our parser for the custom messages we will send:
/// msg[0] and msg[1] will be msg type ('E') and message ID
/// the remainder are the actual contents of the message
/// if you want to send several kinds of messages, you can e.g.
/// switch based on msg[2] for message type
void customMsgParser( uint8_t * msg, uint8_t size, uint16_t source ){
     if ( msg[2] > 0 ){ // change speed yes/no
       stepper.setSpeed( msg[3] ); // second argument of message is the speed
     }
     if ( msg[4] == 0 ) // third argument is the direction
       stepper.step( -1 * msg[5] ); // fourth argument is the amount of steps to do
     else
       stepper.step( msg[5] ); // fourth argument is the amount of steps to do
}

void setup() {
  Bee.setup(57600, 'D' ); // arguments are the baudrate, and the board revision
  
/// set a default speed for our stepper
  stepper.setSpeed( 60 );

  /// define which pins we will be using for our custom functionality:
  /// arguments are: pin number, size of data they will produce (in bytes)
  /// in our case, pins 9 and 10, and we don't output any data from them
  Bee.setCustomPin( 9, 0 );
  Bee.setCustomPin( 10, 0 );

  // set the custom message function
  Bee.setCustomCall( &customMsgParser );
}

void loop() {  
  // do a loop step of the remaining firmware:
  Bee.loopStep();
}
