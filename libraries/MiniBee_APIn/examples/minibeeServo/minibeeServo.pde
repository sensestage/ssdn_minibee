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


#include <Servo.h> 
 
Servo myservo = Servo();  // create servo object to control a servo 

#define SERVOPIN 9

MiniBee_API Bee = MiniBee_API();

/// this will be our parser for the custom messages we will send:
/// msg[0] and msg[1] will be msg type ('E') and message ID
/// the remainder are the actual contents of the message
/// if you want to send several kinds of messages, you can e.g.
/// switch based on msg[2] for message type
void customMsgParser( uint8_t * msg, uint8_t size, uint16_t source ){  
  if ( msg[2] == 0 ){ // 0, 92
    myservo.write( msg[3] );   // sets the servo position
  } else { // 1, 127, 63
    myservo.writeMicroseconds( msg[3]*256 + msg[4] );   // sets the servo position
  }
}

// osc message from software: /minibee/custom minibeeid highbyte lowbyte

void setup() {
  myservo.attach( SERVOPIN );  // attaches the servo on pin 9 to the servo object 

  Bee.setup(57600, 'D' ); // arguments are the baudrate, and the board revision
  
  /// define which pins we will be using for our custom functionality:
  /// arguments are: pin number, size of data they will produce (in bytes)
  /// in our case, pins 9 and 10, and we don't output any data from them
  Bee.setCustomPin( SERVOPIN, 0 );

  // set the custom message function
  Bee.setCustomCall( &customMsgParser );
}

void loop() {  
  // do a loop step of the remaining firmware:
  Bee.loopStep();
}
