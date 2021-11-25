/**
 * Copyright (c) 2017 Marije Baalman. All rights reserved
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

// this example uses the Adafruit DRV2605 library from https://github.com/adafruit/Adafruit_DRV2605_Library


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

// this example shows how to drive the TI DRV2605L haptics driver
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

// pin for enabling the haptics driver
#define DRV_ENABLE 6 

MiniBee_API Bee = MiniBee_API();

bool usingLRA = false;
uint8_t library = 1;

uint8_t privateData[9];

/// this will be our parser for the custom messages we will send:
/// msg[0] and msg[1] will be msg type ('E') and message ID
/// the remainder are the actual contents of the message
/// if you want to send several kinds of messages, you can e.g.
/// switch based on msg[2] for message type
void customMsgParser( uint8_t * msg, uint8_t size, uint16_t source ){
    uint8_t wv;
    uint8_t i;
    switch ( msg[2] ){
        case 'G': // go
            privateData[0] = 'G';
            drv.go();
            break;
        case 'S': // stop
            privateData[0] = 'S';
            drv.stop();
            break;        
        case 'L': // select library
            privateData[0] = 'L';
            setLibrary( msg[3] );
            break;
        case 'W': // set waveform sequence
            privateData[0] = 'W';
            for ( i=0; i<8; i++ ){
                wv = msg[3+i];
                drv.setWaveform( i, wv );
                privateData[i+1] = wv;
            }
            break;
        case 'M': // select motor
            privateData[0] = 'M';
            if ( msg[3] == 'L' ){
                usingLRA = true;
                drv.useLRA();
                setLibrary( library );
            } else {
                usingLRA = false;
                drv.useERM();
                setLibrary( library );
            }
    }
    // send confirmation
    Bee.sendPrivateData( privateData, 9, true ); // 
}

void setLibrary( uint8_t desiredLib ){
    if ( usingLRA ){
        privateData[1] = 'L';
        drv.selectLibrary( 6 );
        library = 6;
    } else {
        privateData[1] = 'E';
        if ( desiredLib < 6 && (desiredLib > 0 ) ){
            drv.selectLibrary( desiredLib );
            library = desiredLib;
        } else {
            library = 0;
        }
    }
    privateData[2] = library;
}

void setup() {
  Bee.setup(57600, 'D' ); // arguments are the baudrate, and the board revision
  
  pinMode( DRV_ENABLE, OUTPUT );
  digitalWrite( DRV_ENABLE, HIGH );
  
  // startup code for driver
  drv.begin();
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);

  /// define which pins we will be using for our custom functionality:
  /// arguments are: pin number, size of data they will produce (in bytes)
  /// in our case, pin 7, and we don't produce data from it
  Bee.setCustomPin( DRV_ENABLE, 0 );

  // set the custom message function
  Bee.setCustomCall( &customMsgParser );
  
  // initialize message back
  for ( uint8_t i=0; i<8; i++ ){
      privateData[i] = 0;
  }
}

void loop() {  
  // do a loop step of the remaining firmware:
  Bee.loopStep();
}
