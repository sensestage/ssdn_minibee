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

MiniBee_API Bee = MiniBee_API();

/// configure the XBees with different addresses:
/// e.g. 13 and 14
/// ATMY

uint16_t otherXBee = 0x0D;

/// this will be our parser for the data messages we will receive from other nodes:
/// msg[0] and msg[1] will be msg type ('d') and message ID
/// the remainder are the actual contents of the message

int lastMsgID = 255;

int otherDigitalValue = 0;
int otherAnalogValues[3] = {0,0,0};
int otherAcceleroValues[3] = {2048,2048,2048};

int myDigitalValue = 0;
int myAnalogValues[3];
int myAcceleroValues[3];

/// both nodes will measure 3 analog values, and 3 accelerometer values, and 1 digital value
/// and send the data to the other node
/// msg[0] is 'd', msg[1] is the message id, the rest of the package is data

void dataMsgParser( uint8_t * msg, uint8_t size, uint16_t source ){
  if ( source == otherXBee ){ // check if this is coming from our remote MiniBee
      if ( lastMsgID != msg[1] ){ // this is a new message
	lastMsgID = msg[1];
	// data will come in as:
	// 1 byte with bits of digital data (depending on what the other node sends)
	// n x 2 bytes of analog data (assuming n channels of 10bit analog data)
	// 3 x 2 bytes of accelerometer data
	// 0b00110001
	// example, we want to know the second (1) digital value that was sent:
	otherDigitalValue = bitRead( msg[2], 1 );
	// no more than 8 digital values were sent, so the rest of the package is analog data (and accelero data).
	uint8_t cnt = 3;
	for ( uint8_t i=0; i<3; i++ ){
	    otherAnalogValues[i] = msg[ cnt ] * 256 + msg[ cnt+1 ]; // convert the two bytes of data to an int
	    cnt += 2; // add two to the count
	}
	for ( uint8_t i=0; i<3; i++ ){
	    otherAcceleroValues[i] = msg[ cnt ] * 256 + msg[ cnt+1 ]; // convert the two bytes of data to an int
	    cnt += 2; // add two to the count
	}
      }
  }
}

/// we use a fixed configuration for the MiniBee, rather than use a remote wireless configuration
/// Here we define:
/// pins D3, D5, D6, D9, D10, D11 as analog outputs
/// pin D7 as digital input
/// pin D8 as digital output
/// pins A0, A1, A2 as analog inputs
/// and we use the onboard ADXL accelerometer via the TwoWire interface
uint8_t myConfig[] = { 0, 1, 0, 50, 1, // null, config id, msgInt high byte, msgInt low byte, samples per message
  AnalogOut, NotUsed, AnalogOut, AnalogOut, DigitalIn, DigitalOut, // D3 to D8 (D4 is reserved for status LED)
  AnalogOut, AnalogOut, AnalogOut, NotUsed, NotUsed,  // D9,D10,D11,D12,D13 (D12, D13 are also reserved)
  AnalogIn10bit, AnalogIn10bit, AnalogIn10bit, NotUsed, TWIData, TWIClock, NotUsed, NotUsed, // A0, A1, A2, A3, A4, A5, A6, A7
  1, TWI_ADXL345 // 1 I2C/TWI device: the ADXL
};

void setup() {
  /// no remote config, we're just listening in
  Bee.setRemoteConfig( 0 );
  
  /// arguments are the baudrate, and the board revision
  Bee.setup(57600, 'D' );   
    
  /// read the configuration message, this will automatically configure all the pins
  Bee.readConfigMsg( myConfig, 26 );
  
  /// set the destination address where our sensed data will be sent to
  Bee.setDestination( otherXBee );
  
  /// set the data message function
  Bee.setDataCall( &dataMsgParser );
}

int datasize;
uint8_t * data;

uint8_t outData[7] = { 0, 0, 0, 0, 0, 0, 0};

void loop() {  
  /// do a loop step of the minibee firmware:
  /// this will do all the sensing according to the configuration we made
  Bee.loopStep( false ); // don't do the delay in the minibee firmware
  
  /// get a reference to our own outputData or sensed data, and get the size:
  datasize = Bee.dataSize();
  data = Bee.getOutData();
  
  // the outdata has the format:
  // 1 or 2 bytes with digital data
  // n bytes of analog data, 2 bytes per channel if Analog10bit
  // n bytes of twi data, for accelero 2 bytes per channel
  
  myDigitalValue = bitRead( data[0], 1 );
  /// no more than 8 digital values were sent, so the rest of the package is analog data (and accelero data).
  uint8_t cnt = 1;
  for ( uint8_t i=0; i<3; i++ ){
    myAnalogValues[i] = data[ cnt ] * 256 + data[ cnt+1 ]; // convert the two bytes of data to an int
    cnt += 2; // add two to the count
  }
  for ( uint8_t i=0; i<3; i++ ){
    myAcceleroValues[i] = data[ cnt ] * 256 + data[ cnt+1 ]; // convert the two bytes of data to an int
    cnt += 2; // add two to the count
  }
  
  /// map my analog inputs to analog outputs on D3, D5, D6
  for ( uint8_t i=0; i<3; i++ ){
    outData[i] = map( myAnalogValues[i], 0, 1023, 0, 255 );
  }
  /// map other analog inputs to analog outputs D9, D10, D11
  for ( uint8_t i=0; i<3; i++ ){
    outData[i+3] = map( otherAnalogValues[i], 0, 1023, 0, 255 );
  }
  /// map digital in from ourselves or from remote to digital out
  outData[6] = myDigitalValue | otherDigitalValue; // we push the button here or there
  
  /// set the output values via the minibee firmware
  /// outData should have the format: N (=3 in our case) x analogOutputs, M(=1 in our case) x digitalValues
  Bee.setOutputValues( outData, 0 );
  /// actual write it to the pins
  Bee.setOutput();
  
  /// do the delay here manually
  delay( 50 );
}
