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

uint8_t triggerData[2];

uint8_t count;

int threshold;
bool sentTrigger;

void setup() {
  Bee.setup(57600,'D');
  count = 0;
  threshold = 100;
  
  // our trigger data
  triggerData[0] = 0;
  triggerData[1] = 0;
  sentTrigger = false;
}


void loop() {
  count++;
  // do a loop step of the remaining firmware:
  Bee.loopMeasureOnly();
  
  // access the measured data
  // the order of data is:
  // digital inputs -> packed in bytes; order: D3, D5, D6, D7, D8, D9, D10, D11 , A0, A1, A2, A3, A4, A5 (whichever ones are configured as digital ins)
  // analog values -> packed in one (8bit) or two bytes (10bit)
  // then the TWI device, e.g. ADXL
  // then the SHT devices
  // then the Ping data

  // assume we have the minibee configured as:
  // D3 = digitalIn
  // A0 = analog10Bit
  int dataSize = Bee.dataSize(); // we can query the size of the output data
  uint8_t * outputData = Bee.getOutData();
  if ( dataSize > 2 ){
    int analogValue = outputData[1]*256 + outputData[0]; // make an int of it
    if ( analogValue > threshold ){
      triggerData[0] = 1;
      triggerData[1] = analogValue - threshold;
      sentTrigger = Bee.sendTriggerData( triggerData, 2, true ); // arguments are data to send, data size in bytes, and whether or not to check the transmission status
    } else if ( sentTrigger ) {
      triggerData[0] = 0;
      Bee.sendTriggerData( triggerData, 2, true ); // arguments are data to send, data size in bytes, and whether or not to check the transmission status
      sentTrigger = false;
    }
  }
    
  if ( count > 5 ){
    count = 0;
    // read and send the rest of the data
    Bee.loopReadOnly();
    Bee.loopSendOnly( false ); // do not use delay  
  }
}
