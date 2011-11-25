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

#include <Wire.h>
// 
#include <LIS302DL.h>
#include <ADXL345.h>
#include <TMP102.h>
#include <BMP085.h>
#include <HMC5843.h>

#include <XBee.h>
#include <MiniBee_APIn.h>

#include <NewSoftSerial.h>

NewSoftSerial mySerial(5, 3);

MiniBee_API mbee = MiniBee_API();

void setup() {
  mbee.setup( 57600, 'D' );

  mbee.setCustomPin( 5, 0 );
  mbee.setCustomPin( 3, 1 );

  
  // set the data rate for the NewSoftSerial port
  mySerial.begin(115200);
  mySerial.println("Hello, world?");

}

void loop() {
  uint8_t * outdata;
  int dataSize;
  char serdata;
  
  if (mySerial.available()) {
    serdata = (char) mySerial.read();
  } else {
    serdata = '0';
  }
  mbee.addCustomData( &serdata, 1 );
  mbee.loopStep();

  dataSize = mbee.dataSize();
  outdata = mbee.getOutData();
    
  for ( int i=0; i<dataSize; i++ ){
    mySerial.print( outdata[i], DEC );
    mySerial.print( "," );
  }
  mySerial.println( "" ); 
}
