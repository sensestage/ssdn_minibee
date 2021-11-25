/**
 * Copyright (c) 2018 Marije Baalman. All rights reserved
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

// this example provides an interface to the HIH8120 humidity / temperature sensor from Honeywell


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

uint8_t count = 0;

#define HIH_ADDRESS 0x27

uint16_t humidity;
uint16_t temperature;
uint8_t error;

int requestMeasurement(){ // if 0 then ready
    Wire.beginTransmission( HIH_ADDRESS );
    return Wire.endTransmission();
}

void readSensor(){
    uint8_t tempByte1;
    uint8_t tempByte2;
    uint8_t status;
    
    Wire.requestFrom( HIH_ADDRESS, (uint8_t) 4);
    if(Wire.available()) {
        tempByte1 = Wire.read();
        tempByte2 = Wire.read();
        
        status = tempByte1 >> 6;        
        switch( status ){
            case 0:
                humidity = (((uint16_t) (tempByte1 & 0x3f)) << 8) | tempByte2;
                
                tempByte1 = Wire.read();
                tempByte2 = Wire.read();                
                temperature = ((((uint16_t) tempByte1) << 8) | tempByte2) >> 2;
                
                error = 0; // all good
                break;            
            case 1:
                error = 1; // not ready to read yet
                break;            
            case 2:
                error = 2; // device is in command mode
                break;            
        }
        Wire.endTransmission();
    }
}


void setup(){    
   Bee.setup(57600, 'D' ); // arguments are the baudrate, and the board revision


   Bee.setCustomInput( 2, 2 ); // 2 inputs with 2 byte data
//    Bee.setCustomInput( 1, 1 ); // 1 inputs with 1 byte data
  
   Wire.begin();
    
   delay(5); // just give it some time to start up
}


void loop(){
    
    if ( requestMeasurement() == 0 ){
        count++;
        readSensor();
//         if( error == 1 ){
//             Bee.sendPrivateData( &error, 1, true );
//         }            
        while( error == 1 ){
            count++;            
            delay( 10 );
            readSensor();
            if ( count > 10 ){
                error = 4; // timeout on error 1
            }
        }
        if ( error == 0 ){
            Bee.addCustomData( &humidity, 1 ); // add the result to our data
            Bee.addCustomData( &temperature, 1 ); // add the result to our data
            Bee.loopStep(false); // we take care of delays ourselves    
            count = 0;
        } else {
            Bee.sendPrivateData( &error, 1, true );
        }
    } else {
        error = 3;
        Bee.sendPrivateData( &error, 1, true );
    }
    if ( count > 10 ){ // do a loopstep every now and then - even if there are errors
        Bee.addCustomData( &humidity, 1 ); // last measured data
        Bee.addCustomData( &temperature, 1 ); // last measured data
        Bee.loopStep(false); // we take care of delays ourselves            
        count = 0;
    }
    delay( 10 );
}

