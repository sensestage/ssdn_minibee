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

// this example uses the SenseStage_FDC2214 library from https://github.com/ ...


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

#include <SenseStage_FDC2214.h>

SenseStage_FDC2214 fdc = SenseStage_FDC2214( 0 );

#define FDC_SHUTDOWN 8
#define FDC_INTERRUPT 7

MiniBee_API Bee = MiniBee_API();


void setup(){
    
   Bee.setup(57600, 'D' ); // arguments are the baudrate, and the board revision
  
  /// define which pins we will be using for our custom functionality:
  /// arguments are: pin number, size of data they will produce (in bytes)
  /// in our case, pins 7 and 8, and we don't output any data from them
  Bee.setCustomPin( FDC_SHUTDOWN, 0 );
  Bee.setCustomPin( FDC_INTERRUPT, 0 );
  
  Bee.setCustomInput( 2, 4 ); // 2 inputs with 4 byte data
  
    pinMode( FDC_SHUTDOWN, OUTPUT );
    digitalWrite( FDC_SHUTDOWN, 0 ); // put low; high is shutdown
    
    delay( 10 );
    
    pinMode( FDC_INTERRUPT, INPUT );
    
    fdc.begin();
       
    // we could choose to do something with the manufacturer id and device id
//     Serial.print( "Manufacturer: " );
//     Serial.println( fdc.getManufacturerID() );
//     Serial.print( "Device: " );
//     Serial.println( fdc.getDeviceID() );

    setupDevice();
    
    delay(5); // just give it some time to start up
}

void setupDevice(){
    fdc.write_register16bit( CONFIG, 0x3E01 ); // enable sleep mode
    
    // these are the default values in EVM gui
    fdc.write_register16bit( RCOUNT_CH0, 0xFFFF );
    fdc.write_register16bit( RCOUNT_CH1, 0xFFFF );
    fdc.write_register16bit( RCOUNT_CH2, 0xFFFF );
    fdc.write_register16bit( RCOUNT_CH3, 0xFFFF );
    
    fdc.write_register16bit( SETTLECOUNT_CH0, 0x0400 );
    fdc.write_register16bit( SETTLECOUNT_CH1, 0x0400 );
    fdc.write_register16bit( SETTLECOUNT_CH2, 0x0400 );
    fdc.write_register16bit( SETTLECOUNT_CH3, 0x0400 );
    
    fdc.write_register16bit( CLOCK_DIVIDERS_CH0, 0x1001 );
    fdc.write_register16bit( CLOCK_DIVIDERS_CH1, 0x1001 );
    fdc.write_register16bit( CLOCK_DIVIDERS_CH2, 0x1001 );
    fdc.write_register16bit( CLOCK_DIVIDERS_CH3, 0x1001 );
    
    fdc.write_register16bit( STATUS_CONFIG, 0x0001 ); // Data ready to interrupt pin
    
//     fdc.write_register16bit( MUX_CONFIG, 0x820C ); // ch0, 1
    fdc.write_register16bit( MUX_CONFIG, 0x820D ); // ch0, 1
//     fdc.write_register16bit( MUX_CONFIG, 0xc20C ); // all 
    
    fdc.write_register16bit( DRIVE_CURRENT_CH0, 0x8c40 );
    fdc.write_register16bit( DRIVE_CURRENT_CH1, 0x8c40 );
    
     fdc.write_register16bit( DRIVE_CURRENT_CH2, 0x8800 ); // when not used
     fdc.write_register16bit( DRIVE_CURRENT_CH3, 0x8800 ); // when not used

//     fdc.write_register16bit( DRIVE_CURRENT_CH2, 0x8c40 );
//     fdc.write_register16bit( DRIVE_CURRENT_CH3, 0x8c40 );    
     
    fdc.write_register16bit( CONFIG, 0x1E01 ); // enable interrupt pin -- exit sleep mode

}


uint8_t count = 0;
uint32_t result = 0;
uint8_t error = 0;

void loop(){
    
    count++;
    
    if ( digitalRead( FDC_INTERRUPT ) == 0 ){
        count = 0;
        for ( uint8_t i=0; i<2; i++ ){
            // read values:
            result = fdc.readChannel( i );
            Bee.addCustomData( &result, 1 ); // add the result to our data
            error = fdc.getReadError( i );
            if ( error > 0 ){
                // send error message
                Bee.sendPrivateData( &error, 1, true );
            }
        }
       // do a loop step of the remaining firmware:
       Bee.loopStep(false); // we take care of delays ourselves
    }
    
    if ( count > 100 ){ // also do a loopstep once in a while if interrupt is not triggered
        Bee.loopStep(false); // we take care of delays ourselves
    }
    
    delay(1);
}

