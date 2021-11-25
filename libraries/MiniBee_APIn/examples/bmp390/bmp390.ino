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

// comment if not debugging. The debugging prints data to pin 3 and 5 as serial data to monitor
#define DEBUGSERIAL

/// Wire needs to be included if TWI is enabled
#include <Wire.h>
/// in the header file of the MiniBee you can disable some options to save
/// space on the MiniBee. If you don't the board may not work as it runs
/// out of RAM.
// #include <LIS302DL.h>
#include <ADXL345.h>
// #include <TMP102.h>
// #include <HMC5843.h>
// #include <BMP085.h>

/// includes needed in all cases
#include <XBee.h>
#include <MiniBee_APIn.h>

// #include <digitalWriteFast.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#ifdef DEBUGSERIAL
#include <Int64String.h>
// for testing
#include <NewSoftSerial.h>
NewSoftSerial mySerial(5, 3);
#endif

Adafruit_BMP3XX bmp;
MiniBee_API Bee = MiniBee_API();

bool bmpInitiated;
float sealevel_hpa = 1006.7;
uint16_t sealevel_int;
uint16_t sealevel_int_fraction;
// int32_t sealevel_int;

// sets the sealevel hpa:
// two bytes are the value before the .
// two bytes the value after the dot
// so 1006.7 is sent as 1006 and 7000: 3, 238, 27, 88
void customMsgParser( uint8_t * msg, uint8_t size, uint16_t source ){
//    sealevel_int = msg[2]<<24 | msg[3]<<16 | msg[4]<<8 | msg[5];
   sealevel_int = (msg[2]<<8) + msg[3];
   sealevel_int_fraction = (msg[4]<<8) + msg[5];
   sealevel_hpa = ((float) sealevel_int_fraction) / 10000.0;
   sealevel_hpa += ((float) sealevel_int);

#ifdef DEBUGSERIAL   
   mySerial.print( "New sealevel reference: ");
   mySerial.print( sealevel_hpa, 4 );
   mySerial.print( ", " );
   mySerial.print( sealevel_int );
   mySerial.print( ", " );
   mySerial.println( sealevel_int_fraction );
#endif
}

void setup() {
  Bee.setup(57600,'D'); // set to D here!

  setupBMP();

#ifdef DEBUGSERIAL
  // new soft serial - for debugging
  Bee.setCustomPin( 5, 0 );
  Bee.setCustomPin( 3, 0 );
  // set the data rate for the NewSoftSerial port
  mySerial.begin(57600);
  mySerial.println("Hello, BMP390!");
#endif
  
  // set the custom message function
  Bee.setCustomCall( &customMsgParser );  // to set the sealevel hPa
  // to read the custom inputs
  Bee.setCustomInput( 1, 2 ); // temperature
  Bee.setCustomInput( 1, 2 ); // pressure (hPa, before .)
  Bee.setCustomInput( 1, 2 ); // pressure (hPa, after .)
  Bee.setCustomInput( 1, 1 ); // sign sealevel
  Bee.setCustomInput( 1, 2 ); // sealevel
  Bee.setCustomInput( 1, 1 ); // newdata flag
 
}

void setupBMP(){
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    bmpInitiated = false;
  } else {
    bmpInitiated = true;   
  }
  

  if ( bmpInitiated ){
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
#ifdef DEBUGSERIAL  
  mySerial.println("BMP initiated");
#endif
}

// uint32_t temperature;
// uint32_t pressure;
// uint32_t altitude;
uint16_t temperature;
// long pressure;
uint16_t pres_hPa;
uint16_t pres_hPaFraction;
uint16_t altitude;
uint8_t sign;
uint8_t newdata;

double pressure_double;
float altitude_float;

void loop() {

  newdata = 0;
  if ( bmpInitiated ){
    if (bmp.performReading()) { 
        altitude_float = bmp.calculateAltitude(sealevel_hpa);
        
#ifdef DEBUGSERIAL
        mySerial.print( "reading float: ");
        mySerial.print( bmp.temperature, 4 );
        mySerial.print( ", " );
        mySerial.print( bmp.pressure, 4 );
        mySerial.print( ", " );
        mySerial.println( bmp.calculateAltitude(sealevel_hpa) );
#endif
        
        temperature = (uint16_t) (bmp.temperature * 1000);
        pressure_double = bmp.pressure / 100.0; // pressure in hPascal
        pres_hPa = (uint16_t) floor(pressure_double);
        pres_hPaFraction = (uint16_t) ( (pressure_double - floor(pressure_double))*10000 );
//         pressure = (long) (bmp.pressure * 1000); // this will not go negative
        
        if ( altitude_float < 0 ){ 
            sign = 0;             
        } else { 
            sign = 1;            
        }
        altitude = (uint16_t) (fabs(altitude_float) * 1000); // this function I added to Adafruit_BMP3XX to prevent an additional reading from the sensor            
        newdata = 1;    
#ifdef DEBUGSERIAL
        mySerial.print( newdata );
        mySerial.print( ", converted integer: ");
        mySerial.print( temperature );
        mySerial.print( ", " );
        mySerial.print( pres_hPa );
        mySerial.print( ", " );
        mySerial.print( pres_hPaFraction );
        mySerial.print( ", " );
        mySerial.print( sign );
        mySerial.print( ", " );
        mySerial.println( altitude );
#endif
    }
  }
  Bee.addCustomData( &temperature, 1 );       // temperature * 1000
//   Bee.addCustomData( &pressure, 1 );
  Bee.addCustomData( &pres_hPa, 1 );          // pressure value in hPa 
  Bee.addCustomData( &pres_hPaFraction, 1 );  // pressure fractional value in hPa
  Bee.addCustomData( &sign, 1 );
  Bee.addCustomData( &altitude, 1 );          // altitue in mm
  Bee.addCustomData( &newdata, 1 );  
  
  // do a loop step of the remaining firmware:
  Bee.loopStep();  
}
