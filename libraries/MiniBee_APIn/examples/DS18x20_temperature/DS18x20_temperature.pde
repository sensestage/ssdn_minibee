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

#include <OneWire.h>

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library


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

OneWire  ds(10);  // on pin 10

byte addr[8];
byte type_s;
byte data[12];

int tempRaw;

void setup() {
  Bee.setup(57600,'D');

  /// in our case we use pin 10 (2 bytes/int)
  Bee.setCustomPin( 10, 2);
}

bool checkForDevice(){
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    return false;
  }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
//       Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
//       Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
//       Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      return false;
  } 

  ds.reset();

  return true;
}

void readFromDevice(){
  byte present = 0;
  byte i;
  byte crc;

  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  // a delay here is not very nice, better to make a counter in the loop, and make a state machine
  delay(750);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
//     Serial.print(data[i], HEX);
//     Serial.print(" ");
  }
//   Serial.print(" CRC=");
//   Serial.print(, HEX);
  crc = OneWire::crc8(data, 8);
//   Serial.println();

  tempRaw = (data[1] << 8) | data[0];
  if (type_s) {
    tempRaw = tempRaw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      tempRaw = (tempRaw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) tempRaw = tempRaw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) tempRaw = tempRaw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) tempRaw = tempRaw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
//   celsius = (float)raw / 16.0;

}

void loop() {

  checkForDevice();
  readFromDevice();
    
  // add our customly measured data to the data package:
  Bee.addCustomData( &tempRaw, 1 );
  // do a loop step of the remaining firmware:
  Bee.loopStep();
}
