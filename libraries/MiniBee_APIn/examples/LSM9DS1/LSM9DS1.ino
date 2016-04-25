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
// #include <LIS302DL.h>
#include <ADXL345.h>
// #include <TMP102.h>
// #include <BMP085.h>
// #include <HMC5843.h>

/// includes needed in all cases
#include <XBee.h>
#include <MiniBee_APIn.h>

// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
// #include <Wire.h>
// #include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu = LSM9DS1();

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW



MiniBee_API Bee = MiniBee_API();


void setup() {
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  
  imu.settings.gyro.sampleRate = 3;
  imu.settings.accel.sampleRate = 3;
  imu.settings.gyro.scale = 2000; // set gyro scale to 2000
  imu.settings.accel.scale = 16; // Set accel scale to +/-16g.
  
  uint16_t status = imu.begin();
  // Or call it with declarations for sensor scales and data rates:  
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
  //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);
  
//   // begin() returns a 16-bit value which includes both the gyro 
//   // and accelerometers WHO_AM_I response. You can check this to
//   // make sure communication was successful.
//   Serial.print("LSM9DS1 WHO_AM_I's returned: 0x");
//   Serial.println(status, HEX);
//   Serial.println("Should be 0x683D");
//   Serial.println();  
//   if (!status)
//   {
//     Serial.println("Failed to communicate with LSM9DS1.");
// //     Serial.println("Double-check wiring.");
// //     Serial.println("Default settings in this sketch will " \
// //                   "work for an out of the box LSM9DS1 " \
// //                   "Breakout, but may need to be modified " \
// //                   "if the board jumpers are.");
//     while (1)
//       ;
//   }

  
  Bee.setup(57600,'D');
  
  // if you generate data without a pin associated, set the pin number to 0.
  // Bee.setCustomPin( 0, 2 );
  // if you generate data without a pin associated, use setCustomInput
  Bee.setCustomInput( 10, 2 ); //uint8_t noInputs, uint8_t size );
}

int16_t data[10];

void loop() {
  
  if ( imu.gyroAvailable() ){
    imu.readGyro();
    data[0] = imu.gx;
    data[1] = imu.gy;
    data[2] = imu.gz;
  }

  if ( imu.accelAvailable() ){
    imu.readAccel();
    data[3] = imu.ax;
    data[4] = imu.ay;
    data[5] = imu.az;
  }

  if ( imu.magAvailable() ){
    imu.readMag();
    data[6] = imu.mx;
    data[7] = imu.my;
    data[8] = imu.mz;
  }

  if ( imu.tempAvailable() ){
    imu.readTemp();
    data[9] = imu.temperature;
  }
  
  // add our customly measured data to the data package:
  Bee.addCustomData( data, 10 );
  // do a loop step of the remaining firmware:
  Bee.loopStep();
}
