/*
 *  ePIR.h created for ePIR library project on 09/01/2010 18:00:00.
 *  Written using AVR Project IDE v1.97.
*/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *      
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *      
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301, USA.
*/

/*
 * * * * * * * * * * * * * * * * * * * *
 * *  Code written by Corey Johnson  * *
 * *  September 1, 2010              * *
 * * * * * * * * * * * * * * * * * * * *
*/

/*
 *  This is a library for the 'Zilog ePIR Motion Detection -Zdots SBC(Single Board Computer)'. This library was 
 *  written for the Arduino Mega. I'm sure with a little work it can be ported to other boards. However, I have
 *  no current plans to do so. The example program shows some various uses of the library and is well commented.
 *  For syntax and function list, see the .txt file that came with the library. Also see zilog's documentation.
 *  This is the first working library I've ever written so if you have any comments or suggestions send an email
 *  to trlrtrsh2@cox.net. All I ask is that if you make any improvements to this library please give credit to 
 *  the original author(Me), give the library a new version/revision number, and please send me a copy so that I
 *  may learn from it. I am not an expert at programming, but I'm always eager to learn new things.
 *  I hope you find this library useful. Enjoy!  ;o)
*/

// ***************************************** Written for Arduino Mega *****************************************

////////////////////////////////////////////// Hardware connections //////////////////////////////////////////////
//               ------------------------------------		                                                        //
//               Zilog ePIR <----------> Arduino Mega                                                           //
//               ------------------------------------                                                           //
//                    GND pin 1 <--> GND                                                                        //
//                    VDD pin 2 <--> 3.3V                                                                       //
//                RXD/DLY pin 3 <--> TXD (This example uses TXD2-pin 16)                                        //
//                TXD/SNS pin 4 <--> RXD (This example uses RXD2-pin 17)                                        //
//                 MD/RST pin 5 <--> Any unused pin (This example uses pin 3)                                   //
//                     LG pin 6 <--> 3.3V (See zilog hardware documentation for use of this pin)                //
//                SLP/DBG pin 7 <--> Any unused pin (This example uses pin 4)                                   //
//                    GND pin 8 <--> GND                                                                        //
//                                                                                                              //
//             (Pins 1 & 8 are internally connected on the ePIR and only one needs to be connected)             //                                                                                                              //
//            **Also**   a 100K resistor needs to be connected from TXD/SNS pin of the ePIR to 3.3V             //
//                (without the resistor the device will not communicate with the Arduino board)                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////// IMPORTANT, PLEASE READ THE FOLLOWING! ////////////

/* For all functions requiring char type arguments, if no value is passed to the function's
 * argument, e.g. EPIR.function(), then the function reads & returns the current value in the
 * device. If a value is passed, the function writes that value to the device then reads and 
 * returns the stored value(should be the new value). This makes it easy to write and check 
 * the new value in one function call. e.g. checkValue = EPIR.function(newValue). If an invalid 
 * value is passed then the *default value will be written to the device and the stored value is 
 * read and returned.
 *
 * For all functions requiring byte type arguments; to read the current value set in the device use
 * any number from 257 to 65535 or >256 (yes, these are word values) and the function returns the current
 * value(byte). Use a value of 256 (also a word value) to write the *default value to the device. The
 * function returns with the stored value as above. Any value from 0 to 255 writes that value and returns 
 * the stored value. THERE IS ONE EXCEPTION, the .PulseCount function. For this function only, using a value
 * of 0 (zero), the function returns the current value. Valid values for .PulseCount are 0,1,2. Any
 * value from 3 to 255 will write the default value and return the stored value. 
 * Note: the .PulseCount function only accepts byte values (0-255), a word value will cause an error!
 *
 *                    (*default values found in zilogs product specs) 
*/

/* Driving the SLP pin LOW puts the ePIR in sleep mode. To wake the device either drive the pin HIGH
 * or send any single character to the ePIR on the serial port assigned by the .Init function 
 * (the character will be ignored/discarded and the device will wake from sleep).
*/

// **********************************************************************************
// ***   After uploading the sketch, open the serial monitor and set buad rate    ***
// *** to 115200 to view the values set by the sketch and when motion is detected ***
// **********************************************************************************

#include <SoftEPir.h> // Include the SoftePIR library.
#include <NewSoftSerial.h> // include NewSoftSerial for serial communication

// byte serialPort = 2; // Use serial port 2 (TXD2=pin 16, RXD2=pin 17).
uint8_t txpin = 5;
uint8_t rxpin = 6;
byte mdrPin = 3; // Use Arduino pin 3 for motion detection input.
byte sleepPin = 4; // Use Arduino pin 4 for sleep mode output.

volatile boolean checkStatus = false; // Variable for detection status set to true by interrupt function .

SoftePIR softpir = SoftePIR( rxpin, txpin );
// SoftePIR softpir;

/* Interrupt function */
void motionDetect(void) {
  checkStatus = true; // Set status variable to true.
}

void setup() {
  pinMode( 2, OUTPUT );
  digitalWrite( 2, 0 );
  delay(500);
  Serial.begin(115200); // Initialize Serial port 0 for serial monitor use. 
	char outChar; // Declare char type output variable.
  byte outByte; // Declare byte type output variable.
  
// //   softpir.initSerial( rxpin, txpin );
  
  softpir.Init(mdrPin, sleepPin); // Initialize Arduino Serial port & pins (port 2 / MD pin 3 / SLP pin 4). 
  Serial.println("Zilog ePIR Motion Detection -"); // Prints message to serial monitor.
  Serial.println("Zdots SBC(Single Board Computer).\n");
  Serial.println("Device stablizing, please wait...");
  delay(5000); // Wait 5 seconds before communicating with ePIR.
	do { // Start of do-loop.
    outChar = softpir.Status(); // Sends status command to ePIR and assigns reply from ePIR to variable outChar. (READ ONLY function)
  } // End of do-loop.
  while (outChar == 'U'); // Repeat do-loop if not stablized. (ePIR replies with the character 'U' until the device becomes stable) 
	Serial.println("Device Ready!\n"); // Print ready message.
  delay(5000); // Wait 5 seconds. (I'm not sure why, but without this delay the device gives inconsistant results)
	word softwareVersion = softpir.Version(); // Returns ePIR S/W version numbers. (READ ONLY function)(this is the only function that returns word data. (highByte is application S/W ver. - lowByte is S/W engine ver.) 
	Serial.print("Application Software v1."); // Prints version messages.
  Serial.println(highByte(softwareVersion), DEC);
  Serial.print("ePIR Software Engine v1.");
  Serial.print(lowByte(softwareVersion), DEC);
  Serial.println("\n");
	delay(10); // Need delays between commands for proper operation.
  
	outByte = softpir.LightLevel(); // Function to get current Light Gate level from ePIR. (READ ONLY function)(Light Gate pin is unused in this example, see zilog documentation for use of this feature)
  Serial.print("Light Gate Level: "); // Prints message. 
  Serial.println(outByte, DEC); // Prints value.
	delay(10);
  
	outByte = softpir.GateThresh(256); //  Writes default value(100)/Read/Print Light Gate Threshold value.
  Serial.print("Light Gate Threshold: ");
  Serial.println(outByte, DEC);
	delay(10);
  
	outChar = softpir.MDRmode('M'); // Sets ePIR and Arduino for hardware motion detection, then read/print new value. 
  Serial.print("MD/R pin: ");
  if (outChar == 'R') { // If set for hardware reset mode.
    Serial.println("ePIR Reset input");
  }
  else { // If set for hardware motion detection mode.
    Serial.println("ePIR Motion Detect output");
  }
	delay(10);
  
	outByte = softpir.MDtime(3); // Set MD pin to stay active for 3 seconds. / Read/Print new value.
  Serial.print("MD Pin active time: ");
  Serial.println(outByte, DEC);
	delay(10);
  
	outChar = softpir.Extended('Y'); // Set Extended Range to 'Y'es / Read/Print new value.
  Serial.print("Extended Range: ");
	Serial.println(outChar);
	delay(10);
  
	outChar = softpir.Frequency('L'); // Set Frequency Response to both 'L'ow & High frequencies / Read/Print new values.
  Serial.print("Frequency Response: ");
	Serial.println(outChar);
	delay(10);
  
	outByte = softpir.PulseCount(1); // Set Pulse Count to 1 / Read/Print new value.
  Serial.print("Pulse Count: ");
  Serial.println(outByte, DEC);
	delay(10);
  
	outByte = softpir.Sensitivity(3); // Set Sensitivity to 3 / Read/Print new value.
  Serial.print("Sensitivity: ");
  Serial.println(outByte, DEC);
	delay(10);
  
	attachInterrupt(1, motionDetect, FALLING); // Setup interrupt on pin 3 to detect motion.
  
	softpir.Status(); // Read/Reset motion status of ePIR.
}

void loop() { // Main loop.
  if (checkStatus == true) { // Check if motion has been detected.
    Serial.println("Motion Detected!"); // Print message to serial monitor.
		softpir.Status(); // Read/Reset ePIR status.
    checkStatus = false; // Reset status variable to false.
  } // End of if-is-true.
} // End of main loop.