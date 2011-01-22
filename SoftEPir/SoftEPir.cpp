/*
 *  SoftePIR.cpp created for SoftePIR library project on 09/01/2010 18:00:00.
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
 * *  Adaptations for use with NewSoftSerial by Marije Baalman  * *
 * *  January, 2011              * *
 * * * * * * * * * * * * * * * * * * * *
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
 *  For hardware configuration see the .txt file that came with the library or go to the Zilog website for docu-
 *  mentation about the hardware. This is the first working library I've ever written so if you have any
 *  comments or suggestions send an email to trlrtrsh2@cox.net. All I ask is that if you make any improvements
 *  to this library please give credit to the original author(Me), give the library a new version/revision number,
 *  and please send me a copy so that I may learn from it. I am not an expert at programming, but I'm always eager
 *  to learn new things. I hope you find this library useful. Enjoy!  ;o)
*/
 
#include "SoftEPir.h"

/////////////////////////// * Constants * /////////////////////////////
const char ACK = char(6); // ..... "Acknowledge"
const char NACK = char(21); // ... "Non-Acknowledge"

///////////////////////// * Global variables * ////////////////////////
static byte MDR_Pin; // ....... Motion Detect/Reset. (Arduino pin) 
static byte SLP_Pin; // ....... Sleep. (Arduino pin)
// static byte Serial_Port; // ... Serial Port used to communicate with ePIR. (Serial1(default), Serial2, Serial3)

SoftePIR::SoftePIR(){/* nothing to construct */}
SoftePIR::SoftePIR( uint8_t rxpin, uint8_t txpin ){
   initSerial( rxpin, txpin );  
//   NewSoftSerial serial( rxpin, txpin );
}

SoftePIR::~SoftePIR(){/* nothing to destruct */}

void SoftePIR::initSerial(uint8_t rxpin, uint8_t txpin ){
//   NewSoftSerial serial( rxpin, txpin );
  serial = (NewSoftSerial*) malloc( sizeof( NewSoftSerial ) );
  serial->initPort( rxpin, txpin );
}

/* *********************** PRIVATE FUNCTIONS *********************** */
////////////////////////// * Read Function * //////////////////////////
char SoftePIR::readChar(char command){ // ......... Returns value selected by command sent to ePIR.
	char outChar = '\0'; // ..................... Assign NULL to output variable.
	do {
		serial->print(command);
		while(serial->available() == 0);
		outChar = serial->read();
	}
	while (outChar == NACK);
	return outChar; // .......................... Return from function with value from ePIR.
} // .......................................... End of read function.

//////////////////////// /* Write Function */ /////////////////////////
char SoftePIR::writeChar(char command, char inChar){ // ... Changes value of ePIR selected by command to value of inChar and returns with ACK. 
	char outChar = '\0'; // ............................. Assign NULL to output variable.
	do { // ............................................. Start of do-loop 'A'.
		do {
			serial->print(command);
			while(serial->available() == 0);
		}
		while (serial->read() == NACK);
		serial->print(inChar);
		while(serial->available() == 0);
		outChar = serial->read();
	} // ................................................ End of do-loop 'A'.
	while (outChar != ACK); // .......................... Repeats if reply from ePIR is not ACK.
	return outChar; // .................................. Return from function with value 'ACK'.
} // .................................................. End of Write function.

///////////////////// /* Confirmation Function */ /////////////////////
char SoftePIR::confirm(void){ // ................ Sends confirmation sequence '1234' to ePIR (Needed for .Sleep and .Reset functions).
	char outChar = '\0'; // ................... Assign NULL to output variable.
	serial->print('1');
	serial->print('2');
	serial->print('3');
	serial->print('4');
	while(serial->available() == 0);
	outChar = serial->read();
	return outChar; // ........................ Return from function with value 'ACK'.
}

/* *********************** PUBLIC  FUNCTIONS *********************** */
/////////////////////////// Initialize ePIR ///////////////////////////
void SoftePIR::Init(byte MDRpin, byte SLPpin){
	MDR_Pin = MDRpin;
	SLP_Pin = SLPpin;
	digitalWrite(MDR_Pin, HIGH);
	pinMode(MDR_Pin, OUTPUT);
	digitalWrite(SLP_Pin, HIGH);
	pinMode(SLP_Pin, OUTPUT);
	serial->begin(9600);
	return;
}

///////////////////////// Motion Detect Status ////////////////////////
char SoftePIR::Status(void){
	char status = readChar('a');
	return status;
}

////////////////////////// Light Gate Level ///////////////////////////
byte SoftePIR::LightLevel(void){
	byte level = byte(readChar('b'));
	return level;
}

//////////////////////// Light Gate Threshold /////////////////////////
byte SoftePIR::GateThresh(word threshold){
	if (257 > threshold){
		if (256 > threshold){
			writeChar('L', char(lowByte(threshold)));
		}
		else {
			byte defValue = 100;
			writeChar('L', char(defValue));
		}
	}
	byte thresholdOut = byte(readChar('l'));
	return thresholdOut;
}

//////////////////////////// MD/R Pin Mode ////////////////////////////
char SoftePIR::MDRmode(char mdrMode){
	if (mdrMode != '\0'){
		switch(mdrMode){
			case 'M':
				pinMode(MDR_Pin, INPUT);
				digitalWrite(MDR_Pin, HIGH);
				writeChar('C', 'M');
			break;
			default:
				pinMode(MDR_Pin, OUTPUT);
				digitalWrite(MDR_Pin, HIGH);
				writeChar('C', 'R');
			break;
		}
	}
	char mdrModeOut = readChar('c');
	return mdrModeOut;
}

///////////////////////// MD-Pin Active Time //////////////////////////
byte SoftePIR::MDtime(word mdTime){
	if (257 > mdTime){
		if (256 > mdTime){
			writeChar('D', char(lowByte(mdTime)));
		}
		else {
			byte defValue = 2;
			writeChar('D', char(defValue));
		}
	}
	byte mdTimeOut = byte(readChar('d'));
	return mdTimeOut;
}

///////////////////// MD-Pin Active Time Remaining ////////////////////
byte SoftePIR::TimeLeft(word timeLeft){
	if (257 > timeLeft){
		if (256 > timeLeft){
			writeChar('O', char(lowByte(timeLeft)));
		}
		else {
			byte defValue = 2;
			writeChar('O', char(defValue));
		}
	}
	byte timeLeftOut = byte(readChar('o'));
	return timeLeftOut;
}

/////////////////////////// Unsolicited Mode //////////////////////////
char SoftePIR::Unsolicited(char unsolicited){
	if (unsolicited != '\0'){
		switch (unsolicited){
			case 'Y':
				writeChar('M', 'Y');
			break;
			default:
				writeChar('M', 'N');
			break;
		}
	}
	char unsolicitedOut = readChar('m');
	return unsolicitedOut;
}

//////////////////////////// Extended Range ///////////////////////////
char SoftePIR::Extended(char extended){
	if (extended != '\0'){
		switch (extended){
			case 'Y':
				writeChar('E', 'Y');
			break;
			default:
				writeChar('E', 'N');
			break;
		}
	}
	char extendedOut = readChar('e');
	return extendedOut;
}

////////////////////////// Frequency Response /////////////////////////
char SoftePIR::Frequency(char frequency){
	if (frequency != '\0'){
		switch (frequency){
			case 'H':
				writeChar('F', 'H');
			break;
			default:
				writeChar('F', 'L');
			break;
		}
	}
	char frequencyOut = readChar('f');
	return frequencyOut;
}

////////////////////// Suspend MD-Pin Activation //////////////////////
char SoftePIR::Suspend(char suspend){
	if (suspend != '\0'){
		switch (suspend){
			case 'y':
				writeChar('H', 'Y');
			break;
			default:
				writeChar('H', 'N');
			break;
		}
	}
	char suspendOut = readChar('h');
	return suspendOut;
}

///////////////////////////// Pulse Count /////////////////////////////
byte SoftePIR::PulseCount(byte pulseCount){
	if (0 < pulseCount){
		byte pcValue;
		switch (pulseCount){
			case 2:
				pcValue = 2;
				writeChar('P', '2');
			break;
			default:
				pcValue = 1;
				writeChar('P', '1');
			break;
		}
	}
	byte pulseCountOut = (byte(readChar('p')) - 48);
	return pulseCountOut;
}

///////////////////////////// Sensitivity /////////////////////////////
byte SoftePIR::Sensitivity(word sensitivity){
	if (257 > sensitivity){
		if (256 > sensitivity){
			writeChar('S', char(lowByte(sensitivity)));
		}
		else {
			byte defValue = 6;
			writeChar('S', char(defValue));
		}
	}
	byte sensitivityOut = byte(readChar('s'));
	return sensitivityOut;
}

///////////////////////// Detection Direction /////////////////////////
char SoftePIR::Direction(char direction){
	if (direction != '\0'){
		switch (direction){
			case '+':
				writeChar('V', '+');
			break;
			case '-':
				writeChar('V', '-');
			break;
			default:
				writeChar('V', 'A');
			break;
		}
	}
	char directionOut = readChar('v');
	return directionOut;
}

///////////////////////////// Reset ePIR //////////////////////////////
void SoftePIR::Reset(void){
	serial->print('X');
	while(serial->available() == 0);
	serial->read();
	confirm();
	return;
}

/////////////////////////// ePIR Sleep Mode ///////////////////////////
void SoftePIR::Sleep(void){
	serial->print('Z');
	while(serial->available() == 0);
	serial->read();
	confirm();
	return;
}

/////////////////////////// ePIR S/W Version //////////////////////////
word SoftePIR::Version(void){
	byte appVer;
	byte engVer;
	word version;
	serial->print('i');
	while(serial->available() == 0);
	appVer = serial->read();
	engVer = serial->read();
	version = word(appVer, engVer);
	return version;
}

// SoftePIR EPIR = SoftePIR(); // ... Create one instance for user.