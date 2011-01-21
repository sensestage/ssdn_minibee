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
static byte Serial_Port; // ... Serial Port used to communicate with ePIR. (Serial1(default), Serial2, Serial3)

SoftePIR::SoftePIR(){/* nothing to construct */}
SoftePIR::~SoftePIR(){/* nothing to destruct */}

/* *********************** PRIVATE FUNCTIONS *********************** */
////////////////////////// * Read Function * //////////////////////////
char SoftePIR::readChar(char command){ // ......... Returns value selected by command sent to ePIR.
	char outChar = '\0'; // ..................... Assign NULL to output variable.
	switch(Serial_Port){ // ..................... Use correct serial port assigned by .Init function.
		case 2: // ................................ Use Serial port 2.
			do { // ................................. Start of do-loop.
				Serial2.print(command); // ............ Sends command to ePIR.
				while(Serial2.available() == 0); // ... Waits for reply from ePIR.
				outChar = Serial2.read(); // .......... Copy reply to output variable.
			} // .................................... End of do-loop.
			while (outChar == NACK); // ............. Repeat do-loop if NACK is received from ePIR.
		break; // ................................. End of case 2.
		case 3: // ................................ Same as case 2 but uses Serial port 3.
			do {
				Serial3.print(command);
				while(Serial3.available() == 0);
				outChar = Serial3.read();
			}
			while (outChar == NACK);
		break; // ................................. End of case 3.
//// If any valid value(byte) other than 2 or 3 is used, the default case will run (You should use 1 for value). ////
		default: // ............................... Same as case 2 but uses Serial port 1.
			do {
				Serial1.print(command);
				while(Serial1.available() == 0);
				outChar = Serial1.read();
			}
			while (outChar == NACK);
		break; // ................................. End of default case.
	} // ........................................ End of switch-case.
	return outChar; // .......................... Return from function with value from ePIR.
} // .......................................... End of read function.

//////////////////////// /* Write Function */ /////////////////////////
char SoftePIR::writeChar(char command, char inChar){ // ... Changes value of ePIR selected by command to value of inChar and returns with ACK. 
	char outChar = '\0'; // ............................. Assign NULL to output variable.
	do { // ............................................. Start of do-loop 'A'.
		switch(Serial_Port){ // ........................... Use correct serial port assigned by .Init function.
			case 2: // ...................................... Use Serial port 2.
				do { // ....................................... Start of do-loop 'B'.
					Serial2.print(command); // .................. Sends command to ePIR.
					while(Serial2.available() == 0); // ......... Waits for reply from ePIR.
				} // .......................................... End of do-loop 'B'.
				while (Serial2.read() == NACK); // ............ Repeat do-loop if NACK is received from ePIR.
				Serial2.print(inChar); // ..................... Send value 'inChar' to ePIR.
				while(Serial2.available() == 0); // ........... Wait for reply from ePIR.
				outChar = Serial2.read(); // .................. Assign received value from ePIR to 'outChar'.
			break; // ....................................... End of case 2.
			case 3: // ...................................... Same as case 2 but uses Serial port 3.
				do {
					Serial3.print(command);
					while(Serial3.available() == 0);
				}
				while (Serial3.read() == NACK);
				Serial3.print(inChar);
				while(Serial3.available() == 0);
				outChar = Serial3.read();
			break; // ....................................... End of case 3.
//// If any valid value(byte) other than 2 or 3 is used, the default case will run (You should use 1 for value). ////
			default: // ..................................... Same as case 2 but uses Serial port 1.
				do {
					Serial1.print(command);
					while(Serial1.available() == 0);
				}
				while (Serial1.read() == NACK);
				Serial1.print(inChar);
				while(Serial1.available() == 0);
				outChar = Serial1.read();
			break; // ....................................... End of default case.
		} // .............................................. End of switch-case.
	} // ................................................ End of do-loop 'A'.
	while (outChar != ACK); // .......................... Repeats if reply from ePIR is not ACK.
	return outChar; // .................................. Return from function with value 'ACK'.
} // .................................................. End of Write function.

///////////////////// /* Confirmation Function */ /////////////////////
char SoftePIR::confirm(void){ // ................ Sends confirmation sequence '1234' to ePIR (Needed for .Sleep and .Reset functions).
	char outChar = '\0'; // ................... Assign NULL to output variable.
	switch(Serial_Port){ // ................... Use correct serial port assigned by .Init function.
		case 2: // .............................. Use Serial port 2.
			Serial2.print('1'); // ................ Send char '1' to ePIR.
			Serial2.print('2'); // ................ Send char '2' to ePIR.
			Serial2.print('3'); // ................ Send char '3' to ePIR.
			Serial2.print('4'); // ................ Send char '4' to ePIR.
			while(Serial2.available() == 0); // ... Waits for reply from EPIR.
			outChar = Serial2.read(); // .......... Assign received value from ePIR to 'outChar'.
		break; // ............................... End of case 2.
		case 3: // .............................. Same as case 2 but uses Serial port 3.
			Serial3.print('1');
			Serial3.print('2');
			Serial3.print('3');
			Serial3.print('4');
			while(Serial3.available() == 0);
			outChar = Serial3.read();
		break; // ............................... End of case 3.
//// If any valid value(byte) other than 2 or 3 is used, the default case will run (You should use 1 for value). ////
		default: // ............................. Same as case 2 but uses Serial port 1.
			Serial1.print('1');
			Serial1.print('2');
			Serial1.print('3');
			Serial1.print('4');
			while(Serial1.available() == 0);
			outChar = Serial1.read();
		break; // ............................... End of case 3.
	} // ...................................... End of switch-case.
	return outChar; // ........................ Return from function with value 'ACK'.
}

/* *********************** PUBLIC  FUNCTIONS *********************** */
/////////////////////////// Initialize ePIR ///////////////////////////
void SoftePIR::Init(byte serialPort, byte MDRpin, byte SLPpin){
	MDR_Pin = MDRpin;
	SLP_Pin = SLPpin;
	Serial_Port = serialPort;
	digitalWrite(MDR_Pin, HIGH);
	pinMode(MDR_Pin, OUTPUT);
	digitalWrite(SLP_Pin, HIGH);
	pinMode(SLP_Pin, OUTPUT);
	switch(Serial_Port){
		case 2:
			Serial2.begin(9600);
		break;
		case 3:
			Serial3.begin(9600);
		break;
		default:
			Serial1.begin(9600);
		break;
	}
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
	switch(Serial_Port){
		case 2:
			Serial2.print('X');
			while(Serial2.available() == 0);
			Serial2.read();
			confirm();
		break;
		case 3:
			Serial3.print('X');
			while(Serial3.available() == 0);
			Serial3.read();
			confirm();
		break;
		default:
			Serial1.print('X');
			while(Serial1.available() == 0);
			Serial1.read();
			confirm();
		break;
	}
	return;
}

/////////////////////////// ePIR Sleep Mode ///////////////////////////
void SoftePIR::Sleep(void){
	switch(Serial_Port){
		case 2:
			Serial2.print('Z');
			while(Serial2.available() == 0);
			Serial2.read();
			confirm();
		break;
		case 3:
			Serial3.print('Z');
			while(Serial3.available() == 0);
			Serial3.read();
			confirm();
		break;
		default:
			Serial1.print('Z');
			while(Serial1.available() == 0);
			Serial1.read();
			confirm();
		break;
	}
	return;
}

/////////////////////////// ePIR S/W Version //////////////////////////
word SoftePIR::Version(void){
	byte appVer;
	byte engVer;
	word version;
	switch(Serial_Port){
		case 2:
			Serial2.print('i');
			while(Serial2.available() == 0);
			appVer = Serial2.read();
			engVer = Serial2.read();
		break;
		case 3:
			Serial3.print('i');
			while(Serial3.available() == 0);
			appVer = Serial3.read();
			engVer = Serial3.read();
		break;
		default:
			Serial1.print('i');
			while(Serial1.available() == 0);
			appVer = Serial1.read();
			engVer = Serial1.read();
		break;
	}
	version = word(appVer, engVer);
	return version;
}

// SoftePIR EPIR = SoftePIR(); // ... Create one instance for user.