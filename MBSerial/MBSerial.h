/**
 * Copyright (c) 2011 Marije Baalman. All rights reserved
 *
 * This file is part of the MBSerial library.
 *
 * MBSerial is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MBSerial is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MBSerial.  If not, see <http://www.gnu.org/licenses/>.
 */

// #include <ADXL345.h>

#ifndef MBSerial_h
#define MBSerial_h

#define MINIBEE_REVISION 'Z'
#define MINIBEE_LIBVERSION 3

/// all together: 3644 bytes
#define MINIBEE_ENABLE_TWI 1  /// TWI/I2C takes up 2064 bytes
#define MINIBEE_ENABLE_SHT 1  /// SHT takes up 1140 bytes
#define MINIBEE_ENABLE_PING 1 /// Ping takes up 440 bytes

/// all in: 19138 bytes
// specific TWI devices
#define MINIBEE_ENABLE_TWI_ADXL 1 /// 962 bytes - without: 18176
#define MINIBEE_ENABLE_TWI_LISDL 1 /// 614 bytes - without: 18524
#define MINIBEE_ENABLE_TWI_HMC 1 /// 1644 bytes - without: 17494 
#define MINIBEE_ENABLE_TWI_BMP 1 /// 4182 bytes - without: 14956
#define MINIBEE_ENABLE_TWI_TMP 1 /// 250 bytes - without: 18888

// #include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <WProgram.h>
// #include <EEPROM.h>
// #include <Wire.h>

#ifndef MINIBEE_ENABLE_TWI
#define MINIBEE_ENABLE_TWI 1
#endif

#ifndef MINIBEE_ENABLE_SHT
#define MINIBEE_ENABLE_SHT 1
#endif

#ifndef MINIBEE_ENABLE_PING
#define MINIBEE_ENABLE_PING 1
#endif

#if MINIBEE_REVISION == 'Z'
#define NRPINS 20
#endif
#if MINIBEE_REVISION == 'D'
#define NRPINS 17
#endif
#if MINIBEE_REVISION == 'B'
#define NRPINS 17
#endif
#if MINIBEE_REVISION == 'A'
#define NRPINS 19
#endif

#if MINIBEE_ENABLE_TWI == 1
#include <Wire.h>

#if MINIBEE_ENABLE_TWI_ADXL == 1
#include <ADXL345.h>
#endif

#if MINIBEE_ENABLE_TWI_LISDL == 1
#include <LIS302DL.h>
#endif

#if MINIBEE_ENABLE_TWI_TMP == 1
#include <TMP102.h>
#endif

#if MINIBEE_ENABLE_TWI_BMP == 1
#include <BMP085.h>
#endif

#if MINIBEE_ENABLE_TWI_HMC == 1
#include <HMC5843.h>
#endif

#endif



enum MiniBeePinConfig { 
  NotUsed,
  DigitalIn, DigitalOut,
  AnalogIn, AnalogOut, AnalogIn10bit, 
  SHTClock, SHTData, 
  TWIClock, TWIData,
  Ping,
  Custom = 100,
  MeID = 150,
  UnConfigured = 200,
};

enum TWIDeviceConfig { 
  TWI_ADXL345=10,
  TWI_LIS302DL=11,
  TWI_BMP085=20,
  TWI_TMP102=30,
  TWI_HMC58X3=40
};

// extern "C" {
//  	void PCINT0_vect(void) __attribute__ ((signal));
//  	void USART_RX_vect(void) __attribute__ ((signal));
// }

class MBSerial {
	public:
		MBSerial();	//constructor

		void (*customMsgFunc)(char *);// = NULL;
		void (*dataMsgFunc)(char *);// = NULL;
		
// 		void ParseCustom( char * msg ){ Serial.println( msg ); };
		
		void begin(long); //init function
		void doLoopStep(void); // loop function
		
		void setRemoteConfig( bool onoff );

		void setCustomPins( uint8_t * ids, uint8_t * sizes, uint8_t n ); // sets pins to custom configuration
		void setCustomPin( uint8_t id, uint8_t size ); // sets a pin to custom configuration
		void setCustomInput( uint8_t noInputs, uint8_t size );
		void addCustomData( uint8_t * cdata, uint8_t n );
		void addCustomData( char * cdata, uint8_t n );
		void addCustomData( int * cdata, uint8_t n );
		void setCustomCall( void (*customFunc)(char * ) );

		void setDataCall( void (*dataFunc)(char * ) );
		
		void openSerial(long);

		void setID( uint8_t id );
		char * getData();
		int dataSize();


	// serial communication with network
		void send(char, char *, int);
		void read(void);

	// set output pins
		void setRunning( uint8_t ); 
		void setLoopback( uint8_t ); 
		void setOutput();
		void setOutputValues( char * msg, uint8_t offset );
	
	// read input pins
		uint8_t readSensors( uint8_t );
	
	// send data
		void sendData( void );

		uint8_t getId(void);
		void sendSerialNumber(void);
// 		void waitForConfig(void); // waits for the configuration message
// 		void configure(void);	//configure from eeprom settings

		void readConfigMsg(char *, uint8_t); // assign config from msg

	//twi
#if MINIBEE_ENABLE_TWI == 1
		bool getFlagTWI();	//returns twi flag state
		int readTWIdevices(int dboff);
		void setupTWIdevices(void);

// #if MINIBEE_REVISION == 'A'
// 		void setupTWI(void);	//setup function for TWI
// 		int readTWI(int, int);	//address, number of bytes;
// 		int readTWI(int, int, int);	//address, register, number of bytes
// #endif
// 		void setupAccelleroTWI();
// 		void readAccelleroTWI( int dboff );
#endif

#if MINIBEE_ENABLE_SHT == 1
	//sht
		int ioSHT;
		int ackSHT;
		int valSHT; 
		bool getFlagSHT();	//returns sht flag state
		uint8_t *getPinSHT();	//returns the pins used for SHT

// 		void setupSHT(int*);	//setup function for SHT
		void setupSHT();	//setup function for SHT
		void startSHT(void);
		void resetSHT(void);
		void softResetSHT(void);
		void waitSHT(void);
		void measureSHT(int cmd);
		void writeByteSHT(void);
		void readByteSHT(void);
		int getStatusSHT(void);
		int shiftInSHT(void);
#endif

#if MINIBEE_ENABLE_PING == 1
	//ping
		bool getFlagPing();	//returns ping flag state
		uint8_t getPinPing();	//returns the pins used for Ping
// 		void setupPing(int*);	//setup function for Ping
		int readPing(void);
#endif

		//listener function
// 		void setupDigitalEvent(void (*event)(int, int));	//attach digital pin listener
// 		void digitalEvent(int pin, int state);	//digital pin event function (add it to the arduino sketch to receive the data)
// 		void attachSerialEvent(void (*event)(void));
// 		void SerialEvent(void);

	private:
		#define PIN_CONFIG_BYTES 18 // 23 for base config. 4 for other stuff, so 19 for the pins
		#define CONFIG_BYTES 64 // 23 for pin configs. then some for twi configuration (which is variable!)
		
		#define MAX_MESSAGE_SIZE 64
		#define AT_OK 167
		#define AT_ERROR 407
		#define DEST_ADDR "1"
		#define ESC_CHAR '\\' 
		#define DEL_CHAR '\n'
		#define CR '\r'
		
		#define S_NO_MSG '0'
		//server message types
		#define S_OUT 'O'
		#define S_RUN 'R'
		#define S_LOOP 'L'
		#define S_ANN 'A'
		#define S_QUIT 'Q'
		#define S_ID 'I'
		#define S_ME 'M'
		#define S_CONFIG 'C'
		#define S_SETTING 'S'
		#define S_CUSTOM 'E'
		
		//node message types
		#define N_DATA 'd'
		#define N_SER 's'
		#define N_INFO 'i'
		#define N_WAIT 'w'
		#define N_CONF 'c'
		
		uint8_t i;
		uint8_t byte_index;
		uint8_t escaping;
		uint8_t node_id;
		uint8_t config_id;
		uint8_t prev_conf_msg;
		uint8_t prev_id_msg;
		uint8_t prev_msg;
		char incoming;
		char msg_type;

		uint8_t status;
		
		bool loopback;
		bool remoteConfig;

		char *serial;

		// incoming message
		char *message;

		int msgInterval;
		uint8_t samplesPerMsg;
		uint8_t msg_id_send;
		
		uint8_t curSample;
		int smpInterval;
		char *outMessage;

		void setMeLed( uint8_t );
// 		void writeMePin( uint8_t );
		void setupMePin();
		
	//AT private commands
		int atGetStatus(void);
		void atSend(char *);
		void atSend(char *, uint8_t);

	//msg with network
		void slip(char);
// 		void slipSoft(char);
		bool checkNodeMsg( uint8_t nid, uint8_t mid );
		bool checkNotNodeMsg( uint8_t nid );
		bool checkConfMsg( uint8_t mid );
		bool checkIDMsg( uint8_t mid );
		void routeMsg(char, char*, uint8_t);

	//config 
		char *config; //array of pointers for all the config bytes
		void writeConfig(char *, uint8_t); // eeprom write
		void readConfig(void); // eeprom read
		void parseConfig(void); // parse the config

	// collecting sensor data:
		void dataFromInt( unsigned int output, int offset );
		void dataFromLong24( unsigned long output, int offset );
		char *data;
		int datacount;
		int datasize;

		bool twiOn;

#if MINIBEE_ENABLE_SHT == 1
		uint8_t mask;
		bool shtOn;
		uint8_t sht_pins[2];	//scl, sda  clock, data
#endif

#if MINIBEE_ENABLE_PING == 1
		bool pingOn;
		uint8_t ping_pin;	//ping pins (these could be more, but not right now)
#endif

		void setSetting( char * msg, uint8_t offset );
		void setADXL_range( char newrange );

		uint8_t me_pin;
		bool analog_in[8]; // sets whether analog in on 
		bool analog_precision[8]; // sets whether analog 10 bit precision is on or not

		bool pwm_on[6]; // sets whether pwm pin is on or not
		static uint8_t pwm_pins[]; // = { 3,5,6, 8,9,10 };
		char pwm_values[6]; // = {0,0,0, 0,0,0};
		
		bool digital_out[NRPINS]; // sets whether digital out on
		char digital_values[NRPINS];
		
		uint8_t isIOPin( uint8_t id );
		uint8_t isAnalogPin( uint8_t id );
		static uint8_t pin_ids[]; 
		static uint8_t anapin_ids[]; 
		
// 		#define ANAOFFSET 11

		bool digital_in[NRPINS]; // sets whether digital in on

		bool custom_pin[NRPINS]; // sets whether custom pin is configured
		uint8_t custom_size[NRPINS]; // sets size of custom pin data
		uint8_t customDataSize;

		#define PINOFFSET 2

#if MINIBEE_ENABLE_TWI == 1
/*#if MINIBEE_REVISION == 'A'
	// LIS302DL accelerometer addresses
		#define accel1Address 0x1C
		#define accelResultX 0x29
		#define accelResultY 0x2B
		#define accelResultZ 0x2D
#endif*/
		uint8_t * twi_devices;
		uint8_t nr_twi_devices;

#if MINIBEE_ENABLE_TWI_ADXL == 1
		ADXL345 * accelADXL;
#endif
#if MINIBEE_ENABLE_TWI_LISDL == 1
		LIS302DL * accelLIS;
#endif
#if MINIBEE_ENABLE_TWI_TMP == 1
		TMP102 * temp102;
#endif
#if MINIBEE_ENABLE_TWI_BMP == 1
		BMP085 * bmp085;
#endif
#if MINIBEE_ENABLE_TWI_HMC == 1
		HMC5843 * hmc58x3;
#endif
#endif

#if MINIBEE_ENABLE_SHT == 1
	// SHT sensor - See Sensirion Data sheet
		#define  SHT_T_CMD  0x03                
		#define  SHT_H_CMD  0x05
		#define  SHT_R_STAT 0x07
		#define  SHT_W_STAT 0x06
		#define  SHT_RST_CMD 0x1E
#endif

	// state machine for the MiniBee...
		#define STARTING 0
		#define SENSING 1
		#define WAITFORHOST 2
		#define WAITFORCONFIG 3
		#define ACTING 4
		#define PAUSING 5

	bool hasInput; // = false;
	bool hasOutput; // = false;
	bool hasCustom; // = false;

	uint8_t customInputs;
// 	uint8_t customSize;

// 	//listener functions
// 		void digitalUpdate(int pin, int status);	//function used to update digitalEvent
// 		friend void PCINT0_vect(void);	//interrupt vector
// 		void (*dEvent)(int, int);	//event listener being passed to listner functions
// 		
// 		void serialUpdate(void);
// 		void (*sEvent)(void);
// 		friend void USART_RX_vect(void);
// 	static void customMsgFuncWrapper( void* mb, char* msg );
};

// extern MiniBee Bee;	

#endif	
