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

#include "WProgram.h"
#include "MiniBee_APIn.h"

#define XBEE_SLEEP_PIN 2
#define STATUS_LED 4

	//server message types
#define S_NO_MSG '0'
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
#define N_ACTIVE 'a'
#define N_PAUSED 'p'

// state machine for the MiniBee...
#define STARTING 0
#define SENSING 1
#define WAITFORHOST 2
#define WAITFORCONFIG 3
#define ACTING 4
#define PAUSING 5


#define PIN_CONFIG_BYTES 19 // 23 for base config. 4 for other stuff, so 19 for the pins

uint8_t *serial;

uint8_t shCmd[] = {'S','H'};  // serial high
uint8_t slCmd[] = {'S','L'};  // serial low

AtCommandRequest atRequest = AtCommandRequest(slCmd);
AtCommandResponse atResponse = AtCommandResponse();

XBee xbee = XBee();

#define MAX_PAYLOAD 61
#define COORD_ADDR 0x0000

uint8_t payload[MAX_PAYLOAD+3]; //, 0x40,0x6A,0x69,0xAB };

// 16-bit addressing: Enter address of remote XBee, typically the coordinator
Tx16Request txs16 = Tx16Request(COORD_ADDR, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();


uint8_t MiniBee_API::pwm_pins[] = { 3,5,6, 9,10,11 };

MiniBee_API::MiniBee_API(){
  serial = NULL;
  outData = NULL;
  me_status = 0;

  loopback = false;
  status = STARTING;
  msg_type = S_NO_MSG;
  
  destination = COORD_ADDR;
  
  msg_id_send = 0;
  node_id = 0;
  config_id = 0;

  curSample = 0;
  datacount = 0;
  
  smpInterval = 50; // default value
  msgInterval = 50;
  samplesPerMsg = 1;

  hasInput = false;
  hasOutput = false;
  hasCustom = false;
  
  remoteConfig = 2;	

  prev_id_msg = 255;
  
  customDataSize = 0;
  customInputs = 0;

  void (*customMsgFunc)(char *) = NULL;
  void (*dataMsgFunc)(char *) = NULL;

#if MINIBEE_ENABLE_SHT == 1
  shtOn = false;
#endif
#if MINIBEE_ENABLE_TWI == 1
  twiOn = false;

  #if MINIBEE_ENABLE_TWI_ADXL == 1
	accelADXL = NULL;
#endif
#if MINIBEE_ENABLE_TWI_LISDL == 1
	accelLIS = NULL;
#endif
#if MINIBEE_ENABLE_TWI_BMP == 1
	bmp085 = NULL;
#endif
#if MINIBEE_ENABLE_TWI_TMP == 1
	temp102 = NULL;
#endif
#if MINIBEE_ENABLE_TWI_HMC == 1
	hmc58x3 = NULL;
#endif
  
#endif
#if MINIBEE_ENABLE_PING == 1
  pingOn = false;
#endif

}

void MiniBee_API::setup( long baud_rate, char boardrev ) {
  board_revision = boardrev;
  
  pinMode(STATUS_LED, OUTPUT);
  
  pinMode( XBEE_SLEEP_PIN, OUTPUT );
  digitalWrite(  XBEE_SLEEP_PIN, 0 );
  
  xbee.begin(baud_rate);
  
  delay( 500 );
  
  readXBeeSerial();
    
  sendXBeeSerial();
  
  actcount = 0;

  status = WAITFORHOST;
}

// TODO: check:

void MiniBee_API::loopStep( bool usedelay ){
  switch( status ){
    case STARTING:
      if ( usedelay ){ delay( 100 ); }
      break;
    case WAITFORHOST:
      if ( actcount == 0 ){ // send the serial number every 10 seconds
	sendXBeeSerial();
      }
      if ( usedelay ){ delay( 100 ); }
      break;
    case WAITFORCONFIG:
      if ( actcount == 0 ){ // send the wait message every 10 seconds
	msg_id_send++;
	msg_id_send = msg_id_send%256;
// 	sendTx16( N_WAIT, &config_id, 1 );
	sendTx16( N_WAIT, configInfo, 2 );
      }
      if ( usedelay ){ delay( 100 ); }
      break;
    case SENSING: // read sensors:
      datacount = readSensors( datacount );
      if ( curSample >= samplesPerMsg ){
	sendData();
	curSample = 0;
	datacount = 0;
      }
      if ( usedelay ){ delay( smpInterval ); }
      break;
    case ACTING:
      if ( actcount == 0 ){ // send an I'm active message every 100 smpIntervals
	sendActive();
      }
      if ( usedelay ){ delay( smpInterval ); }
      break;
    case PAUSING:
      if ( actcount == 0 ){ // send an I'm paused message every 100 smpIntervals
	sendPaused();
      }
      if ( usedelay ){ delay( 500 ); }
      break;
  }
  actcount++;
  actcount = actcount%100;
  
  readXBeePacket();
}

#define ANAPINS 8
#define ANAOFFSET 14
#define PINOFFSET 3

void MiniBee_API::dataFromInt( unsigned int output, int offset ){
    outData[offset]   = byte(output/256);
    outData[offset+1] = byte(output%256);
}

void MiniBee_API::dataFromLong24( unsigned long output, int offset ){
    output = output % 16777216; // 24 bits
    outData[offset]   = byte( output / 65536 );
    output = output % 65536; // 16 bits
    outData[offset+1] = byte(output / 256);
    outData[offset+2] = byte(output % 256);
}

uint8_t MiniBee_API::readSensors( uint8_t db ){
    unsigned int value;

    //--------- read digital sensors ---------- (0 - 3 bytes in total)

    uint8_t newdigital = 0;
    uint8_t bitj = 0;
    bool hasDigital = false;
    for ( uint8_t i = 0; i < nrpins; i++ ){
//       diginfo[i] = (char) digital_in[i];
	if ( digital_in[i] ){
	  hasDigital = true;
	  if ( bitj == 8 ){
	      bitj = 0;
	      outData[db] = newdigital;
	      db++;
	  }
	  newdigital += digitalRead( i + PINOFFSET ) << ( bitj++ );
	}
    }

    if ( hasDigital ){ // last byte
      outData[db] = newdigital;
      db++;  
    }

    //--------- read analog sensors -----------
    for ( uint8_t i = 0; i < ANAPINS; i++ ){
	if ( analog_in[i] ){
	    if ( analog_precision[i] ){
		value = analogRead( i );
		dataFromInt( value, db );
		db += 2;
	    } else {
		outData[db] = analogRead( i )/4;
		db++;      
	    }
	}
    }

#if MINIBEE_ENABLE_TWI == 1
    // read I2C/two wire interface for all devices
    int dbinc;
    if ( twiOn ){
	dbinc = readTWIdevices( db );
	db += dbinc;
    }
#endif

#if MINIBEE_ENABLE_SHT == 1
    // read SHT sensor
    if ( shtOn ){
	measureSHT( SHT_T_CMD );
	dataFromInt( (unsigned int) valSHT, db );
	db += 2;
	measureSHT( SHT_H_CMD );
	dataFromInt( (unsigned int) valSHT, db );
	db += 2;      
    }
#endif

#if MINIBEE_ENABLE_PING == 1
    // read ultrasound sensor
    if ( pingOn ){
	dataFromInt( readPing(), db );
	db += 2;
    }
#endif
    
    curSample++;
//     datacount = db;
    return db;
}

//end TODO: check/

void MiniBee_API::sendData(void){
  boolean result = false;
  byte count = 0;
    msg_id_send++;
    msg_id_send = msg_id_send%256;
    
    while ( !result && count < 3 ){ // try to send max. three times
      result = sendTx16( N_DATA, outData, datacount );
      count++;
    }
      //     datacount = 0;
}

void MiniBee_API::sendActive(void){
    msg_id_send++;
    msg_id_send = msg_id_send%256;
//     outMessage[0] = node_id;
//     outMessage[1] = msg_id_send;
// //     for ( i=0; i < datacount; i++ ){
// // 	outMessage[i+2] = data[i];
// //     }
    sendTx16( N_ACTIVE, outData, 0 );
}

void MiniBee_API::sendPaused(void){
    msg_id_send++;
    msg_id_send = msg_id_send%256;
//     outMessage[0] = node_id;
//     outMessage[1] = msg_id_send;
// //     for ( i=0; i < datacount; i++ ){
// // 	outMessage[i+2] = data[i];
// //     }
    sendTx16( N_PAUSED, outData, 0 );
}

void MiniBee_API::readXBeePacket(){
  uint8_t option = 0;
  uint8_t *data;
  uint8_t datasize = 0;
  uint8_t recvMsgType;
  uint16_t source;
  
  xbee.readPacket();
    
  if (xbee.getResponse().isAvailable()) {// got something
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      // got a rx packet
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
	xbee.getResponse().getRx16Response(rx16);
        option = rx16.getOption();
	source = rx16.getRemoteAddress16();
	datasize = rx16.getDataLength();
        data = rx16.getData();
	recvMsgType = rx16.getData(0);
      } else {
        xbee.getResponse().getRx64Response(rx64);
	option = rx64.getOption();
	source = 0;
	datasize = rx64.getDataLength();
        data = rx64.getData();
	recvMsgType = rx64.getData(0);
      }
      routeMsg( recvMsgType, data, datasize, source );
      // TODO check option, rssi bytes    
//       flashLed(STATUS_LED, 1, 10);
//     } else { // not something we were expecting
//       flashLed(STATUS_LED, 1, 10);    
//     }
    }
  }
//   sendTx16( N_INFO, data, datasize );
}

bool MiniBee_API::checkIDMsg( uint8_t mid ){
  bool res = ( mid != prev_id_msg);
  prev_id_msg = mid;
  return res;
}

void MiniBee_API::routeMsg(uint8_t type, uint8_t *msg, uint8_t size, uint16_t source ) {

  // msg[0] = type
  // msg[1] = msg id
  // msg[2:] = msg specific data
  
    if ( loopback ){
      msg_id_send++;
      msg_id_send = msg_id_send%256;
      sendTx16( N_INFO, msg, size );
    }
    
    switch(type) {
      case S_ANN:
	if ( remoteConfig > 0 ){
	  sendXBeeSerial();
	  status = WAITFORHOST;
	}
	break;
      case S_QUIT:
	if ( remoteConfig > 0 ){
	  status = WAITFORHOST;
	  //do something to stop doing anything
	}
	break;
      case S_LOOP: // Debugging option
	if ( checkIDMsg( msg[1] ) ){
	  setLoopback( msg[2] );
	}
	break;
      case S_RUN:
	if ( checkIDMsg( msg[1] ) ){
	  setRunning( msg[2] );
	}
	break;
      case S_ID:
	if ( remoteConfig > 0 ){
	  if ( checkIDMsg( msg[1] ) ){ 
	    bool serialCorrect = true;
	    for ( uint8_t j=0; j<8; j++ ){
		if ( serial[j] != msg[j+2] ){ // serial is msg[3,4,5, 6,7,8,9,10]
		  serialCorrect = false;
		}
	    }
	    if ( serialCorrect ){
	      node_id = msg[10];
	      config_id = msg[11]; // config id is msg[8]
	      status = WAITFORCONFIG;
	      configInfo[0] = node_id;
	      configInfo[1] = config_id;
	      sendTx16( N_WAIT, configInfo, 2 );
// 	      } else if ( size < 8 ) { // no new config
// 		readConfig();
//   		status = SENSING;
// 	      }
	    } else {
	      sendXBeeSerial();
	    }
	  }
	}
	break;
      case S_CONFIG:
	if ( remoteConfig > 1 ){
	  // check if right config_id:
	  if ( checkIDMsg( msg[1] ) ){
	    if ( config_id == msg[2] ){ // the config id I was waiting for:
// 	      writeConfig( msg, size );
// 	      readConfig();
	      readConfigMsg( msg, size );
	      if ( hasInput ){
		status = SENSING;
	      } else if ( hasOutput | hasCustom ){
		actcount = 0;
		status = ACTING;
		sendActive();
	      }
	    }
	  }
	}
	break;
      case S_ME:
	if ( checkIDMsg( msg[1] ) ){
	  setMeLed( msg[2] );
	}
	break;
      case S_OUT:
	if ( checkIDMsg( msg[1] ) ){
	  setOutputValues( msg, 2 );
	  setOutput();
	}
	break;
      case S_CUSTOM:
	if ( checkIDMsg( msg[1] ) ){
	    this->customMsgFunc( msg, size, source );
	}
	break;
      case N_DATA:
	if ( checkNotNodeMsg( source ) ){
	  this->dataMsgFunc( msg, size, source );
	}
	break;
    }
// }
}

void MiniBee_API::setOutputValues( uint8_t * msg, uint8_t offset ){
    uint8_t i = offset;
    for ( uint8_t j=0; j < 6; j++ ){
	if ( pwm_on[j] ){
	    pwm_values[j] = msg[i];
	    i++;
	}
    }
    for ( uint8_t j=0; j < nrpins; j++ ){
	if ( digital_out[j] ){
	    digital_values[j] = msg[i];
	    i++;
	}
    }
}

void MiniBee_API::setOutput(){
  for( uint8_t i=0; i<6; i++){
    if ( pwm_on[i] ){
      analogWrite( pwm_pins[i], pwm_values[i] );
    }
  } 
  for( uint8_t i=0; i<nrpins; i++){
    if ( digital_out[i] ){
      digitalWrite( i+PINOFFSET, digital_values[i] );
    }
  } 
}

bool MiniBee_API::checkNotNodeMsg( uint16_t nid ){
	bool res = ( (nid != node_id) );
	return res;
}

void MiniBee_API::setMeLed( uint8_t onoff ){
  me_status = onoff;
  digitalWrite( STATUS_LED, onoff );
}

void MiniBee_API::readConfigMsg(uint8_t *msg, uint8_t size){
  // msg[0] = 'C'
  // msg[1] = msg id
  // msg[2:] = configuration bytes
	config = (uint8_t*)malloc(sizeof(uint8_t) * (size-2));
	for(uint8_t i = 0;i < (size-2);i++){
	  config[i] = msg[i+2];
	}
	parseConfig();
	free(config);
	if ( hasInput ){
	    status = SENSING;
	} else if ( hasOutput ){
	    status = ACTING;
	}
}

bool MiniBee_API::isIOPin( uint8_t id ){
  bool isvalid = true;
  switch ( board_revision ){
    case 'A':
      break;
    case 'D':
      if ( id == 4  ) {
	isvalid = false;
      }
      // continue with cases of 'B'
    case 'B':
      if ( id == 12 | id == 13 ){
	isvalid = false;
      }
      break;
    case 'Z': // arduino
      if ( id > 19 ){
	isvalid = false;
      }
      break;
  }  
  return isvalid;
}

bool MiniBee_API::isAnalogPin( uint8_t id ){
  bool isvalid = true;
  uint8_t anapin = id - ANAOFFSET;
  if ( id < ANAOFFSET ){
      isvalid = false;
  }
  switch ( board_revision ){
    case 'A':
      break;
    case 'B':
    case 'D':
      if ( anapin == 4 | anapin == 5){
	isvalid = false;
      }
      break;
    case 'Z': // arduino
      if ( anapin > 5 ){
	isvalid = false;
      }
      break;
  }
  return isvalid;
}

void MiniBee_API::parseConfig(void){
  
  char info [3];
  
  uint8_t anapin;
  uint8_t iopin;
  uint8_t cfpin;
  uint8_t pin = 0;
  uint8_t datasizeout = 0;
  datasize = 0;

  config_id = config[0];
  msgInterval = config[1]*256 + config[2];
  samplesPerMsg = config[3];
  smpInterval = msgInterval / samplesPerMsg;

  if ( outData != NULL ){
    free(outData);
  }

  switch( board_revision ){
    case 'A':
    case 'B':
    case 'D':
      nrpins = 19;
      break;
    case 'Z':
      nrpins = 17;
      break;
  }
  
  for( uint8_t i = 0; i < nrpins; i++){
    cfpin = i + PINOFFSET; // index based on config bytes (pins 3 t/m 21)
    if ( isIOPin( cfpin ) ){
//     pin = pin_ids[i];
//     iopin = isIOPin( cfpin ); // index into pin_ids
/*    if ( iopin < NRPINS ){
      pin = pin_ids[ iopin ]; // actual pin number
*/
/*		 info[0] = (char) i;
		 info[1] = (char) pin;
		 info[2] = (char) config[i+4];
		 send( N_INFO, info, 3 );
*/
    if ( custom_pin[ i ] ){
      config[i+4] = Custom;
      hasCustom = true;
    }
    switch( config[i+4] ){
      case AnalogIn10bit:
	if ( isAnalogPin( cfpin ) ){
	  analog_precision[ cfpin - ANAOFFSET ] = true;
	  analog_in[ cfpin - ANAOFFSET ] = true;
	  datasize += 2;
	  hasInput = true;
	}
      break;
      case AnalogIn:
	if ( isAnalogPin( cfpin ) ){
	  analog_precision[ cfpin - ANAOFFSET ] = false;
	  analog_in[ anapin ] = true;
	  datasize += 1;
	  hasInput = true;
	}
	break;
      case DigitalIn:
	pinMode( cfpin, INPUT );
	digital_in[i] = true;
	datasize += 1;
	hasInput = true;
	break;
      case AnalogOut:
	for ( uint8_t j=0; j < 6; j++ ){
	  if ( pwm_pins[j] == cfpin ){
	    pwm_on[j] = true;
	    datasizeout += 1;
	    hasOutput = true;
	  }
	}
	break;
      case DigitalOut:
	digital_out[ i ] = true;
	pinMode( cfpin , OUTPUT );
	datasizeout += 1;
	hasOutput = true;
	break;
#if MINIBEE_ENABLE_SHT == 1
      case SHTClock:
	sht_pins[0] = cfpin;
	shtOn = true;
	pinMode( cfpin, OUTPUT );
	break;
      case SHTData:
	sht_pins[1] = cfpin;
	shtOn = true;
	pinMode( cfpin, OUTPUT );
	datasize += 4;
	hasInput = true;
	break;
#endif
#if MINIBEE_ENABLE_TWI == 1
      case TWIData:
      case TWIClock:
	twiOn = true;
	hasInput = true;
	break;
#endif
#if MINIBEE_ENABLE_PING == 1
      case Ping:
	pingOn = true;
	ping_pin = cfpin;
	datasize += 2;
	hasInput = true;
	break;
#endif
      case Custom:
	// pin is used in the custom part of the firmware
	custom_pin[ i ] = true;
	hasCustom = true;
	break;
      case NotUsed:
      case UnConfigured:
	break;
      }
    }
  }

#if MINIBEE_ENABLE_TWI == 1
  if ( twiOn ){
    nr_twi_devices = config[PIN_CONFIG_BYTES+4];
    twi_devices = (uint8_t*)malloc(sizeof(uint8_t) * nr_twi_devices);
    for(uint8_t i = 0;i < nr_twi_devices; i++ ){
      twi_devices[i] = config[PIN_CONFIG_BYTES+5+i];
      switch( twi_devices[i] ){
#if MINIBEE_ENABLE_TWI_ADXL == 1
	case TWI_ADXL345:
	  datasize += 6;
	  break;
#endif
#if MINIBEE_ENABLE_TWI_LISDL == 1
	case TWI_LIS302DL:
	  datasize += 3;
	  break;
#endif
#if MINIBEE_ENABLE_TWI_BMP == 1
	case TWI_BMP085:
	  datasize += 8; // 2 byte int, 3 byte long, 3 byte long
	  break;
#endif
#if MINIBEE_ENABLE_TWI_TMP == 1
	case TWI_TMP102:
	  datasize += 2;
	  break;
#endif
#if MINIBEE_ENABLE_TWI_HMC == 1
	case TWI_HMC58X3:
	  datasize += 6;
	  break;
#endif
	}
      }
    }
#endif
	
  datacount = 0;
  datasize += customDataSize;

#if MINIBEE_ENABLE_TWI == 1
  if ( twiOn ){
    Wire.begin();
    delay( 200 );
    setupTWIdevices();
  }
#endif
#if MINIBEE_ENABLE_SHT == 1
  if ( shtOn ){
    setupSHT();
  }
#endif
  // no need to setup ping

  uint8_t confSize = 9;
  uint8_t * configInfoN = (uint8_t*)malloc( sizeof(uint8_t) * (19*2 + confSize) );
  configInfoN[0] = node_id;
  configInfoN[1] = config_id;
  configInfoN[2] = samplesPerMsg;
  configInfoN[3] = (uint8_t) (msgInterval/256);
  configInfoN[4] = (uint8_t) (msgInterval%256);
  configInfoN[5] = datasize;
  configInfoN[6] = datasizeout;
  configInfoN[7] = customInputs;
  configInfoN[8] = customDataSize;
  for ( uint8_t i=0; i<nrpins; i++){
    if ( custom_pin[i] ){
      configInfoN[confSize] = i;
      configInfoN[confSize+1] = custom_size[i];
      confSize += 2;
    }
  }
  sendTx16( N_CONF, configInfoN, confSize );
  free( configInfoN );

  datasize = datasize * samplesPerMsg;

  outData = (uint8_t*)malloc(sizeof(uint8_t) * datasize);

// 	outMessage = (char*)malloc( sizeof(char) * (datasize + 2 ) );
// 	data = outMessage + 2*sizeof(char); // not sure if this is correct... test!!
}

void MiniBee_API::setCustomPins( uint8_t * ids, uint8_t * sizes, uint8_t n  ){
    for ( uint8_t i=0; i<n; i++ ){
	setCustomPin( ids[i], sizes[i] );
    }
}

void MiniBee_API::setCustomPin( uint8_t id, uint8_t size ){
    if ( id >= PINOFFSET ){
	custom_pin[ id-PINOFFSET ] = true;
	custom_size[ id-PINOFFSET ] = size;
    } // id's smaller than PINOFFSET allow for custom data without pin associated
    customDataSize += size;
    
    hasCustom = true;
    if ( size > 0 ){ hasInput = true; }
}

void MiniBee_API::setCustomInput( uint8_t noInputs, uint8_t size ){
    customInputs += noInputs;
    customDataSize += noInputs * size;

    hasCustom = true;
    if ( size > 0 ){ hasInput = true; }
}


void MiniBee_API::addCustomData( uint8_t * cdata, uint8_t n ){
    if ( status == SENSING ){
	for ( uint8_t i=0; i<n; i++){
	    outData[datacount] = cdata[i];
	    datacount++;  
	}
    }
}

void MiniBee_API::addCustomData( char * cdata, uint8_t n ){
    if ( status == SENSING ){
	for ( uint8_t i=0; i<n; i++){
	    outData[datacount] = (uint8_t) cdata[i];
	    datacount++;  
	}
    }
}

void MiniBee_API::addCustomData( int * cdata, uint8_t n ){
    if ( status == SENSING ){
	for ( uint8_t i=0; i<n; i++){
	dataFromInt( cdata[i], datacount );
	    datacount += 2;  
	}
    }
}

void MiniBee_API::setCustomCall( void (*customFunc)(uint8_t *, uint8_t, uint16_t ) ){
  customMsgFunc = customFunc;
}

void MiniBee_API::setDataCall( void (*dataFunc)(uint8_t *, uint8_t, uint16_t ) ){
  dataMsgFunc = dataFunc;
}

void MiniBee_API::setLoopback( uint8_t onoff ){
  loopback = ( onoff == 1 );
}

void MiniBee_API::setRemoteConfig( uint8_t level ){
    remoteConfig = level;
}

void MiniBee_API::setRunning( uint8_t onoff ){
    if ( onoff == 1 ){
      	if ( hasInput ){
	    status = SENSING;
	} else if ( hasOutput ){
	    status = ACTING;
	}
    } else if ( onoff == 0 ){
	status = PAUSING;
    }
    actcount = 0;
}

uint8_t * MiniBee_API::getOutData(){
    return outData;
}

int MiniBee_API::dataSize(){
    return datasize;
}


// TODO: this should set the ATMY!
void MiniBee_API::setID( uint8_t id ){
  node_id = id;
//   configInfo[0] = node_id;
//   configInfo[1] = config_id;
}

// uint8_t MiniBee_API::getId(void) { 
//   return node_id;
// }


void MiniBee_API::readXBeeSerial() {
  byte cnt;
  uint8_t *response1;
  uint8_t *response2;

  // get SH
  atRequest.setCommand(shCmd);  
  response1 = sendAtCommand();

    // set command to SL
  atRequest.setCommand(slCmd);  
  response2 = sendAtCommand();
  
  if ( serial != NULL ){
    free( serial );
  }

  serial = (uint8_t *)malloc( 12 * sizeof( uint8_t ) );
//   sizeof( response1 ) + sizeof(response2) );
//   serial = (uint8_t *)malloc( sizeof(response2) );
  
   for ( byte i = 0; i<4; i++ ){
     serial[i] = response1[i];
     serial[i+4] = response2[i];
//      serial[i] = response2[i];
   }

  free( response1 );
  free( response2 );
}

uint8_t* MiniBee_API::sendAtCommand() {
  uint8_t *response;

  // send the command
  xbee.send(atRequest);

  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) { // got a response!
    // should be an AT command response
    if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      xbee.getResponse().getAtCommandResponse(atResponse);
      if (atResponse.isOk()) {
//         atResponse.getCommand()[0];
//         atResponse.getCommand()[1];

        if (atResponse.getValueLength() > 0) {
	  response =  (uint8_t *)malloc(sizeof(uint8_t)* atResponse.getValueLength() );
          for (int i = 0; i < atResponse.getValueLength(); i++) {
            response[i] = atResponse.getValue()[i];
          }
// 	  flashLed(STATUS_LED, 10, 100);
        }
      } else { // error 
            flashLed(STATUS_LED, 3, 50);
//         atResponse.getStatus();
      }
    } else { // not an AT response
      flashLed(STATUS_LED, 4, 50);
//       xbee.getResponse().getApiId();
    }   
  } else { // time out   // at command failed, there may be an error code returned
    if (xbee.getResponse().isError()) {
      flashLed(STATUS_LED, 5, 70);
//       xbee.getResponse().getErrorCode();
    } else { // no response at all
      flashLed(STATUS_LED, 5, 100);
    }
  }
  return response;
}

void MiniBee_API::sendXBeeSerial(){
  serial[8] = MINIBEE_LIBVERSION;
  serial[9] = board_revision;
/// 1 byte with data of capabilities that may be commented out in the firmware lib...
  serial[10] = MINIBEE_ENABLE_PING*4 + MINIBEE_ENABLE_SHT*2 + MINIBEE_ENABLE_TWI;
  serial[11] = remoteConfig;
  sendTx16( N_SER, serial, 11 );
}

void MiniBee_API::setDestination( uint16_t addr ){
    destination = addr;
    txs16.setAddress16( destination );
}

boolean MiniBee_API::sendTx16( char type, uint8_t* data, uint8_t length ){
  payload[0] = (uint8_t) type;
//   payload[1] = node_id;
  payload[1] = msg_id_send;

  txs16.setPayloadLength( length + 2 );

  for ( uint8_t i=0; i<length; i++ ){
    payload[i+2] = data[i];
  }  
  txs16.setPayload( payload );
  
  xbee.send(txs16);
//   flashLed(STATUS_LED, 1, 100);

  digitalWrite( STATUS_LED, 0 );
  // after sending a tx request, we expect a status response
  // wait up to 5 seconds for the status response
  if (xbee.readPacket( 20 )) { // got a response!
	// should be a znet tx status            	
	if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
	  xbee.getResponse().getTxStatusResponse(txStatus);
		
	  // get the delivery status, the fifth byte
	  if (txStatus.getStatus() != SUCCESS) {
		// success.  time to celebrate
// 		flashLed(STATUS_LED, 5, 50);
// 	  } else {
		// the remote XBee did not receive our packet. is it powered on?
// 		flashLed(STATUS_LED, 3, 50);
	    digitalWrite( STATUS_LED, 1 );
	    return false;
	  }
	}      
    } else {
      // local XBee did not provide a timely TX Status Response -- should not happen
//       flashLed(STATUS_LED, 3, 50);
	digitalWrite( STATUS_LED, 1 );
	return false;
    }
    return true;
}

void MiniBee_API::flashLed(int pin, int times, int wait) {  
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      delay(wait);
    }
    digitalWrite( pin, me_status );
}


#if MINIBEE_ENABLE_TWI == 1
bool MiniBee_API::getFlagTWI(void) { 
	return twiOn;
}

void MiniBee_API::setupTWIdevices(void){
#if MINIBEE_ENABLE_TWI_ADXL == 1
  if ( accelADXL != NULL ){
      free( accelADXL );
      accelADXL = NULL;
  }
#endif
#if MINIBEE_ENABLE_TWI_LISDL == 1
  if ( accelLIS != NULL ){
      free( accelLIS );
      accelLIS = NULL;
  }
#endif
#if MINIBEE_ENABLE_TWI_BMP == 1
  if ( bmp085 != NULL ){
      free( bmp085 );
      bmp085 = NULL;
  }
#endif
#if MINIBEE_ENABLE_TWI_TMP == 1
  if ( temp102 != NULL ){
      free( temp102 );
      temp102 = NULL;
  }
#endif
#if MINIBEE_ENABLE_TWI_HMC == 1
  if ( hmc58x3 != NULL ){
      free( hmc58x3 );
      hmc58x3 = NULL;
  }
#endif
	for(uint8_t i = 0;i < nr_twi_devices; i++ ){
	  switch( twi_devices[i] ){
#if MINIBEE_ENABLE_TWI_ADXL == 1
	      case TWI_ADXL345:
		accelADXL = (ADXL345*) malloc( sizeof( ADXL345 ) );
		accelADXL->init();
// 		accelADXL = (ADXL345*) new ADXL345();
		accelADXL->powerOn();
		accelADXL->setJustifyBit( false );
		accelADXL->setFullResBit( true );
		accelADXL->setRangeSetting( 16 ); // 2: 2g, 4: 4g, 8: 8g, 16: 16g
		break;
#endif
#if MINIBEE_ENABLE_TWI_LISDL == 1
	      case TWI_LIS302DL:
		accelLIS = (LIS302DL*) malloc( sizeof( LIS302DL ) );
// 		accelLIS = new LIS302DL();
		accelLIS->setup();
		break;
#endif
#if MINIBEE_ENABLE_TWI_BMP == 1
	      case TWI_BMP085:
		bmp085 = (BMP085*) malloc( sizeof( BMP085 ) );
// 		bmp085 = new BMP085();
		bmp085->initialisation();
		bmp085->init();
		//bmp085.init(MODE_STANDARD, 1018.50, false);  //  false = using hpa units
                  // this initialization is useful for normalizing pressure to specific datum.
                  // OR setting current local hPa information from a weather station/local airport (QNH).
		//bmp085.init(MODE_STANDARD, 250.0, true);  // true = using meter units
                  // this initialization is useful if current altitude is known,
                  // pressure will be calculated based on TruePressure and known altitude.
		break;
#endif
#if MINIBEE_ENABLE_TWI_TMP == 1
	      case TWI_TMP102:
 		temp102 = (TMP102*) malloc( sizeof( TMP102 ) );
// 		temp102 = new TMP102();
		temp102->init();
		// no setup needed
		break;
#endif
#if MINIBEE_ENABLE_TWI_HMC == 1
	      case TWI_HMC58X3:
		hmc58x3 = (HMC5843*) malloc( sizeof( HMC5843 ) );
// 		temp102 = new TMP102();
		hmc58x3->init(false);
		hmc58x3->calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
		// Single mode conversion was used in calibration, now set continuous mode
		hmc58x3->setMode(0);
		break;
#endif
	    }
	}
}

int MiniBee_API::readTWIdevices( int dboff ){
	int dbplus = 0;
	
	int accx, accy, accz;
	unsigned int accx2, accy2, accz2;
	float bmpT, bmpA, bmpP;
	unsigned long bmpConv;

	for(uint8_t i = 0;i < nr_twi_devices; i++ ){
	  switch( twi_devices[i] ){
#if MINIBEE_ENABLE_TWI_ADXL == 1
	      case TWI_ADXL345:
		accelADXL->readAccel( &accx, &accy, &accz );
		accx2 = (unsigned int) (accx + 4096); // from twos complement signed int to unsigned int
		accy2 = (unsigned int) (accy + 4096); // from twos complement signed int to unsigned int
		accz2 = (unsigned int) (accz + 4096); // from twos complement signed int to unsigned int
		dataFromInt( accx2, dboff + dbplus );
		dataFromInt( accy2, dboff + dbplus + 2 );
		dataFromInt( accz2, dboff + dbplus + 4 );
// 		dboff += 6;
		dbplus =+ 6;
		break;
#endif
#if MINIBEE_ENABLE_TWI_LISDL == 1
	      case TWI_LIS302DL:
		accelLIS->read( &accx, &accy, &accz );
		accx2 = (unsigned int) (accx); // from twos complement signed int to unsigned int
		accy2 = (unsigned int) (accy); // from twos complement signed int to unsigned int
		accz2 = (unsigned int) (accz); // from twos complement signed int to unsigned int
		outData[dboff + dbplus] = (uint8_t) accx;
		outData[dboff + dbplus + 1] = (uint8_t) accy;
		outData[dboff + dbplus + 2] = (uint8_t) accz;
// 		dataFromInt( accx, dboff + dbplus );
// 		dataFromInt( accy, dboff + dbplus + 2 );
// 		dataFromInt( accz, dboff + dbplus + 4 );
		dbplus += 3;
// 		dboff += 6;
		break;
#endif
#if MINIBEE_ENABLE_TWI_BMP == 1
	      case TWI_BMP085:
		bmp085->getTemperature( &bmpT );
		accx2 = (unsigned int) ( (bmpT + 273 ) * 100 ); // temperature in centi - Kelvin
		dataFromInt( accx2, dboff + dbplus );
		dbplus += 2;

		bmp085->getPressure( &bmpP );
		bmpConv = (unsigned long) ( bmpP * 100 ); // pressure in hpascal * 100
		dataFromLong24( bmpConv, dboff + dbplus );
		dbplus += 3;

		bmp085->getAltitude( &bmpA );
		bmpConv = (unsigned long) ( (bmpA + 100 ) * 100 ); // altitude in centimeters
		dataFromLong24( bmpConv, dboff + dbplus );
		dbplus += 3;
		break;
#endif
#if MINIBEE_ENABLE_TWI_TMP == 1
	      case TWI_TMP102:
		temp102->readTemp();
		accx2 = (unsigned int) ( temp102->currentTemp + 2048 );
		dataFromInt( accx2, dboff + dbplus );
		dbplus += 2;
// 		dboff += 2;
		break;
#endif
#if MINIBEE_ENABLE_TWI_HMC == 1
	      case TWI_HMC58X3:
		/// DOES THIS ONE RETURN SIGNED OR UNSIGNED INTS?
		hmc58x3->getValuesInt( &accx, &accy, &accz );
		accx2 = (unsigned int) ( accx + 2048 );
		dataFromInt( accx2, dboff + dbplus );
		dbplus += 2;
		accy2 = (unsigned int) ( accy + 2048 );
		dataFromInt( accy2, dboff + dbplus );
		dbplus += 2;
		accz2 = (unsigned int) ( accz + 2048 );
		dataFromInt( accz2, dboff + dbplus );
		dbplus += 2;
		break;
#endif
	    }
	}
	return dbplus;
}

#endif


#if MINIBEE_ENABLE_SHT == 1
//SHT
bool MiniBee_API::getFlagSHT(void) { 
    return shtOn;
}

uint8_t* MiniBee_API::getPinSHT(void) {
	return sht_pins; 
}

void MiniBee_API::setupSHT() {
// void MiniBee_API::setupSHT(int* pins) {
	//pins[0] is scl pins[1] is sda
// 	*sht_pins = *pins;
	pinMode(sht_pins[0], OUTPUT);
	digitalWrite(sht_pins[0], HIGH);     
	pinMode(sht_pins[1], OUTPUT);   
	startSHT();
}

void MiniBee_API::startSHT(void) {
	pinMode(sht_pins[1], OUTPUT);
	digitalWrite(sht_pins[1], HIGH);
	digitalWrite(sht_pins[0], HIGH);    
	digitalWrite(sht_pins[1], LOW);
	digitalWrite(sht_pins[0], LOW);
	digitalWrite(sht_pins[0], HIGH);
	digitalWrite(sht_pins[1], HIGH);
	digitalWrite(sht_pins[0], LOW);
}

void MiniBee_API::resetSHT(void) {
	shiftOut(sht_pins[1], sht_pins[0], LSBFIRST, 0xff);
	shiftOut(sht_pins[1], sht_pins[0], LSBFIRST, 0xff);
	startSHT();
}

void MiniBee_API::softResetSHT(void) {
	resetSHT();
	ioSHT = SHT_RST_CMD;
	ackSHT = 1;
	writeByteSHT();
	delay(15);
}

void MiniBee_API::waitSHT(void) {
	delay(5);
	int j = 0;
	while(j < 600) {
		if(digitalRead(sht_pins[1]) == 0) j = 2600;
		delay(1);
		j++;
	}
}

void MiniBee_API::measureSHT(int cmd) {
	softResetSHT();
	startSHT();
	ioSHT = cmd;

	writeByteSHT();   
	waitSHT();          
	ackSHT = 0;      

	readByteSHT();
	int msby;                  
	msby = ioSHT;
	ackSHT = 1;

	readByteSHT();          
	valSHT = msby;           
	valSHT = valSHT * 0x100;
	valSHT = valSHT + ioSHT;
	if(valSHT <= 0) valSHT = 1;
}

void MiniBee_API::readByteSHT(void) {
	ioSHT = shiftInSHT();
	digitalWrite(sht_pins[1], ackSHT);
	pinMode(sht_pins[1], OUTPUT);
	digitalWrite(sht_pins[0], HIGH);
	digitalWrite(sht_pins[0], LOW);
	pinMode(sht_pins[1], INPUT);
	digitalWrite(sht_pins[1], LOW);
}

void MiniBee_API::writeByteSHT(void) {
	pinMode(sht_pins[1], OUTPUT);
	shiftOut(sht_pins[1], sht_pins[0], MSBFIRST, ioSHT);
	pinMode(sht_pins[1], INPUT);
	digitalWrite(sht_pins[1], LOW);
	digitalWrite(sht_pins[0], LOW);
	digitalWrite(sht_pins[0], HIGH);
	ackSHT = digitalRead(sht_pins[1]);
	digitalWrite(sht_pins[0], LOW);
}

int MiniBee_API::getStatusSHT(void) {
	softResetSHT();
	startSHT();
	ioSHT = SHT_R_STAT;	//R_STATUS

	writeByteSHT();
	waitSHT();
	ackSHT = 1;

	readByteSHT();
	return ioSHT;
}

int MiniBee_API::shiftInSHT(void) {
	int cwt = 0;
	for(shtMask = 128; shtMask >= 1; shtMask >>= 1) {
		digitalWrite(sht_pins[0], HIGH);
		cwt = cwt + shtMask * digitalRead(sht_pins[1]);
		digitalWrite(sht_pins[0], LOW);
	}
	return cwt;
}
#endif

#if MINIBEE_ENABLE_PING == 1
//PING
bool MiniBee_API::getFlagPing(void) { 
  return pingOn;
} 

uint8_t MiniBee_API::getPinPing(void) { 
	return ping_pin; 
}

int MiniBee_API::readPing(void) {
	long ping;

	// The Devantech US device is triggered by a HIGH pulse of 10 or more microseconds.
	// We give a short LOW pulse beforehand to ensure a clean HIGH pulse.
	pinMode(ping_pin, OUTPUT);
	digitalWrite(ping_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(ping_pin, HIGH);
	delayMicroseconds(11);
	digitalWrite(ping_pin, LOW);

	// The same pin is used to read the signal from the Devantech Ultrasound device: a HIGH
	// pulse whose duration is the time (in microseconds) from the sending
	// of the ping to the reception of its echo off of an object.
	pinMode(ping_pin, INPUT);
	ping = pulseIn(ping_pin, HIGH);

	//max value is 30000 so easily fits in an int
	return int(ping);
}
#endif
