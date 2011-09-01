#include "MBSerial.h"

// #include <NewSoftSerial.h>

uint8_t MBSerial::pwm_pins[] = { 3,5,6, 9,10,11 };

// arduino directly connected
	uint8_t MBSerial::pin_ids[] = {2,3, 4,5, 6,7, 8,9, 10,11, 12,13, 14,15, 16,17 ,18,19 }; // ids of I/O pins
	// pin 4 is not actually used anymore - it is now the ME led
	// 18,19 are TWI
	// 3,5,6,7,8,9,10,11 are digital pins
	// 14,15,16,17,20,21 are analog pins
	uint8_t MBSerial::anapin_ids[] = {14,15,16,17 ,18,19 }; // ids of I/O pins	
//		#define ANAOFFSET 9


MBSerial::MBSerial() {
// 	pwm_pins = { 3,5,6, 8,9,10 };
	
#if MINIBEE_ENABLE_SHT == 1
	shtOn = false;
#endif
#if MINIBEE_ENABLE_TWI == 1
	twiOn = false;
#endif
#if MINIBEE_ENABLE_PING == 1
	pingOn = false;
#endif
	prev_msg = 0;
	prev_conf_msg = 0;
	prev_id_msg = 0;

	curSample = 0;
	datacount = 0;
	msg_id_send = 0;
	
	for ( i = 0; i<8; i++ ){
	    analog_precision[i] = false; // false is 8bit, true is 10bit
	    analog_in[i] = false;
	}
	for ( i = 0; i<NRPINS; i++ ){
	    digital_in[i] = false;
	    digital_out[i] = false;
	    digital_values[i] = 0;
	    custom_pin[i] = false;
	    custom_size[i] = 0;
	}
	for ( i=0; i<6; i++ ){
	    pwm_on[i] = false;
	    pwm_values[i] = 0;
	}

	smpInterval = 5; // default value
	msgInterval = 5;
	samplesPerMsg = 1;
	
	customDataSize = 0;
	
	loopback = false;
	remoteConfig = true;
	
// 	useSoftSerial = false;

	status = STARTING;
	msg_type = S_NO_MSG;

	message = (char*)malloc(sizeof(char) * MAX_MESSAGE_SIZE);
	
	void (*customMsgFunc)(char *) = NULL;

	hasInput = false;
	hasOutput = false;
	hasCustom = false;
	serial = "00000000000000\0";
#if MINIBEE_ENABLE_TWI_ADXL == 1
	accelADXL = NULL;
#endif
}

// MBSerial Bee = MBSerial();

void MBSerial::openSerial(long baud_rate) {
	Serial.begin(baud_rate);
// 	setupMePin();
}

void MBSerial::setID( uint8_t id ){
    node_id = id;
}

char * MBSerial::getData(){
    return data;
}

int MBSerial::dataSize(){
    return datasize;
}

void MBSerial::begin(long baud_rate) {
	openSerial(baud_rate);
	delay( 100 );
	status = STARTING;
	sendSerialNumber();
}

void MBSerial::doLoopStep(void){
  int bytestoread;
  // do something based on current status:
  switch( status ){
      case SENSING:
	  //send( N_INFO, "sensing" );
	  // read sensors:
	  datacount = readSensors( datacount );
	  if ( curSample >= samplesPerMsg ){
	      sendData();
	      curSample = 0;
	      datacount = 0;
	  }
	  delay( smpInterval );
	  break;
      case STARTING:
	  sendSerialNumber();
//          send( N_INFO, "starting", 8 );
	  delay( 1000 );
	  break;
      case WAITFORCONFIG:
//          send( N_INFO, "waitforconfig" );
	  delay( 100 );
	  break;
      case WAITFORHOST:
//       send( N_INFO, "waitforhost" );
	  delay( 100 );
	  break;
      case ACTING:
	  delay( smpInterval );
	  break;
      case PAUSING:
	  delay( 500 );
	  break;
    }
  
    // read any new data from XBee:
    // do this at the end, so status change happens at the beginning of next loop
    bytestoread = Serial.available();
    if ( bytestoread > 0 ){
//        send( N_INFO, "reading" );
	for ( i = 0; i < bytestoread; i++ ){
	    read();      
	}
//   } else {
//       send( N_INFO, "no data" );
    }
}

void MBSerial::setCustomPins( uint8_t * ids, uint8_t * sizes, uint8_t n  ){
    for ( i=0; i<n; i++ ){
	setCustomPin( ids[i], sizes[i] );
    }
}

void MBSerial::setCustomInput( uint8_t noInputs, uint8_t size ){
    customInputs = noInputs;
    customDataSize += noInputs * size;

    hasCustom = true;
    if ( size > 0 ){ hasInput = true; }
}

void MBSerial::setCustomPin( uint8_t id, uint8_t size ){
    if ( id >= PINOFFSET ){
	custom_pin[ pin_ids[id-PINOFFSET] ] = true;
	custom_size[ pin_ids[id-PINOFFSET] ] = size;
    } // id's smaller than PINOFFSET allow for custom data without pin associated
    customDataSize += size;
    
    hasCustom = true;
    if ( size > 0 ){ hasInput = true; }
    
//     char info [2];
//     info[0] = (char) id;
//     info[1] = (char) size;
//     send( N_INFO, info, 2 );
}

void MBSerial::addCustomData( uint8_t * cdata, uint8_t n ){
    if ( status == SENSING ){
	for ( i=0; i<n; i++){
	    data[datacount] = (char) cdata[i];
	    datacount++;  
	}
    }
}

void MBSerial::addCustomData( char * cdata, uint8_t n ){
    if ( status == SENSING ){
	for ( i=0; i<n; i++){
	    data[datacount] = cdata[i];
	    datacount++;  
	}
    }
}

void MBSerial::addCustomData( int * cdata, uint8_t n ){
    if ( status == SENSING ){
	for ( i=0; i<n; i++){
	dataFromInt( cdata[i], datacount );
	    datacount += 2;  
	}
    }
}

void MBSerial::setCustomCall( void (*customFunc)(char * ) ){
//    customMsgFunc = customFunc;
    customMsgFunc = customFunc;
    hasOutput = true;
}

void MBSerial::setDataCall( void (*dataFunc)(char * ) ){
    dataMsgFunc = dataFunc;  
}

// void MBSerial::customMsgFuncWrapper( void* mb, char* msg ){
//   MBSerial * me = (MBSerial*) mb;
//   me->parseCustomMsg( msg );
// }

void MBSerial::send(char type, char *p, int size) {
	Serial.print(ESC_CHAR, BYTE);
	Serial.print(type, BYTE);
	for(i = 0;i < size;i++) slip(p[i]);
	Serial.print(DEL_CHAR, BYTE);
}

void MBSerial::slip(char c) {
	if((c == ESC_CHAR) || (c == DEL_CHAR) || (c == CR))
	    Serial.print(ESC_CHAR, BYTE);
	Serial.print(c, BYTE);
}

void MBSerial::read() {
	incoming = Serial.read();
	if(escaping) {	//escape set
		if((incoming == ESC_CHAR)  || (incoming == DEL_CHAR) || (incoming == CR)) {	//escape to integer
			if ( msg_type != S_NO_MSG ){ // only add if message type set
			  message[byte_index] = incoming;
			  byte_index++;
			}
		} else {	//escape to char
			msg_type = incoming;
		}
		escaping = false;
	} else {	//escape not set
		if(incoming == ESC_CHAR) {
			escaping = true;
		} else if(incoming == DEL_CHAR) {	//end of msg
			message[byte_index] = '\0'; // null-termination
			routeMsg(msg_type, message, byte_index);	//route completed message
			msg_type = S_NO_MSG;
			byte_index = 0;	//reset buffer index
		} else {
			if ( msg_type != S_NO_MSG ){ // only add if message type set
			  message[byte_index] = incoming; 
			  byte_index++;
			}
		}
	}
}

bool MBSerial::checkNodeMsg( uint8_t nid, uint8_t mid ){
	bool res = ( (nid == node_id)  && ( mid != prev_msg) );
	prev_msg = mid;
	return res;
}

bool MBSerial::checkNotNodeMsg( uint8_t nid ){
	bool res = ( (nid != node_id) );
	return res;
}

bool MBSerial::checkConfMsg( uint8_t mid ){
	bool res = ( mid != prev_conf_msg);
	prev_conf_msg = mid;
	return res;
}

bool MBSerial::checkIDMsg( uint8_t mid ){
	bool res = ( mid != prev_id_msg);
	prev_id_msg = mid;
	return res;
}

void MBSerial::routeMsg(char type, char *msg, uint8_t size) {
	uint8_t len;
	char * ser;

	if ( loopback ){
	  char * loopbackMsg = (char *)malloc(sizeof(char)* (size + 2 ) );
	  loopbackMsg[0] = type;
	  loopbackMsg[1] = size;
	  for ( i=0; i<size; i++ ){
	      loopbackMsg[i+2] = msg[i];
	  }
	  send( N_INFO, loopbackMsg, size + 2 );
	  free( loopbackMsg );
	}

	switch(type) {
		case S_ANN:
			if ( remoteConfig ){
				sendSerialNumber();
				status = WAITFORHOST;
  // 			send( N_INFO, "waitforhost", 11 );
			}
			break;
		case S_QUIT:
			if ( remoteConfig ){
				status = WAITFORHOST;
			//do something to stop doing anything
			}
// 			send( N_INFO, "waitforhost", 11 );
			break;
		case S_ID:
			if ( remoteConfig ){
			    if ( checkIDMsg( msg[0] ) ){
				len = strlen(serial);
				ser = (char *)malloc(sizeof(char)* (len + 1 ) );
				for(i = 0;i < len;i++)
				    { ser[i] = msg[i+1]; }
				ser[len] = '\0';
				if(strcmp(ser, serial) == 0){
				    node_id = msg[len+1];	//writeConfig(msg);
				    if ( size == (len+3) ){
					config_id = msg[len+2];
					status = WAITFORCONFIG;
					char configInfo[2];
					configInfo[0] = node_id;
					configInfo[1] = config_id;
					send( N_WAIT, configInfo, 2 );
			// 			send( N_INFO, "waitforconfig", 13 );
				    } else if ( size == (len+2) ) {
					readConfig();
					status = SENSING;
			// 				send( N_INFO, "sensing", 7 );
				    }
// 		    		} else {
// // 		    		    send( N_INFO, "wrong serial number", 19 );
// 		    			    send( N_INFO, ser, len );
				}
				free(ser);
			    }
			}
			break;
		case S_ME:
			if ( checkIDMsg( msg[0] ) ){
				len = strlen(serial);
				ser = (char *)malloc(sizeof(char)* (len + 1 ) );
				for(i = 0;i < len;i++)
				    { ser[i] = msg[i+1]; }
				ser[len] = '\0';
				if(strcmp(ser, serial) == 0){
				    setMeLed( msg[len+1] );
				}
				free(ser);
			}
			break;
		case S_CONFIG:
// 		  send( N_INFO, (char*) size, 1 );
//  		  send( N_INFO, msg, size  );
			if ( remoteConfig ){
			// check if right config_id:
			    if ( checkConfMsg( msg[0] ) ){
				if ( (msg[1] == node_id) && (msg[2] == config_id) ){
				    writeConfig( msg, size );
				    readConfig();
	      //                readConfigMsg( msg, size );
				    if ( hasInput ){
				      status = SENSING;
				    } else if ( hasOutput ){
				      status = ACTING;
				    }
	      //                send( N_INFO, "sensing", 7 );
			      }
			  }
			}
			break;
		case S_SETTING:
		  if ( checkNodeMsg( msg[0], msg[1] ) ){
		      setSetting( msg, size );
		  }
		case S_RUN:
			if ( checkNodeMsg( msg[0], msg[1] ) ){
			  setRunning( msg[2] );
			}
			break;
		case S_LOOP:
			if ( checkNodeMsg( msg[0], msg[1] ) ){
			  setLoopback( msg[2] );
			}
			break;
		/*
		case S_PWM:
			if ( checkNodeMsg( msg[0], msg[1] ) ){
			    for( i=0; i<6; i++){
			      pwm_values[i] = msg[2+i];
			    }
			    setPWM();
			}
			break;
		case S_DIGI:
			if ( checkNodeMsg( msg[0], msg[1] ) ){
			    for( i=0; i< (size-2); i++){
			      digital_values[i] = msg[2+i];
			    }
			    setDigital();
			}
			break;
		*/
		case S_OUT:
			if ( checkNodeMsg( msg[0], msg[1] ) ){
			  setOutputValues( msg, 2 );
			  setOutput();
			}
			break;
		case S_CUSTOM:
			if ( checkNodeMsg( msg[0], msg[1] ) ){
			    this->customMsgFunc( msg );
			}
			break;
		case N_DATA:
			if ( checkNotNodeMsg( msg[0] ) ){
			    this->dataMsgFunc( msg );
			}
			break;
// 		default:
// 			break;
		}
}

void MBSerial::setMeLed( uint8_t onoff ){
    digitalWrite( me_pin, onoff );
}

void MBSerial::setSetting( char * msg, uint8_t offset ){
  i = offset;
  switch ( msg[i] ){
    case 'a': // set range setting for ADXL345
      setADXL_range( msg[i+1] );
      break;
  }
}

void MBSerial::setADXL_range( char newrange ){
#if MINIBEE_ENABLE_TWI_ADXL == 1
  if ( accelADXL != NULL ){
    accelADXL->setRangeSetting( newrange );
  }
#endif
}

void MBSerial::setRemoteConfig( bool onoff ){
    remoteConfig = onoff;
}

void MBSerial::setRunning( uint8_t onoff ){
    if ( onoff == 1 ){
	status = SENSING;
    } else if ( onoff == 0 ){
	status = PAUSING;
    }
}

void MBSerial::setLoopback( uint8_t onoff ){
    if ( onoff == 1 ){
	loopback = true;
    } else if ( onoff == 0 ){
	loopback = false;
    }
}

void MBSerial::setOutputValues( char * msg, uint8_t offset ){
    i = offset;
    for ( uint8_t j=0; j < 6; j++ ){
	if ( pwm_on[j] ){
	    pwm_values[j] = msg[i];
	    i++;
	}
    }
    for ( uint8_t j=0; j < NRPINS; j++ ){
	if ( digital_out[j] ){
	    digital_values[j] = msg[i];
	    i++;
	}
    }
}

void MBSerial::setOutput(){
	for( i=0; i<6; i++){
	if ( pwm_on[i] ){
	    analogWrite( pwm_pins[i], pwm_values[i] );
	}
	} 
	for( i=0; i<NRPINS; i++){
	if ( digital_out[i] ){
	    digitalWrite( pin_ids[i], digital_values[i] );
	}
	} 
}

void MBSerial::dataFromInt( unsigned int output, int offset ){
    data[offset]   = byte(output/256);
    data[offset+1] = byte(output%256);
}

void MBSerial::dataFromLong24( unsigned long output, int offset ){
    output = output % 16777216; // 24 bits
    data[offset]   = byte( output / 65536 );
    output = output % 65536; // 16 bits
    data[offset+1] = byte(output / 256);
    data[offset+2] = byte(output % 256);
}

uint8_t MBSerial::readSensors( uint8_t db ){
    unsigned int value;
    // read analog sensors
    for ( i = 0; i < 8; i++ ){
	if ( analog_in[i] ){
	    if ( analog_precision[i] ){
		value = analogRead(i);
		dataFromInt( value, db );
		db += 2;
	    } else {
		data[db] = analogRead(i)/4;
		db++;      
	    }
	}
    }
    
    // read digital sensors
    for ( i = 0; i < NRPINS; i++ ){
//       diginfo[i] = (char) digital_in[i];
	if ( digital_in[i] ){
	//TODO this can be done way more clever by shifting the results into 3 bytes, resulting in shorter messages to be sent.
	    data[db] = digitalRead( pin_ids[i] );
	    db++;
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

void MBSerial::sendData(void){
    msg_id_send++;
    msg_id_send = msg_id_send%256;
    outMessage[0] = node_id;
    outMessage[1] = msg_id_send;
//     for ( i=0; i < datacount; i++ ){
// 	outMessage[i+2] = data[i];
//     }
    send( N_DATA, outMessage, datacount+2 );
}

uint8_t MBSerial::getId(void) { 
	return node_id;
}

void MBSerial::sendSerialNumber(void){
	int size = strlen(serial);
	char * serdata = (char*)malloc(sizeof(char) * (size + 3) );
	serdata = strcpy( serdata, serial );
	serdata[ size ] = MINIBEE_LIBVERSION;
	serdata[ size+1 ] = MINIBEE_REVISION;
/// 1 byte with data of capabilities that may be commented out in the firmware lib...
	serdata[ size+2 ] = MINIBEE_ENABLE_PING*4 + MINIBEE_ENABLE_SHT*2 + MINIBEE_ENABLE_TWI;
//  	serdata[ size+3 ] = '\0';
	send(N_SER, serdata, size+3 );
	free( serdata );
//	send(N_SER, serial, strlen(serial) );
}

// void MBSerial::writeMePin( uint8_t mepin ){
//   eeprom_write_byte( (uint8_t *) CONFIG_BYTES + 1, mepin );
//   char info [2];
//   info[0] = (char) mepin;
//   info[1] = (char) me_pin;
//   send( N_INFO, info, 2 );
// }

void MBSerial::setupMePin(){
  me_pin = 4;
  pinMode( me_pin, OUTPUT );
}

void MBSerial::writeConfig(char *msg, uint8_t size) {
// 	eeprom_write_byte((uint8_t *) i, id ); // writing id
	for(i = 0;i < (size-2);i++){
	    eeprom_write_byte((uint8_t *) i, msg[i+2]);
	    //write byte to memory
	}
// 	send( N_INFO, msg, size );
}

void MBSerial::readConfigMsg(char *msg, uint8_t size){
	config = (char*)malloc(sizeof(char) * size);
	for(i = 0;i < (size-1);i++){
	  config[i] = msg[i+1];
	}
	parseConfig();
	free(config);
	if ( hasInput ){
	    status = SENSING;
	} else if ( hasOutput ){
	    status = ACTING;
	}
}

void MBSerial::readConfig(void) {
	config = (char*)malloc(sizeof(char) * CONFIG_BYTES);
	for(i = 0;i < CONFIG_BYTES;i++) config[i] = eeprom_read_byte((uint8_t *) i);
// 	send( N_INFO, config, CONFIG_BYTES );
	parseConfig();
	free(config);
}

uint8_t MBSerial::isIOPin( uint8_t id ){
  uint8_t isvalid = 21;
  for ( uint8_t j = 0; j<NRPINS; j++ ){
      if ( pin_ids[j] == id ){
	  isvalid = j;
      }
  }
  return isvalid;
}

uint8_t MBSerial::isAnalogPin( uint8_t id ){
  uint8_t isvalid = 7;
  for ( uint8_t j = 0; j<6; j++ ){
      if ( anapin_ids[j] == id ){
	  isvalid = j;
      }
  }
  return isvalid;
}

void MBSerial::parseConfig(void){
  char info [3];
  
	uint8_t anapin;
	uint8_t iopin;
	uint8_t cfpin;
	uint8_t pin = 0;
	uint8_t datasizeout = 0;
	datasize = 0;

	free(outMessage);

	config_id = config[0];
	msgInterval = config[1]*256 + config[2];
	samplesPerMsg = config[3];
	
// 	send( N_INFO, config, CONFIG_BYTES );
	
	for(i = 0;i < PIN_CONFIG_BYTES;i++){
	    cfpin = i + PINOFFSET; // index based on config bytes (pins 3 t/m 21)
// 	    pin = pin_ids[i];
	    iopin = isIOPin( cfpin ); // index into pin_ids
	    if ( iopin < NRPINS ){
		pin = pin_ids[ iopin ]; // actual pin number
/*		 info[0] = (char) i;
		 info[1] = (char) pin;
		 info[2] = (char) config[i+4];
		 send( N_INFO, info, 3 );*/
		if ( custom_pin[ iopin ] ){
		  config[i+4] = Custom;
		  hasCustom = true;
		}
		switch( config[i+4] ){
			case AnalogIn10bit:
			    anapin = isAnalogPin( pin );
			    if ( anapin < 7 ){ // check whether analog pin
				analog_precision[ anapin ] = true;
				analog_in[ anapin ] = true;
// 				pinMode( pin, INPUT );
				datasize += 2;
				hasInput = true;
			    }
			    break;
			case AnalogIn:
			    anapin = isAnalogPin( pin );
			    if ( anapin < 7 ){ // check whether analog pin
				analog_precision[ anapin ] = false;
				analog_in[ anapin ] = true;
// 				pinMode( pin, INPUT );
				datasize += 1;
				hasInput = true;
			    }
			    break;
			case DigitalIn:
			    pinMode( pin, INPUT );
			    digital_in[iopin] = true;
			    datasize += 1;
			    hasInput = true;
			    break;
			case AnalogOut:
			    for ( int j=0; j < 6; j++ ){
				if ( pwm_pins[j] == pin ){
// 				    pinMode( pin, OUTPUT );
				    pwm_on[j] = true;
				    datasizeout += 1;
				    hasOutput = true;
				}
			    }
			    break;
			case DigitalOut:
			    digital_out[ iopin ] = true;
			    pinMode( pin , OUTPUT );
			    datasizeout += 1;
			    hasOutput = true;
			    break;
	#if MINIBEE_ENABLE_SHT == 1
			case SHTClock:
			    sht_pins[0] = pin;
			    shtOn = true;
			    pinMode(  pin, OUTPUT );
			    break;
			case SHTData:
			    sht_pins[1] = pin;
			    shtOn = true;
			    pinMode( pin, OUTPUT );
			    datasize += 4;
			    hasInput = true;
			    break;
	#endif
	#if MINIBEE_ENABLE_TWI == 1
			case TWIData:
// 			    datasize += 6; // datasize will be determined in twi setup
			case TWIClock:
// 			  	send( N_INFO, "setting twi", 12 );
			    twiOn = true;
			    hasInput = true;
			    break;
	#endif
	#if MINIBEE_ENABLE_PING == 1
			case Ping:
			    pingOn = true;
			    ping_pin = pin;
			    datasize += 2;
			    hasInput = true;
			    break;
	#endif
			case Custom:
			    // pin is used in the custom part of the firmware
			    custom_pin[ iopin ] = true;
			    hasCustom = true;
			    break;
/*			case MeID:
			    me_pin = pin;
			    pinMode( me_pin, OUTPUT );
			    break;*/
			case NotUsed:
			    break;
			case UnConfigured:
			    break;
// 			default:
// 			    send( N_INFO, "unknown pin", 12 );
// 			    break;
// 		    }
		}
	    }
	}

// 	send( N_INFO, "parsed config", 14 );
// 	writeMePin( me_pin );

// 	readMePin();

#if MINIBEE_ENABLE_TWI == 1
// 	send( N_INFO, "checking twi", 13 );
	if ( twiOn ){
//  	  send( N_INFO, "twi on", 7 );
	  nr_twi_devices = config[PIN_CONFIG_BYTES+4];
//  	  send( N_INFO, (char*) &nr_twi_devices, 1 );
	  twi_devices = (uint8_t*)malloc(sizeof(uint8_t) * nr_twi_devices);
	  for(i = 0;i < nr_twi_devices; i++ ){
	    twi_devices[i] = config[PIN_CONFIG_BYTES+5+i];
	    switch( twi_devices[i] ){
#if MINIBEE_ENABLE_TWI_ADXL == 1
	      case TWI_ADXL345:
		datasize += 6;
		break;
#endif
#if MINIBEE_ENABLE_TWI_LISDL == 1
	      case TWI_LIS302DL:
		datasize += 6;
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
// 	  send( N_INFO, (char*) twi_devices, nr_twi_devices );
// 	  send( N_INFO, (char*) &datasize, 1 );
	}
#endif
	
	datacount = 0;
	datasize += customDataSize;
	datasize = datasize * samplesPerMsg;

// 	free(data);

// 	data = (char*)malloc(sizeof(char) * datasize);
	smpInterval = msgInterval / samplesPerMsg;

#if MINIBEE_ENABLE_TWI == 1
	if ( twiOn ){
	    Wire.begin();
	    delay( 200 );
// 	    setupAccelleroTWI();
	    setupTWIdevices();
	}
#endif
#if MINIBEE_ENABLE_SHT == 1
	if ( shtOn ){
	    setupSHT();
	}
#endif
	// no need to setup ping
// 	if ( pingOn ){
// 	    setupPing();
// 	}
	
	uint8_t confSize = 9;
	char * configInfo = (char*)malloc( sizeof(char) * (19*2 + confSize) );
	configInfo[0] = node_id;
	configInfo[1] = config_id;
	configInfo[2] = samplesPerMsg;
	configInfo[3] = (uint8_t) (smpInterval/256);
	configInfo[4] = (uint8_t) (smpInterval%256);
	configInfo[5] = datasize;
	configInfo[6] = datasizeout;
	configInfo[7] = customInputs;
	configInfo[8] = customDataSize;
	for ( i=0; i<NRPINS; i++){
	  if ( custom_pin[i] ){
	    configInfo[confSize] = i;
	    configInfo[confSize+1] = custom_size[i];
	    confSize += 2;
	  }
	}
	send( N_CONF, configInfo, confSize );
	free( configInfo );

	outMessage = (char*)malloc( sizeof(char) * (datasize + 2 ) );
	data = outMessage + 2*sizeof(char); // not sure if this is correct... test!!

}

#if MINIBEE_ENABLE_TWI == 1
bool MBSerial::getFlagTWI(void) { 
	return twiOn;
}

void MBSerial::setupTWIdevices(void){
	for(i = 0;i < nr_twi_devices; i++ ){
	  switch( twi_devices[i] ){
#if MINIBEE_ENABLE_TWI_ADXL == 1
	      case TWI_ADXL345:
		accelADXL = (ADXL345*) malloc( sizeof( ADXL345 ) );
		accelADXL->init();
// 		accelADXL = (ADXL345*) new ADXL345();
		accelADXL->powerOn();
		accelADXL->setJustifyBit( false );
		accelADXL->setFullResBit( true );
		accelADXL->setRangeSetting( 2 ); // 2: 2g, 4: 4g, 8: 8g, 16: 16g
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

int MBSerial::readTWIdevices( int dboff ){
	int dbplus = 0;
	
	int accx, accy, accz;
	unsigned int accx2, accy2, accz2;
	float bmpT, bmpA, bmpP;
	unsigned long bmpConv;

	for(i = 0;i < nr_twi_devices; i++ ){
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
		accx2 = (unsigned int) (accx + 2048); // from twos complement signed int to unsigned int
		accy2 = (unsigned int) (accy + 2048); // from twos complement signed int to unsigned int
		accz2 = (unsigned int) (accz + 2048); // from twos complement signed int to unsigned int
		dataFromInt( accx, dboff + dbplus );
		dataFromInt( accy, dboff + dbplus + 2 );
		dataFromInt( accz, dboff + dbplus + 4 );
		dbplus += 6;
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
bool MBSerial::getFlagSHT(void) { 
    return shtOn;
}

uint8_t* MBSerial::getPinSHT(void) {
	return sht_pins; 
}

void MBSerial::setupSHT() {
	pinMode(sht_pins[0], OUTPUT);
	digitalWrite(sht_pins[0], HIGH);     
	pinMode(sht_pins[1], OUTPUT);   
	startSHT();
}

void MBSerial::startSHT(void) {
	pinMode(sht_pins[1], OUTPUT);
	digitalWrite(sht_pins[1], HIGH);
	digitalWrite(sht_pins[0], HIGH);    
	digitalWrite(sht_pins[1], LOW);
	digitalWrite(sht_pins[0], LOW);
	digitalWrite(sht_pins[0], HIGH);
	digitalWrite(sht_pins[1], HIGH);
	digitalWrite(sht_pins[0], LOW);
}

void MBSerial::resetSHT(void) {
	shiftOut(sht_pins[1], sht_pins[0], LSBFIRST, 0xff);
	shiftOut(sht_pins[1], sht_pins[0], LSBFIRST, 0xff);
	startSHT();
}

void MBSerial::softResetSHT(void) {
	resetSHT();
	ioSHT = SHT_RST_CMD;
	ackSHT = 1;
	writeByteSHT();
	delay(15);
}

void MBSerial::waitSHT(void) {
	delay(5);
	int j = 0;
	while(j < 600) {
		if(digitalRead(sht_pins[1]) == 0) j = 2600;
		delay(1);
		j++;
	}
}

void MBSerial::measureSHT(int cmd) {
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

void MBSerial::readByteSHT(void) {
	ioSHT = shiftInSHT();
	digitalWrite(sht_pins[1], ackSHT);
	pinMode(sht_pins[1], OUTPUT);
	digitalWrite(sht_pins[0], HIGH);
	digitalWrite(sht_pins[0], LOW);
	pinMode(sht_pins[1], INPUT);
	digitalWrite(sht_pins[1], LOW);
}

void MBSerial::writeByteSHT(void) {
	pinMode(sht_pins[1], OUTPUT);
	shiftOut(sht_pins[1], sht_pins[0], MSBFIRST, ioSHT);
	pinMode(sht_pins[1], INPUT);
	digitalWrite(sht_pins[1], LOW);
	digitalWrite(sht_pins[0], LOW);
	digitalWrite(sht_pins[0], HIGH);
	ackSHT = digitalRead(sht_pins[1]);
	digitalWrite(sht_pins[0], LOW);
}

int MBSerial::getStatusSHT(void) {
	softResetSHT();
	startSHT();
	ioSHT = SHT_R_STAT;	//R_STATUS

	writeByteSHT();
	waitSHT();
	ackSHT = 1;

	readByteSHT();
	return ioSHT;
}

int MBSerial::shiftInSHT(void) {
	int cwt = 0;
	for(mask = 128;mask >= 1;mask >>= 1) {
		digitalWrite(sht_pins[0], HIGH);
		cwt = cwt + mask * digitalRead(sht_pins[1]);
		digitalWrite(sht_pins[0], LOW);
	}
	return cwt;
}
#endif

#if MINIBEE_ENABLE_PING == 1
//PING
bool MBSerial::getFlagPing(void) { 
  return pingOn;
} 

uint8_t MBSerial::getPinPing(void) { 
	return ping_pin; 
}

int MBSerial::readPing(void) {
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

// //**** EVENTS ****//
// void MBSerial::setupDigitalEvent(void (*event)(int, int)) {
// 	dEvent = event;
// 	
// 	//set up digital interrupts 
// 	EICRA = (1 << ISC10) | (1 << ISC00);	
// 	PCICR = (1 << PCIE0);
// 	PCMSK0 = (1 << PCINT0);
// }
// 
// void MBSerial::digitalUpdate(int pin, int status) {
// 	(*dEvent)(pin, status);
// }
// 
// void PCINT0_vect(void) {
// 	Bee.digitalUpdate(0, PINB & (1 << PINB0));
// }

//serial rx event
/*void MBSerial::attachSerialEvent(void (*event)(void)) {
	//usart registers
	UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0); //turn on rx + tx and enable interrupts
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);	//set frame format (81NN)
	UBRR0L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low char of the UBRR register 
	UBRR0H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high char of the UBRR register
}

void USART_RX_vect(void) {
	
}*/

//**** REST OF THE EVENTS ****//