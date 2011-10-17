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



MiniBee_API::MiniBee_API(){
  status = STARTING;
  msg_type = S_NO_MSG;
  
  msg_id_send = 0;
  node_id = 0;
  config_id = 0;

  datacount = 0;

  prev_id_msg = 0;
}

void MiniBee_API::setup( long baud_rate ) {
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

void MiniBee_API::loopStep(){
  switch( status ){
    case STARTING:
      delay( 100 );
      break;
    case WAITFORCONFIG:
      if ( actcount == 0 ){ // send the wait message every 10 seconds
	msg_id_send++;
	msg_id_send = msg_id_send%256;
	sendTx16( N_WAIT, configInfo, 2 );
      }
      delay( 100 );
      break;
    case WAITFORHOST:
      if ( actcount == 0 ){ // send the serial number every 10 seconds
	sendXBeeSerial();
      }
      delay( 100 );
      break;
    case SENSING: // read sensors:
      datacount = readSensors( datacount );
      if ( curSample >= samplesPerMsg ){
	sendData();
	curSample = 0;
	datacount = 0;
      }
      delay( smpInterval );
      break;
    case ACTING:
      if ( actcount == 0 ){ // send an I'm active message every 100 smpIntervals
	sendActive();
      }
      delay( smpInterval );
      break;
    case PAUSING:
      if ( actcount == 0 ){ // send an I'm active message every 100 smpIntervals
	sendPaused();
      }
      delay( 500 );
      break;
  }
  actcount++;
  actcount = actcount%100;
  
  readXBeePacket();
}


uint8_t MiniBee_API::readSensors( uint8_t db ){
/*    unsigned int value;
    // read analog sensors
    for ( i = 0; i < ANAPINS; i++ ){
	if ( analog_in[i] ){
	    if ( analog_precision[i] ){
		value = analogRead( anapin_read_ids[i] );
		dataFromInt( value, db );
		db += 2;
	    } else {
		data[db] = analogRead( anapin_read_ids[i] )/4;
		db++;      
	    }
	}
    }
    
//     char diginfo[NRPINS];
    
    // read digital sensors
    for ( i = 0; i < NRPINS; i++ ){
//       diginfo[i] = (char) digital_in[i];
	if ( digital_in[i] ){
	//TODO this can be done way more clever by shifting the results into 3 bytes, resulting in shorter messages to be sent.
	    data[db] = digitalRead( pin_ids[i] );
	    db++;
	}
    }

//     send( N_INFO, diginfo, NRPINS );

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
//     datacount = db;*/
    return db;
}

void MiniBee_API::sendData(void){
    msg_id_send++;
    msg_id_send = msg_id_send%256;
//     outMessage[0] = node_id;
//     outMessage[1] = msg_id_send;
//     for ( i=0; i < datacount; i++ ){
// 	outMessage[i+2] = data[i];
//     }
    sendTx16( N_DATA, outMessage, datacount );
}

void MiniBee_API::sendActive(void){
    msg_id_send++;
    msg_id_send = msg_id_send%256;
//     outMessage[0] = node_id;
//     outMessage[1] = msg_id_send;
// //     for ( i=0; i < datacount; i++ ){
// // 	outMessage[i+2] = data[i];
// //     }
    sendTx16( N_ACTIVE, outMessage, 0 );
}

void MiniBee_API::sendPaused(void){
    msg_id_send++;
    msg_id_send = msg_id_send%256;
//     outMessage[0] = node_id;
//     outMessage[1] = msg_id_send;
// //     for ( i=0; i < datacount; i++ ){
// // 	outMessage[i+2] = data[i];
// //     }
    sendTx16( N_PAUSED, outMessage, 0 );
}

void MiniBee_API::readXBeePacket(){
  uint8_t option = 0;
//   uint8_t *data;
  uint8_t datasize = 0;
  uint8_t recvMsgType;
  
  xbee.readPacket();
    
  if (xbee.getResponse().isAvailable()) {// got something
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      // got a rx packet
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
	xbee.getResponse().getRx16Response(rx16);
        option = rx16.getOption();
	datasize = rx16.getDataLength();
        data = rx16.getData();
	recvMsgType = rx16.getData(0);
      } else {
        xbee.getResponse().getRx64Response(rx64);
	option = rx64.getOption();
	datasize = rx64.getDataLength();
        data = rx64.getData();
	recvMsgType = rx64.getData(0);
      }
      routeMsg( recvMsgType, data, datasize );
      // TODO check option, rssi bytes    
      flashLed(STATUS_LED, 1, 10);
    } else { // not something we were expecting
      flashLed(STATUS_LED, 1, 25);    
    }
  }
//   sendTx16( N_INFO, data, datasize );
}

bool MiniBee_API::checkIDMsg( uint8_t mid ){
  bool res = ( mid != prev_id_msg);
  prev_id_msg = mid;
  return res;
}

void MiniBee_API::routeMsg(uint8_t type, uint8_t *msg, uint8_t size) {

  // msg[0] = type
  // msg[1] = node id
  // msg[2] = msg id
  // msg[3] = msg specific data
  
//   if ( loopback ){
    msg_id_send++;
    msg_id_send = msg_id_send%256;
    sendTx16( 'x', msg, size );
    
    
//   }
    switch(type) {
      case S_ANN:
// 	if ( remoteConfig ){
	  sendXBeeSerial();
	  status = WAITFORHOST;
// 	}
	break;
      case S_QUIT:
// 	if ( remoteConfig ){
	  status = WAITFORHOST;
	  //do something to stop doing anything
// 	}
	break;
      case S_ID:
// 	if ( remoteConfig ){
// 	  if ( checkIDMsg( msg[2] ) ){ 
// 	    node_id = msg[1];
	    bool serialCorrect = true;
	    for ( uint8_t j=0; j<8; j++ ){
		if ( serial[j] != msg[j+3] ){ // serial is msg[3:10]
		  serialCorrect = false;
		}
	    }
// 	    if ( serialCorrect ){
// 	      node_id = rx16.getData( 1 );
// 	      config_id = rx16.getData( 2 );
// 	      setID( msg[1] ); // node id is msg[1]
// 	      if ( size == 8 ){
// 		config_id = msg[11]; // config id is msg[8]
		status = WAITFORCONFIG;
//  		configInfo[0] = node_id;
//  		configInfo[1] = config_id;
// 		configInfo[0] = rx16.getData( 1 );
// 		configInfo[1] = rx16.getData( 11 );
// 		configInfo[0] = rx16.getDataOffset();
// 		configInfo[1] = rx16.getDataLength();
		configInfo[0] = msg[0];
		configInfo[1] = size;
		
		sendTx16( N_WAIT, configInfo, 2 );
// 		sendTx16( 'x', serial, 8 );
// 	      } else if ( size < 8 ) { // no new config
// 		readConfig();
//   		status = SENSING;
// 	      }
// 	    }
// 	  }
	break;
    }
// }
}

void MiniBee_API::setID( uint8_t id ){
  node_id = id;
//   configInfo[0] = node_id;
//   configInfo[1] = config_id;
}

uint8_t MiniBee_API::getId(void) { 
  return node_id;
}


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

  serial = (uint8_t *)malloc( 8 * sizeof( uint8_t ) );
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
	  flashLed(STATUS_LED, 10, 100);
        }
      } else { // error 
            flashLed(STATUS_LED, 3, 500);
//         atResponse.getStatus();
      }
    } else { // not an AT response
      flashLed(STATUS_LED, 4, 500);
//       xbee.getResponse().getApiId();
    }   
  } else { // time out   // at command failed, there may be an error code returned
    if (xbee.getResponse().isError()) {
      flashLed(STATUS_LED, 5, 700);
//       xbee.getResponse().getErrorCode();
    } else { // no response at all
      flashLed(STATUS_LED, 5, 1000);
    }
  }
  return response;
}

void MiniBee_API::sendXBeeSerial(){
  sendTx16( N_SER, serial, 8 );  
}


void MiniBee_API::sendTx16( char type, uint8_t* data, uint8_t length ){
  payload[0] = (uint8_t) type;
  payload[1] = node_id;
  payload[2] = msg_id_send;
  for ( uint8_t i=0; i<length; i++ ){
    payload[i+3] = data[i];
  }
  
  txs16.setPayloadLength( length + 3 );
//   txs16.setPayload( payload );
  
  xbee.send(txs16);
  flashLed(STATUS_LED, 1, 100);

  // after sending a tx request, we expect a status response
  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) { // got a response!
	// should be a znet tx status            	
	if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
	  xbee.getResponse().getTxStatusResponse(txStatus);
		
	  // get the delivery status, the fifth byte
	  if (txStatus.getStatus() == SUCCESS) {
		// success.  time to celebrate
		flashLed(STATUS_LED, 5, 50);
	  } else {
		// the remote XBee did not receive our packet. is it powered on?
		flashLed(STATUS_LED, 3, 500);
	  }
	}      
    } else {
      // local XBee did not provide a timely TX Status Response -- should not happen
      flashLed(STATUS_LED, 2, 50);
    }
}

void MiniBee_API::flashLed(int pin, int times, int wait) {  
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      
      if (i + 1 < times) {
        delay(wait);
      }
    }
}
