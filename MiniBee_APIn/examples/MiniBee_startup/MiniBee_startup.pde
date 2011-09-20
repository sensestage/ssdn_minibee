/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file is part of XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <XBee.h>

/*
This example is for Series 1 XBee
Sends a TX16 or TX64 request with the value of analogRead(pin5) and checks the status response for success
Note: In my testing it took about 15 seconds for the XBee to start reporting success, so I've added a startup delay
*/

XBee xbee = XBee();


int statusLed = 4;
int errorLed = 9;

void flashLed(int pin, int times, int wait) {
    
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      
      if (i + 1 < times) {
        delay(wait);
      }
    }
}

void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  
  pinMode( 2, OUTPUT );
  digitalWrite( 2, 0 );
  
  xbee.begin(57600);
  
  delay( 500 );
  
  readXBeeSerial();
    
  sendXBeeSerial();
}


uint8_t *serial;

// uint8_t shCmd[] = {'S','H'};  // serial high
uint8_t slCmd[] = {'S','L'};  // serial low

AtCommandRequest atRequest = AtCommandRequest(slCmd);
AtCommandResponse atResponse = AtCommandResponse();


void readXBeeSerial() {
//   uint8_t *response;// = (char *)malloc(sizeof(char)*6);
  uint8_t *response2;// = (char *)malloc(sizeof(char)*8);

  // get SH
//   atRequest.setCommand(shCmd);  
//   response = sendAtCommand();

    // set command to SL
  atRequest.setCommand(slCmd);  
  response2 = sendAtCommand();

//   serial = (uint8_t *)malloc( sizeof(response2) + sizeof( reponse ) );
  serial = (uint8_t *)malloc( sizeof(response2) );
  
   for ( int i = 0; i<4; i++ ){
//      serial[i] = response[i];
//      serial[i+4] = response2[i];
     serial[i] = response2[i];
   }

//   free( response );
  free( response2 );
}

uint8_t* sendAtCommand() {
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
	  flashLed(statusLed, 10, 100);
        }
      } else { // error 
            flashLed(errorLed, 3, 500);
//         atResponse.getStatus();
      }
    } else { // not an AT response
      flashLed(errorLed, 4, 500);
//       xbee.getResponse().getApiId();
    }   
  } else { // time out   // at command failed, there may be an error code returned
    if (xbee.getResponse().isError()) {
      flashLed(errorLed, 5, 700);
//       xbee.getResponse().getErrorCode();
    } else { // no response at all
      flashLed(errorLed, 5, 1000);
    }
  }
  return response;
}

#define MAX_PAYLOAD 63
#define COORD_ADDR 0x0000

uint8_t payload[MAX_PAYLOAD+1]; //, 0x40,0x6A,0x69,0xAB };

// 16-bit addressing: Enter address of remote XBee, typically the coordinator
Tx16Request txs16 = Tx16Request(COORD_ADDR, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

void sendTx16( char type, uint8_t* data, uint8_t length ){
  payload[0] = (uint8_t) type;
  for ( uint8_t i=0; i<length; i++ ){
    payload[i+1] = data[i];
  }
  
  txs16.setPayloadLength( length + 1 );
//   txs16.setPayload( payload );
  
  xbee.send(txs16);
  flashLed(statusLed, 1, 100);

  // after sending a tx request, we expect a status response
  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) { // got a response!
	// should be a znet tx status            	
	if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
	  xbee.getResponse().getTxStatusResponse(txStatus);
		
	  // get the delivery status, the fifth byte
	  if (txStatus.getStatus() == SUCCESS) {
		// success.  time to celebrate
		flashLed(statusLed, 5, 50);
	  } else {
		// the remote XBee did not receive our packet. is it powered on?
		flashLed(errorLed, 3, 500);
	  }
	}      
    } else {
      // local XBee did not provide a timely TX Status Response -- should not happen
      flashLed(errorLed, 2, 50);
    }
}

void sendXBeeSerial(){
  sendTx16( 's', serial, 4 );  
}

uint8_t data[5] = { 'h', 'e', 'l', 'l', 'o' };

void sendOtherData(){
  sendTx16( 'd', data, 5 );  
}


uint8_t actcount = 0;

void loop() {
//    sendXBeeSerial();
//    delay( 500 );
//    sendOtherData();

   actcount++;
   sendTx16( 'i', &actcount, 1 );

  delay(50);
}
