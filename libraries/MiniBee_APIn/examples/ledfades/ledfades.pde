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


/// in the header file of the MiniBee you can disable some options to save
/// space on the MiniBee. If you don't the board may not work as it runs
/// out of RAM.

/// Wire needs to be included if TWI is enaleds[2]

#include <Wire.h>

#include <LIS302DL.h>
#include <ADXL345.h>
#include <TMP102.h>
#include <BMP085.h>
#include <HMC5843.h>

#include <XBee.h>
#include <MiniBee_APIn.h>


#include <TimerOne.h>

#define BLED_PIN 9
#define GLED_PIN 10
#define RLED_PIN 11

enum {
  OFF,
  FADEIN,
  HOLD,
  FADEOUT
};

struct ledstruct {
  boolean onoff;
  boolean pulse;

  uint16_t duration;
      
  uint8_t stage;

  uint16_t fadein;
  uint16_t hold;
  uint16_t fadeout;
  
  uint16_t pulsedur;

  uint16_t time;      // current time
  uint8_t pulsetime; // current pulse time
  
  uint8_t pulseduty;
  uint8_t currentvalue;
    
  uint16_t intensity;

  uint16_t intensity_increment; // or time increment?
  uint16_t current_intensity;
} leds[3];


MiniBee_API Bee = MiniBee_API();


/// this will be our parser for the custom messages we will send:
/// msg[0] and msg[1] will be msg type ('E') and message ID
/// the remainder are the actual contents of the message
/// if you want to send several kinds of messages, you can e.g.
/// switch based on msg[2] for message type

// leds[0] msg: 'L' on/off, pulse on/off, intensity (1), fadein (2), hold (2), fadeout (2), pulsetime (1) 
// servo msg: 'S' newposition (2), movetime (2)
void customMsgParser( uint8_t * msg, uint8_t size, uint16_t source ){
  switch( msg[2] ){
    case 'a':
	leds[0].intensity = msg[3]*256;
	leds[1].intensity = msg[4]*256;
	leds[2].intensity = msg[5]*256;
	leds[0].onoff = 1;
	leds[1].onoff = 1;
	leds[2].onoff = 1;
	
	leds[0].pulse = leds[2].pulse = leds[1].pulse = msg[6];
	leds[0].fadein = leds[2].fadein = leds[1].fadein = msg[7]*256 + msg[8];
	leds[0].hold = leds[2].hold = leds[1].hold = msg[9]*256 + msg[10];
	leds[0].fadeout = leds[1].fadeout = leds[2].fadeout = msg[11]*256 + msg[12];
	leds[0].pulsedur = leds[1].pulsedur = leds[2].pulsedur = msg[13];
	leds[0].pulseduty = leds[1].pulseduty = leds[2].pulseduty = msg[14];

	leds[0].duration = leds[1].duration = leds[2].duration = leds[0].fadein + leds[0].hold + leds[0].fadeout;
	
	leds[0].intensity_increment = int( (float) leds[0].intensity / (float) leds[0].fadein );
	leds[0].stage = FADEIN;
	leds[0].time = 0;
	leds[0].pulsetime = 0;

	leds[1].intensity_increment = int( (float) leds[1].intensity / (float) leds[1].fadein );
	leds[1].stage = FADEIN;
	leds[1].time = 0;
	leds[1].pulsetime = 0;
	
	leds[2].intensity_increment = int( (float) leds[2].intensity / (float) leds[2].fadein );
	leds[2].stage = FADEIN;
	leds[2].time = 0;
	leds[2].pulsetime = 0;
      break;
    case 'r':
	leds[0].onoff = msg[3];
	leds[0].pulse = msg[4];
	leds[0].intensity = msg[5]*256;
	leds[0].fadein = msg[6]*256 + msg[7];
	leds[0].hold = msg[8]*256 + msg[9];
	leds[0].fadeout = msg[10]*256 + msg[11];
	leds[0].pulsedur = msg[12];
	leds[0].pulseduty = msg[13];
	leds[0].duration = leds[0].fadein + leds[0].hold + leds[0].fadeout;
	leds[0].intensity_increment = int( (float) leds[0].intensity / (float) leds[0].fadein );
	leds[0].stage = FADEIN;
	leds[0].time = 0;
	leds[0].pulsetime = 0;
      break;
    case 'g':
	leds[1].onoff = msg[3];
	leds[1].pulse = msg[4];
	leds[1].intensity = msg[5]*256;
	leds[1].fadein = msg[6]*256 + msg[7];
	leds[1].hold = msg[8]*256 + msg[9];
	leds[1].fadeout = msg[10]*256 + msg[11];
	leds[1].pulsedur = msg[12];
	leds[1].pulseduty = msg[13];
	leds[1].duration = leds[1].fadein + leds[1].hold + leds[1].fadeout;
	leds[1].intensity_increment = int( (float) leds[1].intensity / (float) leds[1].fadein );
	leds[1].stage = FADEIN;
	leds[1].time = 0;
	leds[1].pulsetime = 0;
      break;
    case 'b':
	leds[2].onoff = msg[3];
	leds[2].pulse = msg[4];
	leds[2].intensity = msg[5]*256;
	leds[2].fadein = msg[6]*256 + msg[7];
	leds[2].hold = msg[8]*256 + msg[9];
	leds[2].fadeout = msg[10]*256 + msg[11];
	leds[2].pulsedur = msg[12];
	leds[2].pulseduty = msg[13];
	leds[2].duration = leds[2].fadein + leds[2].hold + leds[2].fadeout;
	leds[2].intensity_increment = int( (float) leds[2].intensity / (float) leds[2].fadein );
	leds[2].stage = FADEIN;
	leds[2].time = 0;
	leds[2].pulsetime = 0;
      break;
  } 
}

volatile boolean clockticked = false;

volatile uint8_t beecnt = 0;
uint8_t beecnt2 = 0;

uint8_t ledstatus[3];
uint16_t statustoserver[3];

void callback()
{
  clockticked = true;
  beecnt++;
  leds[0].time++;
  leds[0].pulsetime++;
  leds[1].time++;
  leds[1].pulsetime++;
  leds[2].time++;
  leds[2].pulsetime++;
}

void setup() {  
  Bee.setup(57600, 'D' ); // arguments are the baudrate, and the board revision
  
  Bee.setCustomPin( RLED_PIN, 0 );
  Bee.setCustomPin( GLED_PIN, 0 );
  Bee.setCustomPin( BLED_PIN, 0 );
  
  Bee.setCustomInput( 3, 1 ); // laser current_intensity, servo oldvalue
  Bee.setCustomInput( 3, 2 ); // laser current_intensity, servo oldvalue
    
  // set the custom message function
  Bee.setCustomCall( &customMsgParser );
  
  analogWrite( RLED_PIN, 255 );
  analogWrite( GLED_PIN, 255 );
  analogWrite( BLED_PIN, 255 );
  
  leds[0].onoff = false;
  leds[0].stage = OFF;
  leds[0].pulse = false;
  leds[0].time = 0;

  leds[1].onoff = false;
  leds[1].stage = OFF;
  leds[1].pulse = false;
  leds[1].time = 0;

  leds[2].onoff = false;
  leds[2].stage = OFF;
  leds[2].pulse = false;
  leds[2].time = 0;

  noInterrupts();

   Timer1.initialize(1000);         // initialize timer1, and set a 1/2 second period
   Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  // TODO: check the clock cycle!
 // Set up PWM  with Clock/256 (i.e.  31.25kHz on Arduino 16MHz;
 // and  phase accurate
//   TCCR2A = _BV(COM2B1) | _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  //   TCCR2A = _BV(COM2A1) | _BV(WGM20);
//   TCCR2B = _BV(CS20);
//   TIMSK2 = _BV(TOIE2);
//   sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt
  
   interrupts();
}

void calc_led( uint8_t i ){
  if ( leds[i].onoff ){ // if laser on:
    if ( leds[i].time > leds[i].duration ){
	leds[i].stage = OFF;
    }
    // envelope of the laser:
    switch( leds[i].stage ){
      case OFF:
	leds[i].time = 0;
	leds[i].current_intensity = 0;
	leds[i].intensity_increment = int( (float) leds[i].intensity / (float) leds[i].fadein );
	break;
      case FADEIN:
	leds[i].current_intensity += leds[i].intensity_increment;
	if ( leds[i].time > leds[i].fadein ){
	    leds[i].stage = HOLD;
	}
	break;
      case HOLD:
	leds[i].current_intensity = leds[i].intensity;
	if ( leds[i].time > (leds[i].fadein + leds[i].hold) ){
	  leds[i].stage = FADEOUT;
	  leds[i].intensity_increment = int( (float) leds[i].intensity / (float) leds[i].fadeout );
	}
	break;
      case FADEOUT:
	leds[i].current_intensity -= leds[i].intensity_increment;
	if ( leds[i].time > leds[i].duration ){
	  leds[i].stage = OFF;
	}
	break;
    }
    if ( leds[i].pulse ){
      if ( leds[i].pulsetime < leds[i].pulseduty ){ // pulse duty on
	leds[i].currentvalue = (leds[i].current_intensity>>8);
      } else if ( leds[i].pulsetime < leds[i].pulsedur ) { // pulse duty off
	leds[i].currentvalue = 0;
      } else { // end of pulse, reset to start
	leds[i].currentvalue = 0;
	leds[i].pulsetime = 0;
      }
    } else {
      leds[i].currentvalue = (leds[i].current_intensity>>8);
    }
  } else {
    leds[i].stage = OFF; 
    leds[i].current_intensity = 0;
    leds[i].currentvalue = 0;
  }
}

void loop() { 
//   callback();
  while( !clockticked ){}
  clockticked = false;

  calc_led( 0 );
  calc_led( 1 );
  calc_led( 2 );
  
  analogWrite( RLED_PIN, 255 - leds[0].currentvalue );
  analogWrite( GLED_PIN, 255 - leds[1].currentvalue );
  analogWrite( BLED_PIN, 255 - leds[2].currentvalue );
  
  // do a loop step of the remaining firmware:
  if ( beecnt > 100 ){ // every fifty milliseconds
    beecnt = 0;
    beecnt2++;
    
    if ( beecnt2 > 10 ){
      beecnt2 = 0;
      ledstatus[0]= (100 * leds[0].onoff) + (10 * leds[0].pulse) + leds[0].stage;
      ledstatus[1] = (100 * leds[1].onoff) + (10 * leds[1].pulse) + leds[1].stage;
      ledstatus[2] = (100 * leds[2].onoff) + (10 * leds[2].pulse) + leds[2].stage;
      statustoserver[0] = leds[0].current_intensity;
      statustoserver[1] = leds[1].current_intensity;
      statustoserver[2] = leds[2].current_intensity;
      Bee.addCustomData( ledstatus, 3 );
      Bee.addCustomData( statustoserver, 3 );
      Bee.loopStep( false );
    } else {
      Bee.loopReadOnly();
    }
  }
//   delay(1);
}
