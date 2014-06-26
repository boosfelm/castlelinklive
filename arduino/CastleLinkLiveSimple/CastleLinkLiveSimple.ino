/*****************************************************************************
 *  CastleLinkLiveSimple
 *  Copyright (C) 2012  Matteo Piscitelli
 *  E-mail: matteo@picciux.it
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  For further info, check http://code.google.com/p/castlelinklive4arduino/
 *
 *  SVN $Id: CastleLinkLiveSimple.pde 35 2012-10-15 17:29:44Z picciux@gmail.com $
 *****************************************************************************
 *  SAFETY NOTICE
 *  Always keep in mind that an electric motor can be dangerous for you, for 
 *  people and for things. It can start at any time if there is power. 
 *  Castle Creations ESC are very good ones, and have many security
 *  strategies to avoid accidental and unwanted motor start.
 *  This program and CastleLinkLive library also try to keep things as safe as
 *  possible, but using them together with an Arduino (or similar) board 
 *  connected to an electric power system adds another possible point of 
 *  failure to your motor control chain.
 *  So please stay always on the safe side. If you have any doubts, ask
 *  other modelers to help.
 *  It's your responsibility to keep things safe. Developers of this software
 *  can't be considered liable for any possible damage will result from its use.
 ******************************************************************************
 * CastleLinkLiveSimple
 * is a program using CastleLinkLive library to implement
 * a simple terminal monitor for Castle ESCs.
 *******************************************************/

#include "CastleLinkLiveSimple.h"
#include "CastleLinkLive.h"
#include <Wire.h>

#define SERIAL_BUFSIZE  10

/****************** CONFIGS ***********************/
#define SERIAL_BAUD_RATE            9600
//#define SOFTWARE_THROTTLE               1 
#define THROTTLE_IN_PIN_1               12
#define THROTTLE_IN_PIN_2                7//Must not be on same port

#define nESC  2

// initializing throttle value for software generated throttle.
// 50 (half-throttle) is the safe value: Castle ESCs will not arm
// until they see idle.
uint8_t throttle = 50;

// buffer to read serial commands
char buffer[SERIAL_BUFSIZE];
int bufcount = 0;

// print ESC data every second
int printDelay = 1000;

// setThrottle (in software throttle mode) every 20ms (average
// RC equipment frequency)
int setThrottleDelay = 20;

// control variables
int loopCnt = 1;
boolean sendThrottle = true;
boolean autoGenThrottle = false;
boolean rawData = false;

/*
 * Prints out human-readable data from ESC
 */
void print_data(CASTLE_ESC_DATA *d) {
  Serial.println("************* DATA **************");
  Serial.print("Voltage: .......... ");
  Serial.println(d->voltage);
  Serial.print("Ripple Voltage: ... ");
  Serial.println(d->rippleVoltage);
  Serial.print("Current: .......... ");
  Serial.println(d->current);
  Serial.print("Throttle: ......... ");
  Serial.println(d->throttle);
  Serial.print("Output Power: ..... ");
  Serial.println(d->outputPower);
  Serial.print("RPM: .............. ");
  Serial.println(d->RPM);
  Serial.print("BEC Voltage: ...... ");
  Serial.println(d->BECvoltage);
  Serial.print("BEC Current: ...... ");
  Serial.println(d->BECcurrent);
  Serial.print("Temperature: ...... ");
  Serial.println(d->temperature);
  Serial.println("************** EOD **************");
}

/*
 * Prints out raw data (ticks count) from ESC
 */
void print_data(CASTLE_RAW_DATA *d) {
  Serial.println("************* DATA TICKS **************");
  Serial.print("Reference ......... ");
  Serial.println(d->ticks[FRAME_REFERENCE]);
  Serial.print("Voltage: .......... ");
  Serial.println(d->ticks[FRAME_VOLTAGE]);
  Serial.print("Ripple Voltage: ... ");
  Serial.println(d->ticks[FRAME_RIPPLE_VOLTAGE]);
  Serial.print("Current: .......... ");
  Serial.println(d->ticks[FRAME_CURRENT]);
  Serial.print("Throttle: ......... ");
  Serial.println(d->ticks[FRAME_THROTTLE]);
  Serial.print("Output Power: ..... ");
  Serial.println(d->ticks[FRAME_OUTPUT_POWER]);
  Serial.print("RPM: .............. ");
  Serial.println(d->ticks[FRAME_RPM]);
  Serial.print("BEC Voltage: ...... ");
  Serial.println(d->ticks[FRAME_BEC_VOLTAGE]);
  Serial.print("BEC Current: ...... ");
  Serial.println(d->ticks[FRAME_BEC_CURRENT]);
  Serial.print("Temperature 1: .... ");
  Serial.println(d->ticks[FRAME_TEMP1]);
  Serial.print("Temperature 2: .... ");
  Serial.println(d->ticks[FRAME_TEMP2]);
  Serial.println("************** EOD **************");
}
  CASTLE_ESC_DATA escHR0;
  CASTLE_ESC_DATA escHR1;
  byte *i2cdata = new byte[48];


/*
 * event-handling funtion to attach to CastleLinkLive to be notified
 * when throttle signal failure (or recovery) is detected
 */
void throttlePresence(boolean presence) {
  if (! presence)
    Serial.println("No more throttle signal!");
  else
    Serial.println("We have throttle signal again!");
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.print("CastleLinkLive-SimpleSerialMonitor is starting... ");
  Wire.begin(2);
  Wire.onRequest(requestEvent); // register event



  
  CastleLinkLive.init();
  
  //attaching our throttleFailure function to CastleLinkLive to be
  //notified in case of throttle failure/recovery
  CastleLinkLive.attachThrottlePresenceHandler(throttlePresence);

  /* 
   * Starting CastleLinkLive: 
   * first parameter is number of ESC(s) to connect to 
   * (max 2 on ATmega 168/328 and 1280/2560, max 1 on ATmega8),
   * second parameter is Arduino pin to read throttle signal from:
   * if unspecified, CastleLinkLive will auto-generate throttle signal
   * for you. Beware that, when software generating throttle, as a safety
   * measure, CastleLinkLive expects throttle to be continuosly set 
   * (setThrottle() function) at a rate faster than ~1 Hz (1 time per second); 
   * failure to do so will result in CastleLinkLive to stop generating 
   * throttle signal and raising throttle failure event until setThrottle is
   * called again.
   */ 
#ifdef SOFTWARE_THROTTLE
  autoGenThrottle = true;
  if (! CastleLinkLive.begin(nESC)) {
    
#else
  #ifdef THROTTLE_IN_PIN_2
      if (! CastleLinkLive.begin(nESC, THROTTLE_IN_PIN_1, THROTTLE_IN_PIN_2)) {
        
        #else
            if (! CastleLinkLive.begin(nESC, THROTTLE_IN_PIN_1)) {
  #endif
#endif
    Serial.println("Initialization error!");
  } 
  
  Serial.println("GO!");
  
}

void loop() {
  delay(setThrottleDelay);
  
  if (CastleLinkLive.getData(0, &escHR0) && CastleLinkLive.getData(1, &escHR1))
  {
  float data_float[12]={escHR0.voltage, escHR0.current, escHR0.RPM*2.0f/24.0f, escHR0.BECvoltage, escHR0.BECcurrent, escHR0.temperature, escHR1.voltage, escHR1.current, escHR1.RPM*2.0f/24.0f, escHR1.BECvoltage, escHR1.BECcurrent, escHR1.temperature};
  i2cdata = (byte *)data_float; 
                       // as expected by master
                       Serial.println(escHR0.RPM);
  }
  else 
{
float data_float[12]={0,0,0,0,0,0,0,0,0,0,0,0};
i2cdata = (byte *)data_float; 
                       // as expected by master
                       Serial.println("bad");
}
  
}
void requestEvent()
{

   Wire.write(i2cdata,48); // respond with message of 6 bytes
  
}
  


