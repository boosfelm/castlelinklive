#include "test_interrupt.h"
#include <Wire.h>
int counter_overflow;
bool is_in_pwm;
bool is_waiting_for_tick;
uint8_t TIMER_START = 0b10; //with presacler 8
uint8_t TIMER_STOP = ! TIMER_START;
uint8_t ENABLE_OVF = (1 << TOIE2);
uint8_t DISABLE_OVF = !ENABLE_OVF;
uint8_t RISING_EDGE = (1 << ISC11) | (1 << ISC10); //0b00001100
uint8_t FALLING_EDGE = (!RISING_EDGE) | (1 << ISC11); // 0b11111011 only works if before was rising edge

typedef struct castle_priv_data_struct {
   unsigned int ticks[DATA_FRAME_CNT];

   int frameIdx;
   uint8_t ticked;
} CASTLE_PRIV_DATA;

CASTLE_PRIV_DATA data;

volatile bool flag_ready;

  CASTLE_ESC_DATA escdata;
  CASTLE_RAW_DATA escraw;
  
 byte *i2cdata = new byte[48];

void setup()
{

  //init data structure
  data.frameIdx = FRAME_RESET;
  data.ticked = 0;
  for (int j = 0; j < DATA_FRAME_CNT; j++)
    data.ticks[j] = 0;
  //

    is_in_pwm = false;
    is_waiting_for_tick = false;
    pinMode(13, OUTPUT);
    pinMode(3, INPUT);
    Serial.begin(2400);
    cli();
    TIMSK2 = ENABLE_OVF;
    EICRA |= (1 << ISC11); //Falling Edge
    EIMSK |= (1 << INT1); //Enable interrupt via pin
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2B |= 0b010;    // 8 prescaler 0b010;
      Wire.begin(2);
  Wire.onRequest(requestEvent); // register event
    Serial.println("Start");
    sei();
}

void loop()
{
//  delay(400);

getData(0,&escdata);
getData(0,&escraw);


//Serial.println(escdata.RPM);
float data_float[12]={escdata.voltage, escdata.current, escdata.RPM*2.0f/24.0f, escdata.BECvoltage, escdata.BECcurrent, escdata.temperature, escdata.voltage, escdata.current, escdata.RPM*2.0f/24.0f, escdata.BECvoltage, escdata.BECcurrent, escdata.temperature};

  i2cdata = (byte *)data_float;
delay(20);


}

void requestEvent()
{

   Wire.write(i2cdata,48); // respond with message of 6 bytes
  
}


ISR(INT1_vect)
{

     unsigned int ovf = counter_overflow;

    unsigned int time = TCNT2;
        TCNT2 = 0;    //reset time

    counter_overflow = 0;
    
    time = time + (ovf<<8);    
    

    if (is_waiting_for_tick) {

      tick(time);
//        TIMSK2 &= DISABLE_OVF;
TCCR2B = 0;
        is_waiting_for_tick = false;


        return;
    }

//    TIMSK2 |= ENABLE_OVF;
TCCR2B = 0b10;

    if (is_in_pwm) {
        is_waiting_for_tick = true;
        is_in_pwm = false;
        EICRA &= FALLING_EDGE;
        
        return;
    }

}

ISR(TIMER2_OVF_vect)
{
        counter_overflow++;

    if (counter_overflow == 3 && digitalRead(3) == LOW) {
      
//            TIMSK2 &= DISABLE_OVF;
TCCR2B = 0;
            EICRA |= RISING_EDGE;
            is_in_pwm = true;
   
            counter_overflow = 0;
            return;
        }
    

    if (counter_overflow >= 28) {
        if (digitalRead(3) == HIGH && is_waiting_for_tick) {
//                   TIMSK2 &= DISABLE_OVF;
TCCR2B = 0;

          reset();
            is_waiting_for_tick = false;


        }

        counter_overflow = 0;
        return;
    }


}











// castle data timeout
void reset()
{
          CASTLE_PRIV_DATA *d = &data;

              d->frameIdx = FRAME_RESET;

//  Serial.println("reset");

        //if castle ESC ticked some data in, reset the ticked indicator for next cycle
        
            flag_ready = false;

        

}


inline void tick(unsigned int ticks) {

//Serial.println(ticks);

  if (ticks == 0) return; //timer was stopped


  CASTLE_PRIV_DATA *d = &data;
  
  d->frameIdx++;
  
  d->ticks[d->frameIdx] = ticks;
//  Serial.println(d->ticks[0]);

  d->ticked = true;


  if(d->frameIdx == DATA_FRAME_CNT -1)
  {
  flag_ready = true;
//  Serial.println("READY");
  }

}

uint8_t getData(uint8_t index, CASTLE_RAW_DATA *o)
{

  
  while (!flag_ready) { //wait for ISRs code to finish filling data structure
  }
EIMSK = 0;
memcpy(o, data.ticks, sizeof(unsigned int) * DATA_FRAME_CNT);
EIMSK  |= (1 << INT1);
  return true;
}


uint8_t getData( uint8_t index, CASTLE_ESC_DATA *o) {
  uint8_t whichTemp;

  CASTLE_RAW_DATA c;
  
  while (!flag_ready) { //wait for ISRs code to finish filling data structure
  }
  memcpy(&c, &(data), sizeof(unsigned int) * DATA_FRAME_CNT);

  
  whichTemp = CLL_GET_WHICH_TEMP(c);

  
  for (int f = 1; f < DATA_FRAME_CNT; f++) {
    float value = CLL_BASE_VALUE(
    		c.ticks[f],
    		c.ticks[FRAME_REFERENCE],
    		CLL_GET_OFFSET_TICKS(c)
    );

    switch(f) {
      case FRAME_VOLTAGE:
        o->voltage = CLL_CALC_VOLTAGE(value);
        break;
      case FRAME_RIPPLE_VOLTAGE:
        o->rippleVoltage = CLL_CALC_RIPPLE_VOLTAGE(value);
        break;
      case FRAME_CURRENT:
        o->current = CLL_CALC_CURRENT(value);
        break;
      case FRAME_THROTTLE:
        o->throttle = CLL_CALC_THROTTLE(value);
        break;
      case FRAME_OUTPUT_POWER:
        o->outputPower = CLL_CALC_OUTPUT_POWER(value);
        break;
      case FRAME_RPM:
        o->RPM = CLL_CALC_RPM(value);
        break;
      case FRAME_BEC_VOLTAGE:
        o->BECvoltage = CLL_CALC_BEC_VOLTAGE(value);
        break;
      case FRAME_BEC_CURRENT:
        o->BECcurrent = CLL_CALC_BEC_CURRENT(value);
        break;
      case FRAME_TEMP1:
        if (whichTemp == FRAME_TEMP1) o->temperature = CLL_CALC_TEMP1(value);
        break;
      case FRAME_TEMP2:
        if (whichTemp == FRAME_TEMP2) {
        	o->temperature = CLL_CALC_TEMP2(value);
        }
        break;
    }
    
  }
  
  return true;
}


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



