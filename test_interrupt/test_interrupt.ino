#include "test_interrupt.h"
#include <Wire.h>

//made volatile
volatile int counter_overflow[2];
volatile bool is_in_pwm[2];
volatile bool is_waiting_for_tick[2];

uint8_t RISING_EDGE1 = (1 << ISC11) | (1 << ISC10); //0b00001100
uint8_t FALLING_EDGE1 = (!RISING_EDGE1) | (1 << ISC11); // 0b11111011 only works if before was rising edge

uint8_t RISING_EDGE0 = (1 << ISC01) | (1 << ISC00); //0b00001100
uint8_t FALLING_EDGE0 = (!RISING_EDGE0) | (1 << ISC01); // 0b11111011 only works if before was rising edge

typedef struct castle_priv_data_struct {
    unsigned int ticks[DATA_FRAME_CNT];
    int frameIdx;
} CASTLE_PRIV_DATA;

CASTLE_PRIV_DATA data[2];

volatile bool flag_ready[2];

volatile int my_ctr=0;
volatile int ctr_data[3];
int loop_ctr = 0;


CASTLE_ESC_DATA escdata[2];

static int test_led_toggle = 1;

byte *i2cdata = new byte[48];

void setup()
{
    //init data structure
    for (int i = 0 ; i < 2; i++) {
        data[i].frameIdx = FRAME_RESET;

        for (int j = 0; j < DATA_FRAME_CNT; j++) {
            data[i].ticks[j] = 0;
        }

        is_in_pwm[i] = false;
        is_waiting_for_tick[i] = false;
        counter_overflow[i] = 0;
        flag_ready[i] = false;
    }

    pinMode(13, OUTPUT);//test led blink
    digitalWrite(13, LOW);
  
    pinMode(3, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    Serial.begin(2400);
    cli();
    TIMSK2 |= (1<<TOIE2); //Enable overflow
    
    //OCR1A = 256;
    ICR1 = 256;
    TIFR1 |= _BV(OCF1A);      // clear any pending interrupts; 
    TIMSK1 |=  _BV(OCIE1A) ;  // enable the output compare interrupt 
    EICRA |=  (1 << ISC11) | (1 << ISC01); //Falling Edge for both interrupt INT0 and interrupt INT1
    EIMSK |= (1 << INT1); //| (1 << INT0); //Enable interrupt via pin for INT0 and INT1

    //EICRA |=  (1 << ISC11);
    EIMSK |= (1 << INT0);

    TCCR2A = 0;
    TCCR2B = 0;
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR2B |= 0b010;    // 8 prescaler 0b010;
    TCCR1A |= 0b00;
    TCCR1B |= 0b11010;     // set prescaler of 8. Time for overflow = 65ms. Should never happen when connected to esc. //higher prescale slower clock, more time for overflow, more difficult to characterize pwm or tick etc based on overflow as you have done (magic numbers 3 and 28 for 8bit timer with prescale 8). Needs be adjusted for 16bit timer with no prescale.
    Wire.begin(2);
    Wire.onRequest(requestEvent); // register event
    Serial.println("Start");
    sei();
}

void loop()
{
    //  delay(400);
    if(getData(0, &escdata[0])){
      Serial.println(escdata[0].RPM);
      float data_float[12] = {escdata[0].voltage, escdata[0].current, escdata[0].RPM * 2.0f / 24.0f, escdata[0].BECvoltage, escdata[0].BECcurrent, escdata[0].temperature, escdata[0].voltage, escdata[0].current, escdata[0].RPM * 2.0f / 24.0f, escdata[0].BECvoltage, escdata[0].BECcurrent, escdata[0].temperature};
      i2cdata = (byte *)data_float;  
    }
    
    if(getData(1, &escdata[1])){
      Serial.println(escdata[1].RPM);
      float data_float[12] = {escdata[1].voltage, escdata[1].current, escdata[1].RPM * 2.0f / 24.0f, escdata[1].BECvoltage, escdata[1].BECcurrent, escdata[1].temperature, escdata[1].voltage, escdata[1].current, escdata[1].RPM * 2.0f / 24.0f, escdata[1].BECvoltage, escdata[1].BECcurrent, escdata[1].temperature};
      i2cdata = (byte *)data_float;  
    }
    delay(20);
//    getData(0, &escdata[1]);
    
/*
    Serial.println("H");
    ctr_data[loop_ctr]= my_ctr;
    loop_ctr++;
    if(loop_ctr == 3){
        loop_ctr = 0;
        Serial.println(ctr_data[0]);
        Serial.println(ctr_data[1]);
        Serial.println(ctr_data[2]);
        
    }
    */
        
}

void requestEvent()
{
    Wire.write(i2cdata, 48); // respond with message of 6 bytes
}


ISR(INT1_vect)
{
  digitalWrite(13,HIGH);

    unsigned int time = TCNT2;  
    unsigned int ovf = counter_overflow[1];
    
    TCNT2 = 0;    //reset time
    counter_overflow[1] = 0;
    time = time + (ovf << 8);

  //tick(time,1);//testing only should be 1

  

    if (is_waiting_for_tick[1]) {
        tick(time,1);
        digitalWrite(13,HIGH);
        //        TIMSK2 &= DISABLE_OVF;
        TCCR2B = 0; // prescale remover or timer clock stopped ? does it stop the clock or just remove prescale.
        is_waiting_for_tick[1] = false;
        return;
    }

    //    TIMSK2 |= ENABLE_OVF;
    TCCR2B = 0b10;//set prescale 8 (timer clock start or already running ? )

    if (is_in_pwm[1]) {
        is_waiting_for_tick[1] = true;
        is_in_pwm[1] = false;
        EICRA &= FALLING_EDGE1; // look for falling edge
        return;
    }
    
    
}


ISR(TIMER2_OVF_vect)
{
    counter_overflow[1]++;


  //  my_ctr = my_ctr + 1;
    

    //digitalWrite(13,HIGH);    
    
    if (counter_overflow[1] == 3 && digitalRead(3) == LOW) { //time taken = (2power8 = 256) x (prescale=8) / (clockfreq = 8Mhz) = 256 microsec per overflow. 3 overflow = 3*256 microsec = 0.7 ms low bottom we are in pwm 
        //            TIMSK2 &= DISABLE_OVF;
        TCCR2B = 0; //why ? stopping clock ?
        EICRA |= RISING_EDGE1;
        is_in_pwm[1] = true;
        counter_overflow[1] = 0;
        return;
    }

    if (counter_overflow[1] >= 28) { // no falling edge for over 7ms after rising edge
        if (digitalRead(3) == HIGH && is_waiting_for_tick[1]) {
            //                   TIMSK2 &= DISABLE_OVF;
            TCCR2B = 0; //stop timer clock
            reset(1);
            is_waiting_for_tick[1] = false;
        }

        counter_overflow[1] = 0;
        return;
    }
    
    
}


ISR(INT0_vect) 
{
  digitalWrite(13,LOW);
  unsigned int time = TCNT1;
  unsigned int ovf = counter_overflow[0];

    TCNT1 = 0;    //reset time
    counter_overflow[0] = 0;
    time = time + (ovf << 8);


    tick(time,0);
  
  /*

    if (is_waiting_for_tick[0]) {
        //digitalWrite(13, HIGH);
        tick(time,0);
        digitalWrite(13,LOW);
        
        //        TIMSK2 &= DISABLE_OVF;
        TCCR1B = 0;
        is_waiting_for_tick[0] = false;
        return;
    }

    //    TIMSK2 |= ENABLE_OVF;
    TCCR1B = 0b11010; 

    if (is_in_pwm[0]) {
        is_waiting_for_tick[0] = true;
        is_in_pwm[0] = false;
        EICRA &= FALLING_EDGE0;
        return;
    }
    
    */
}


ISR(TIMER1_COMPA_vect)
{
    counter_overflow[0]++;

  
  /*
  // my_ctr = my_ctr + 1;
    
    //unsigned int time = TCNT1;
    //TCNT1 = 0;
    //Serial.println(time);
    //digitalWrite(13, HIGH);
    //TCNT1 = 0;    //reset time. Time resets after over flow.


    if (counter_overflow[0] == 3 && digitalRead(2) == LOW) { //counter overflow 2 or 3 ??
        //            TIMSK2 &= DISABLE_OVF;
        TCCR1B = 0;
        EICRA |= RISING_EDGE0;
        is_in_pwm[0] = true;
        counter_overflow[0] = 0;
        return;
    }

    if (counter_overflow[0] >= 28) {
        if (digitalRead(2) == HIGH && is_waiting_for_tick[0]) {
            //                   TIMSK2 &= DISABLE_OVF;
            TCCR1B = 0;
            reset(0);
            is_waiting_for_tick[0] = false;
        }

        counter_overflow[0] = 0;
        return;
    }
    
    */
}


// castle data timeout
void reset(int index)
{
    CASTLE_PRIV_DATA *d = &(data[index]);
    d->frameIdx = FRAME_RESET;
    //  Serial.println("reset");
    //if castle ESC ticked some data in, reset the ticked indicator for next cycle
    flag_ready[index] = false;
}


inline void tick(unsigned int ticks, int index)
{
    //Serial.println(ticks);
    if (ticks == 0) { return; } //timer was stopped

    CASTLE_PRIV_DATA *d = &(data[index]);
    d->frameIdx++;
    d->ticks[d->frameIdx] = ticks;
    //  Serial.println(d->ticks[0]);

    if (d->frameIdx == DATA_FRAME_CNT - 1) {
        flag_ready[index] = true;
        d->frameIdx = FRAME_RESET;
        //  Serial.println("READY");
    }
}

/*

uint8_t getData(uint8_t index, CASTLE_RAW_DATA *o)
{
    if (flag_ready[index]) { //wait for ISRs code to finish filling data structure. Stops the main loop, nothing else is procesed if not ready. Why not revisit when ready ?
      EIMSK = 0;
      memcpy(o, data[index].ticks, sizeof(unsigned int) * DATA_FRAME_CNT);
      EIMSK  |= (1 << INT1) ;//| (1 << INT1);
      return true;
    }else {
      return false;
    }

}*/


uint8_t getData(uint8_t index, CASTLE_ESC_DATA *o)
{
    uint8_t whichTemp;
    CASTLE_RAW_DATA c;

    if (flag_ready[index]) { //only do if ISRs code finishes filling data structure
        memcpy(&c, &(data[index]), sizeof(unsigned int) * DATA_FRAME_CNT);
        whichTemp = CLL_GET_WHICH_TEMP(c);
    
        for (int f = 1; f < DATA_FRAME_CNT; f++) {
            float value = CLL_BASE_VALUE(
                              c.ticks[f],
                              c.ticks[FRAME_REFERENCE],
                              CLL_GET_OFFSET_TICKS(c)
                          );
    
            switch (f) {
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
                if (whichTemp == FRAME_TEMP1) { o->temperature = CLL_CALC_TEMP1(value); }
    
                break;
    
            case FRAME_TEMP2:
                if (whichTemp == FRAME_TEMP2) {
                    o->temperature = CLL_CALC_TEMP2(value);
                }
    
                break;
            }
        }
        
        // Reset flag immediately. Only true again when next complete batch of ticks are recorded. 
        // Why wait for reset() to be called from timer-overflow ? Without this, random ticks are 
        // recorded(with no pwm connected!) and after the required nuumber of them, flag is set. 
        // Some check is required that says the recorded batch of ticks should be within so much time.
        flag_ready[index] = false;
        
        return true;

    } else return false;

}


void print_data(CASTLE_ESC_DATA *d)
{
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

void print_data(CASTLE_RAW_DATA *d)
{
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



