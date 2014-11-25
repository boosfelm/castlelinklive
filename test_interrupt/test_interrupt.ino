#include "test_interrupt.h"
#include <Wire.h>

//made volatile
volatile int counter_overflow[2];
volatile bool is_in_pwm[2];
volatile bool is_waiting_for_tick[2];

uint8_t RISING_EDGE1 = 0b00001100;//(1 << ISC11) | (1 << ISC10); //0b00001100 //dec 12 //EICRA configuration check datasheet
uint8_t FALLING_EDGE1 = 0b11111011;//(!RISING_EDGE1) | (1 << ISC11); // 0b11111011 //dec 251  //EICRA configuration check datasheet

uint8_t RISING_EDGE0 = 0b00000011;//(1 << ISC01) | (1 << ISC00); //0b00000011 // dec 3 //EICRA configuration check datasheet
uint8_t FALLING_EDGE0 = 0b11111110;//(!RISING_EDGE0) | (1 << ISC01); // 0b11111110 // dec 254 //EICRA configuration check datasheet

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


float iii[2] = {130,140};// = new byte[1];
byte *idata = new byte[8];

//iii[0] = 130;

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

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
  
    pinMode(3, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    Serial.begin(2400);
    cli();
    
    //Configure Timer2 (8bit). Over flow at 0-255. 
    TIMSK2 |= (1<<TOIE2);                             // Enable overflow
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2B |= 0b010;                                  // 8 prescaler 0b010;

    //Configure Timer1 (16bit). CTC reset at TOP 256.
    ICR1 = 256;                                       // Comparator TOP value. Timer1 (16bit) should reset in CTC mode via TCCR1A and TCCR1B values. Check datasheet.
    TIFR1 |= _BV(OCF1A);                              // Clear any pending interrupts; 
    TIMSK1 |=  _BV(OCIE1A) ;                          // Enable the output compare interrupt 
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1A |= 0b00;
    TCCR1B |= 0b11010;                                // Set prescaler of 8.

    //Configure Interrupts at falling edge for both interrupts.
    EICRA |=  (1 << ISC11) | (1 << ISC01);            // Falling Edge for both interrupt INT0 and interrupt INT1 //0b00001010 //dec 

    EIMSK |= (1 << INT1);                             // Enable interrupt via pin for INT1 configured to time with Timer2
    EIMSK |= (1 << INT0);                             // Enable external interrupt on INT0, times with Timer1.
    
    
    Wire.begin(2);
    Wire.onRequest(requestEvent);                     // Register event
    Serial.println("Start");
    sei();
}


void loop()
{
    
    if(getData(0, &escdata[0])){
      Serial.println(escdata[0].RPM);
      float data_float[12] = {escdata[0].voltage, escdata[0].current, escdata[0].RPM * 2.0f / 24.0f, escdata[0].BECvoltage, escdata[0].BECcurrent, escdata[0].temperature, escdata[0].voltage, escdata[0].current, escdata[0].RPM * 2.0f / 24.0f, escdata[0].BECvoltage, escdata[0].BECcurrent, escdata[0].temperature};
      i2cdata = (byte *)data_float;  
    }
    /*
    if(getData(1, &escdata[1])){
      Serial.println(escdata[1].RPM);
      float data_float[12] = {escdata[1].voltage, escdata[1].current, escdata[1].RPM * 2.0f / 24.0f, escdata[1].BECvoltage, escdata[1].BECcurrent, escdata[1].temperature, escdata[1].voltage, escdata[1].current, escdata[1].RPM * 2.0f / 24.0f, escdata[1].BECvoltage, escdata[1].BECcurrent, escdata[1].temperature};
      i2cdata = (byte *)data_float;  
    }*/
    
    //iii[1] = 130;// = new byte[1];
    idata = (byte *)iii;
    
    delay(20);
}

void requestEvent()
{
    Wire.write(idata, 8);                          // Respond with message of 6 bytes
}
 

/* A     B  C  D E     F       G  H   I J     K      L  M   N       O       P  Q   R  S     T
 * ______       _ _____________        _ ____________        _______________        __ _________
 *       |_____| |             |______| |            |______|               |______|  |   
 *
 * The above waveform is what is expected out of the ESC. BCD, GHI, LMN & PQR represent the PWM 
 * values. The RPM/Voltage/Current etc information is encoded in the position of a batch of 'Ticks' 
 * positioned after the PWMs, represented here by E,J and S. We basically want to measure the time
 * interval between the end of PWM and start of 'Tick', here length of DE, IJ and RS.
 *
 * The logic for detecting the time to 'Ticks', or just calling them 'Ticks', is exactly the same
 * for both Timer2 (8 bit) and Timer1 (16 bit). The wave-forms (one for each ESC, on PIN2 and PIN3) 
 * is sampled every 256 colck cycles prescaled by value 8 prescaler. That, for 8MHz clock is around 
 * 256 micro seconds. This sampling is done by over-flow interrup for Timer2 and COMPA interrupt for
 * Timer1. Over-flow counters are incremented to keep track of the length of time.
 *
 * The logic below can be verified by manually running through it with the waveform given above. Key
 * values to note in the waveform are:
 * Waveform cycle (eg DI or IN or NR) = 20 milli seconds
 * Minimum length of PWM (eg BD or GI or LN or PR) = 0.7 milli seconds (over-flow > 3)
 * Maximum time to 'Tick', when Tick is present (not RESET as in NP) < 7 milli seconds  (over-flow < 28)
 */



ISR(INT1_vect)
{

    unsigned int time = TCNT2;  
    unsigned int ovf = counter_overflow[1];
    
    TCNT2 = 0;                                        // Reset time
    counter_overflow[1] = 0;
    time = time + (ovf << 8);

    //my_ctr = my_ctr + 1;                            // For debug only

    if (is_waiting_for_tick[1]) {                     // Enter at E J S 
        tick(time,1);
        digitalWrite(13,LOW);
        TCCR2B = 0; 
        is_waiting_for_tick[1] = false;
        return;
    }
    
    TCCR2B = 0b10;

    if (is_in_pwm[1]) {                              // Enter at B G L P
        
        is_waiting_for_tick[1] = true;
        is_in_pwm[1] = false;
        EICRA &= FALLING_EDGE1;   
        return;
    }
    
    
}


ISR(TIMER2_OVF_vect)
{
    counter_overflow[1]++;

    // Time taken = (2power8 = 256) x (prescale=8) / (clockfreq = 8Mhz) = 256 microsec per overflow. 
    // 3 overflow = 3*256 microsec = 0.7 ms low bottom we are in pwm 
    if (counter_overflow[1] == 3 && digitalRead(3) == LOW) {  // Enter at C H M Q
        TCCR2B = 0; 
        EICRA |= RISING_EDGE1;
        is_in_pwm[1] = true;
        counter_overflow[1] = 0;
        return;
    }
    // No falling edge for over 7ms after rising edge
    if (counter_overflow[1] >= 28) {                         // Enter at O
        if (digitalRead(3) == HIGH && is_waiting_for_tick[1]) {
            TCCR2B = 0; 
            reset(1);
            is_waiting_for_tick[1] = false;
        }

        counter_overflow[1] = 0;
        return;
    }
    
    
}


ISR(INT0_vect) 
{  
  unsigned int time = TCNT1;
  unsigned int ovf = counter_overflow[0];

    TCNT1 = 0;    //reset time
    counter_overflow[0] = 0;
    time = time + (ovf << 8); 
  

    if (is_waiting_for_tick[0]) {                          // Enter at E J S
        digitalWrite(13, HIGH);
        tick(time,0);
        TCCR1B = 0;
        is_waiting_for_tick[0] = false;
        return;
    }

    TCCR1B = 0b11010;                                      // Start prescaler, keep CTC mode as it is

    if (is_in_pwm[0]) {                                    // Enter at B G L P
        is_waiting_for_tick[0] = true;
        is_in_pwm[0] = false;
        EICRA &= FALLING_EDGE0;
        return;
    }
    
    
}


ISR(TIMER1_COMPA_vect)
{
    counter_overflow[0]++;

    // Time taken = (2power8 = 256) x (prescale=8) / (clockfreq = 8Mhz) = 256 microsec per overflow. 
    // 3 overflow = 3*256 microsec = 0.7 ms low bottom we are in pwm
    if (counter_overflow[0] == 3 && digitalRead(2) == LOW) {  // Enter at C H M Q
        TCCR1B = 0;
        EICRA |= RISING_EDGE0;
        is_in_pwm[0] = true;
        counter_overflow[0] = 0;
        return;
    }

    if (counter_overflow[0] >= 28) {                          // Enter at O
        if (digitalRead(2) == HIGH && is_waiting_for_tick[0]) {
            TCCR1B = 0;
            reset(0);
            is_waiting_for_tick[0] = false;
        }
        counter_overflow[0] = 0;
        return;
    }
    
    
}


// castle data timeout
void reset(int index)
{
    CASTLE_PRIV_DATA *d = &(data[index]);
    d->frameIdx = FRAME_RESET;
    flag_ready[index] = false;
}


inline void tick(unsigned int ticks, int index)
{
    if (ticks == 0) { return; } //timer was stopped

    CASTLE_PRIV_DATA *d = &(data[index]);
    d->frameIdx++;
    d->ticks[d->frameIdx] = ticks;

    if (d->frameIdx == DATA_FRAME_CNT - 1) {
        flag_ready[index] = true;
        d->frameIdx = FRAME_RESET;//TEST THIS
        //  Serial.println("READY");
    }
}

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
        
        // Reset flag immediately.
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



