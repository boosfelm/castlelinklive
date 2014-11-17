#include "test_interrupt.h"
#include <Wire.h>

int counter_overflow[2];
bool is_in_pwm[2];
bool is_waiting_for_tick[2];

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

CASTLE_ESC_DATA escdata[2];


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

    pinMode(3, INPUT);
    pinMode(2, INPUT);
    Serial.begin(2400);
    cli();
    TIMSK2 |= (1<<TOIE2); //Enable overflow
    OCR1A = 256;
    TIMSK1 |= (1<<OCIE1A);//Enable compare interrupt
    EICRA |= (1 << ISC11) ;//| (1 << ISC01); //Falling Edge
    EIMSK |= (1 << INT1); //| (1 << INT0); //Enable interrupt via pin
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR2B |= 0b010;    // 8 prescaler 0b010;
//    TCCR1B |= 0b010;    // 8 prescaler 0b010;
    Wire.begin(2);
    Wire.onRequest(requestEvent); // register event
    Serial.println("Start");
    sei();
}

void loop()
{
    //  delay(400);
    getData(1, &escdata[1]);
//    getData(0, &escdata[1]);
    Serial.println(escdata[1].voltage);
//    Serial.println("HELLO");
    float data_float[12] = {escdata[1].voltage, escdata[1].current, escdata[1].RPM * 2.0f / 24.0f, escdata[1].BECvoltage, escdata[1].BECcurrent, escdata[1].temperature, escdata[1].voltage, escdata[1].current, escdata[1].RPM * 2.0f / 24.0f, escdata[1].BECvoltage, escdata[1].BECcurrent, escdata[1].temperature};
    i2cdata = (byte *)data_float;
    delay(20);
}

void requestEvent()
{
    Wire.write(i2cdata, 48); // respond with message of 6 bytes
}


ISR(INT1_vect)
{
    unsigned int ovf = counter_overflow[1];
    unsigned int time = TCNT2;
    TCNT2 = 0;    //reset time
    counter_overflow[1] = 0;
    time = time + (ovf << 8);

    if (is_waiting_for_tick[1]) {
        tick(time,1);
        //        TIMSK2 &= DISABLE_OVF;
        TCCR2B = 0;
        is_waiting_for_tick[1] = false;
        return;
    }

    //    TIMSK2 |= ENABLE_OVF;
    TCCR2B = 0b10;

    if (is_in_pwm[1]) {
        is_waiting_for_tick[1] = true;
        is_in_pwm[1] = false;
        EICRA &= FALLING_EDGE1;
        return;
    }
}

ISR(INT0_vect)
{
    unsigned int ovf = counter_overflow[0];
    unsigned int time = TCNT1;
    TCNT1 = 0;    //reset time
    counter_overflow[0] = 0;
    time = time + (ovf << 8);

    if (is_waiting_for_tick[0]) {
        tick(time,0);
        //        TIMSK2 &= DISABLE_OVF;
        TCCR1B = 0;
        is_waiting_for_tick[0] = false;
        return;
    }

    //    TIMSK2 |= ENABLE_OVF;
    TCCR1B = 0b10;

    if (is_in_pwm[0]) {
        is_waiting_for_tick[0] = true;
        is_in_pwm[0] = false;
        EICRA &= FALLING_EDGE0;
        return;
    }
}

ISR(TIMER2_OVF_vect)
{
    counter_overflow[1]++;

    if (counter_overflow[1] == 3 && digitalRead(3) == LOW) {
        //            TIMSK2 &= DISABLE_OVF;
        TCCR2B = 0;
        EICRA |= RISING_EDGE1;
        is_in_pwm[1] = true;
        counter_overflow[1] = 0;
        return;
    }

    if (counter_overflow[1] >= 28) {
        if (digitalRead(3) == HIGH && is_waiting_for_tick[1]) {
            //                   TIMSK2 &= DISABLE_OVF;
            TCCR2B = 0;
            reset(1);
            is_waiting_for_tick[1] = false;
        }

        counter_overflow[1] = 0;
        return;
    }
}

ISR(TIMER1_COMPA_vect)
{
    counter_overflow[0]++;

    if (counter_overflow[0] == 2 && digitalRead(2) == LOW) {
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
        //  Serial.println("READY");
    }
}

uint8_t getData(uint8_t index, CASTLE_RAW_DATA *o)
{
    while (!flag_ready[index]) { //wait for ISRs code to finish filling data structure
    }

    EIMSK = 0;
    memcpy(o, data[index].ticks, sizeof(unsigned int) * DATA_FRAME_CNT);
    EIMSK  |= (1 << INT1) ;//| (1 << INT0);
    return true;
}


uint8_t getData(uint8_t index, CASTLE_ESC_DATA *o)
{
    uint8_t whichTemp;
    CASTLE_RAW_DATA c;

    while (!flag_ready[index]) { //wait for ISRs code to finish filling data structure
    }

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

    return true;
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



