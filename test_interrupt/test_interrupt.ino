#include "CastleLinkLive.h"

int counter_overflow;
bool is_in_pwm;
bool is_waiting_for_tick;
uint8_t TIMER_START = 0b10; //with presacler 8
uint8_t TIMER_STOP = ! TIMER_START;
uint8_t ENABLE_OVF = (1 << TOV2);
uint8_t DISABLE_OVF = !ENABLE_OVF;
uint8_t RISING_EDGE = (1 << ISC11) | (1 << ISC10); //0b00001100
uint8_t FALLING_EDGE = (!RISING_EDGE) | (1 << ISC11); // 0b11111011 only works if before was rising edge

typedef struct castle_priv_data_struct {
   uint16_t ticks[DATA_FRAME_CNT];

   int frameIdx;
   uint8_t ticked;
   uint8_t ready;
} CASTLE_PRIV_DATA;

CASTLE_PRIV_DATA data;

uint8_t debug[1000];
int counter = 0;
void setup()
{

  //init data structure
  data.frameIdx = FRAME_RESET;
  data.ready = 0;
  data.ticked = 0;
  for (int j = 0; j < DATA_FRAME_CNT; j++)
    data.ticks[j] = 0;
  //

    is_in_pwm = false;
    is_waiting_for_tick = false;
    pinMode(13, OUTPUT);
    pinMode(3, INPUT);
    Serial.begin(9600);
    cli();
    EICRA |= (1 << ISC11); //Falling Edge
    EIMSK |= (1 << INT1); //Enable interrupt via pin
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2B |= 0b010;    // 8 prescaler 0b010;
    Serial.println("Start");
    sei();
}

int value = 1;
void loop()
{
    delay(1000);

    for (int i = 0; i < 100; i++) {
        Serial.println(debug[i]);
    }

    delay(1000000);
}


ISR(INT1_vect)
{
    unsigned int time = TCNT2 + counter_overflow << 8; //take time
    TCNT2 = 0;    //reset time
    counter_overflow = 0;

    if (is_waiting_for_tick) {
      tick(time);
        debug[counter] = value;
        counter++;
        value++;
        TIMSK2 &= DISABLE_OVF;
        is_waiting_for_tick = false;
        return;
    }

    TIMSK2 |= ENABLE_OVF;

    if (is_in_pwm) {
        is_waiting_for_tick = true;
        is_in_pwm = false;
        EICRA &= FALLING_EDGE;
        return;
    }
}

ISR(TIMER2_OVF_vect)
{
    TCNT2 = 0;
    counter_overflow++;

    if (counter_overflow == 3) {
        if (digitalRead(3) == LOW) {
            TIMSK2 &= DISABLE_OVF;
            EICRA = RISING_EDGE;
            is_in_pwm = true;
        }
    }

    if (counter_overflow == 28) {
        if (digitalRead(3) == HIGH && is_waiting_for_tick) {
            debug[counter] = 0;
            value = 0;
            counter++;
            TIMSK2 &= DISABLE_OVF;
            is_waiting_for_tick = false;
        }

        counter_overflow = 0;
    }
}











// castle data timeout
void reset()
{

        CASTLE_PRIV_DATA *d = &data;

        //if castle ESC ticked some data in, reset the ticked indicator for next cycle
        //otherwise, it was a reset frame
        if (d->ticked) {
            d->ticked = false;

        } else {
            d->frameIdx = FRAME_RESET;
        }

        if (d->frameIdx == FRAME_RESET && d->ready) {
            d->ready = false;

        }

}


inline void tick(uint16_t ticks) {


  
  if (ticks == 0) return; //timer was stopped


  CASTLE_PRIV_DATA *d = &data;
  
  d->frameIdx++;
  
  d->ticks[d->frameIdx] = ticks;

  d->ticked = true;


  
  d->ready = (d->frameIdx == DATA_FRAME_CNT -1);

}


