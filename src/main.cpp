#include <Arduino.h>
#include "input.h"

// TODO pinout
#define ENCODER_PIN 1 // TODO INT0 pin
#define PWM_PIN 2     // TODO OC2A pin
#define PEDAL_PIN A0  // ADC in 0
#define DIAL_PIN A1   // ADC in 1

#define F_PWM = F_CPU / (256 * 32)
#define ENCODER_TICK = F_CPU / 8

volatile uint16_t smooth_encoder_time = 0;

volatile uint8_t next_pwm_value = 0;
volatile bool control_sync = false;

void setup()
{
    Serial.begin(115200); // use fast baud rate to avoid stalling the control loop
    Serial.println("Starting");

    // === setup pins ===
    pinMode(INPUT, ENCODER_PIN);
    pinMode(INPUT, PEDAL_PIN);
    pinMode(INPUT, DIAL_PIN);
    pinMode(OUTPUT, PWM_PIN);

    // === setup enocoder timer ===
    attachInterrupt(INT0, ENCODER_ISR, RISING); // attach encoder interrupt
    TCCR1B |= 0x02 << CS10;                     // set clock prescaler to 8. This starts the timer.

    // === set up PWM timer ===
    TCCR2A |= 0x03 << WGM20; // set waveform generation mode to Fast PWM (full)
    TCCR2B |= 0x03 << CS20;  // set clock prescaler to 32. This starts the timer.
    TIMSK2 |= 1 << TOIE2;    // enable timer 2 overflow interrupt

    // set noninverting PWM output active on OC2A pin.
    // Nothing should happen until value is written to OCR2A, though
    TCCR2A |= 1 << COM2A1;

    // === setup adc ===
    ADCSRA |= 0x07 << ADPS0; // set ADC clock divider to 128 (because of 16MHz clock)
    ADCSRA |= 1 << ADATE;    // enable ADC free running mode
    ADCSRA |= 1 << ADIE;     // enable ADC conversion complete interrupt
    ADCSRA |= 1 << ADEN;     // enable ADC
    ADCSRA |= 1 << ADSC;     // start conversion
}

void loop()
{
    // synchronize to pwm frequency
    while (!control_sync)
        ;
    control_sync = false;

    Serial.println(smooth_encoder_time);
}

void ENCODER_ISR()
{
    // CPU automatically clears global interrupt enable (and reenables afterwards).
    // the compiler takes care of storing and restoring the SREG register.

    uint16_t encoder_time = TCNT1; // compiler takes care of atomic access here
    TCNT1 = 0;                     // reset timer

    // do some primitive exponential smoothing to filter out potential glitches and jitter.
    // prevent overflow. The compiler optimizes this nicely
    smooth_encoder_time = (uint16_t)(((uint32_t)smooth_encoder_time + encoder_time) / 2);
}

// PWM (Timer 2)
ISR(TIMER2_OVF_vect)
{
    // sync with main control loop
    control_sync = true;

    // set pwm comparator value. This register is double buffered
    // and the write will be applied on the next cycle of the timer
    OCR2A = next_pwm_value;
}
