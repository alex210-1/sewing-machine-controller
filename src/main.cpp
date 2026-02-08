#include <Arduino.h>
#include "main.h"
#include "input.h"

#define ENCODER_PIN 2 // INT0 pin
#define PWM_PIN 11    // OC2A pin

#define F_CTRL (F_CPU / (256 * 64))
#define F_PWM (F_CPU / (256 * 8))
#define ENCODER_PULSES 1000
#define ENCODER_TICK (F_CPU / 8)

#define Kp 20
#define Ki 0
#define Li (Ki * 1)

volatile uint16_t smooth_encoder_count = 0;
volatile float target_rps_sync = 0;
volatile float current_rps_sync = 0;
volatile byte isr_time_usage = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting");

    pinMode(INPUT, PEDAL_PIN);
    pinMode(INPUT, DIAL_PIN);
    pinMode(INPUT, ENCODER_PIN);
    pinMode(OUTPUT, PWM_PIN);
    DDRB |= (1 << PB3); // pin 11 COM1A output

    // === setup enocoder timer ===
    attachInterrupt(INT0, ENCODER_ISR, RISING); // attach encoder interrupt
    // reset timer registers, the arduino library seems to be setting it's own defaults!!!
    TCCR1A = 0x00;
    TCCR1B = 0x02 << CS10; // set clock prescaler to 8. This starts the timer.
    TCCR1C = 0x00;

    // === set up PWM timer ===
    TCCR2A = 0x03 << WGM20; // set waveform generation mode to Fast PWM (full)
    TCCR2B = 0x02 << CS20;  // set clock prescaler to 8. This starts the timer.
    TIMSK2 = 1 << TOIE2;    // enable timer 2 overflow interrupt

    // set noninverting PWM output active on OC2A pin.
    // Nothing should happen until value is written to OCR2A, though
    TCCR2A |= 0x02 << COM2A0;
}

void loop()
{
    float target_rps = calc_target_speed();

    cli();
    target_rps_sync = target_rps; // keep calculation out of critical section
    sei();

    // Log state
    Serial.println();
    Serial.print(">target_rps:");
    Serial.println(target_rps);
    Serial.print(">current_rps:");
    Serial.println(current_rps_sync);
    Serial.print(">PWM:");
    Serial.println(OCR2A);
    Serial.print(">isr_time_usage:");
    Serial.println(isr_time_usage);
}

inline float clampf(float x, float min, float max)
{
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

// TODO timer overflow ISR
void ENCODER_ISR()
{
    // CPU automatically clears global interrupt enable (and reenables afterwards).
    // the compiler takes care of storing and restoring the SREG register.
    // This interrupt is sensitive to timing jitter,
    // so critical sections everywhere else need to be short.

    // Datasheet says compiler handles 16-bit access here
    uint16_t encoder_time = TCNT1;
    TCNT1 = 0; // reset timer

    // TODO use TOV1 flag for overflow detection

    // do some primitive exponential smoothing to filter out potential glitches and jitter.
    // prevent overflow. The compiler optimizes this nicely
    smooth_encoder_count = (uint16_t)(((uint32_t)smooth_encoder_count + encoder_time) / 2);
}

// PWM (Timer 0): PI control loop
// make this ISR preemptable by setting the interrupt enable flag.
// this should reduce jitter for the encoder ISR.
ISR(TIMER2_OVF_vect)
{
    // limit control loop frequency
    static int limiter = 0;

    if (limiter++ < 8)
        return;

    limiter = 0;

    // static float u_i = 0; // integral part

    float target_rps = target_rps_sync;
    uint16_t current_encoder_count = smooth_encoder_count;
    sei();

    // see notes.md
    float current_rps = (ENCODER_TICK / ENCODER_PULSES) / current_encoder_count;

    float e = target_rps - current_rps;
    // u_i += (e * Ki) / F_CTRL;

    // windup prevention by clamping integrand
    // u_i = clampf(u_i, -Li, Li);

    float u = e * Kp; // + u_i;
    byte u_int = (byte)clampf(u, 0, 0xFF);

    // set pwm comparator value. This register is double buffered
    // and the write will be applied on the next cycle of the timer
    cli(); // disable interrupts for atomic access
    OCR2A = u_int;
    current_rps_sync = current_rps;
    isr_time_usage = TCNT2; // monitor timing headroom
}
