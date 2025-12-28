#include <Arduino.h>
#include "main.h"
#include "input.h"

#define DEBUG

#define ENCODER_PIN 2 // INT0 pin
#define PWM_PIN 11    // OC2A pin

#define F_PWM (F_CPU / (256 * 32))
#define ENCODER_PULSES 1000
#define ENCODER_TICK (F_CPU / 8)

volatile float Kp = 0;
volatile float Ki = 0;
volatile float Li = Ki * 1; // Integral limit

volatile uint16_t smooth_encoder_count = 0;
volatile float target_rps_sync = 0;
volatile byte isr_time_usage = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting");

    pinMode(INPUT, PEDAL_PIN);
    pinMode(INPUT, DIAL_PIN);
    pinMode(INPUT, ENCODER_PIN);
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
}

void loop()
{
    float target_rps = calc_target_speed();

    cli();
    target_rps_sync = target_rps; // keep calculation out of critical section
    sei();

#ifdef DEBUG
    // Log state
    Serial.print(">target_rps:");
    Serial.println(target_rps);
    Serial.print(">smooth_encoder_count:");
    Serial.println(smooth_encoder_count);
    Serial.print(">PWM:");
    Serial.println(OCR2A);
    Serial.print(">isr_time_usage:");
    Serial.println(isr_time_usage);

    // parameter tuning
    if (Serial.available())
    {
        char cmd = Serial.read();
        if (cmd == 'P')
        {
            // Writing these non-atomic should be "good enough" for tuning.
            Kp = Serial.parseFloat();
        }
        else if (cmd == 'I')
        {
            Ki = Serial.parseFloat();
        }
        else if (cmd == 'L')
        {
            Li = Serial.parseFloat();
        }
        else
        {
            Serial.println("Unknown command");
        }
    }
#endif
}

inline float clampf(float x, float min, float max)
{
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

void ENCODER_ISR()
{
    // CPU automatically clears global interrupt enable (and reenables afterwards).
    // the compiler takes care of storing and restoring the SREG register.
    // This interrupt is sensitive to timing jitter,
    // so critical sections everywhere else need to be short.

    uint16_t encoder_time = TCNT1; // compiler takes care of atomic access here TODO verify
    TCNT1 = 0;                     // reset timer

    // do some primitive exponential smoothing to filter out potential glitches and jitter.
    // prevent overflow. The compiler optimizes this nicely
    smooth_encoder_count = (uint16_t)(((uint32_t)smooth_encoder_count + encoder_time) / 2);
}

// PWM (Timer 2): PI control loop
// make this ISR preemptable by setting the interrupt enable flag.
// this should reduce jitter for the encoder ISR.
ISR(TIMER2_OVF_vect)
{
    static float u_i = 0; // integral part

    float target_rps = target_rps_sync;
    uint16_t current_encoder_count = smooth_encoder_count;
    sei();

    // see notes.md
    float current_rps = (ENCODER_TICK / ENCODER_PULSES) / current_encoder_count;

    float e = target_rps - current_rps;
    u_i += (e * Ki) / F_PWM;

    // windup prevention by clamping integrand
    u_i = clampf(u_i, -Li, Li);

    float u = e * Kp + u_i;
    byte u_int = (byte)clampf(u, 0, 0xFF);

    // set pwm comparator value. This register is double buffered
    // and the write will be applied on the next cycle of the timer
    cli(); // disable interrupts for atomic access
    OCR2A = u_int;
    isr_time_usage = TCNT2; // monitor timing headroom
}
