#include "input.h"
#include <Arduino.h>

#define PEDAL_THRESHOLD 0.2
#define MAX_RPS 10

// 10 bit ADC inputs, simple exponential smoothing
volatile uint16_t smooth_pedal_value = 0;
volatile uint16_t smooth_dial_value = 0;

/// @return target rev/s
float calc_target_speed()
{
    float pedal_val = ((float)smooth_pedal_value) / 1024;
    float dial_mult = ((float)smooth_dial_value) / (1024 * 2);

    if (pedal_val < PEDAL_THRESHOLD)
        return 0;

    float target = fmap(pedal_val, PEDAL_THRESHOLD, 1, 0, MAX_RPS);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ADC conversion finished
ISR(ADC_vect)
{
    // Using analogRead is too slow (blocks ~100us default).
    // use the ADC ISR to continously cycle through all analog inputs
    char current_adc_in = ADMUX & 0x0F;

    static bool discard_next = true;

    // the first measurement after switching the pin needs to be discarded!
    if (discard_next)
    {
        discard_next = false;
        return;
    }
    uint16_t adc_value = ADC; // compiler takes care of register access

    // select variable to write and apply smoothing
    switch (current_adc_in)
    {
    case 0:
        smooth_pedal_value = (smooth_pedal_value + adc_value) / 2;
        break;
    case 1:
        smooth_dial_value = (smooth_dial_value + adc_value) / 2;
        break;
    }

    // cycle through adc inputs
    ADMUX = ADMUX & 0xF0 | ((current_adc_in + 1) % 2);
    discard_next = true;
}
