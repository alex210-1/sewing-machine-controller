#include "input.h"
#include <Arduino.h>

#define PEDAL_THRESHOLD 0.2
#define MAX_RPS 10

// 10 bit ADC inputs, simple exponential smoothing
uint16_t smooth_pedal_raw = 0;
uint16_t smooth_dial_raw = 0;

/// @return current target speed in rev/s
float calc_target_speed()
{
    smooth_pedal_raw = (smooth_pedal_raw + analogRead(PEDAL_PIN)) / 2;
    smooth_dial_raw = (smooth_dial_raw + analogRead(DIAL_PIN)) / 2;

    float pedal_val = ((float)smooth_pedal_raw) / 1024;
    float dial_adjust = ((float)smooth_dial_raw) / 1024;

    if (pedal_val < PEDAL_THRESHOLD)
        return 0;

    // use liner interpolation for now. Use a Bezier or a polynomial later?
    float target_rps = fmap(pedal_val, PEDAL_THRESHOLD, 1, 0, MAX_RPS);
    return target_rps * dial_adjust;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
