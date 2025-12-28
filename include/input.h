#pragma once
#include "stdint.h"

#define PEDAL_PIN A0 // ADC in 0
#define DIAL_PIN A1  // ADC in 1

float calc_target_speed();

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
