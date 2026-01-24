#pragma once
#include "stdint.h"

#define PEDAL_PIN A1
#define DIAL_PIN A0

float calc_target_speed();
float calc_dummy_speed();

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
