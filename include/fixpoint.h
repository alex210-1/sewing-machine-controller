#pragma once
#include "stdint.h"

typedef uint32_t ufix1616_t;

ufix1616_t int_to_fix(uint16_t val);

uint16_t fix_to_int(ufix1616_t val);
