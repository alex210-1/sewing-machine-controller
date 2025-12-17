#include "fixpoint.h"

ufix1616_t int_to_fix(uint16_t val)
{
    return ((uint32_t)val) << 16;
}

uint16_t fix_to_int(ufix1616_t val)
{
    return (uint16_t)(val >> 16);
}
