#ifndef __MEASUREMENT_H
#define __MEASUREMENT_H

#include "stm8s.h"

int compare_func( const void* a, const void* b);
uint16_t median_16(uint16_t* buf, uint32_t blen);

#endif