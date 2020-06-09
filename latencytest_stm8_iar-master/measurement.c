#include "measurement.h"

/* https://stackoverflow.com/a/3893967/1794026 */
int 
compare_func( const void* a, const void* b) {
	/* Compare function for qsort (used for median sorting) */
     int int_a = *((int*) a);
     int int_b = *((int*) b);
     return (int_a > int_b) - (int_a < int_b);
}

uint16_t 
median_16(uint16_t* buf, uint32_t blen) {
  /* Returns median of given (u16) buffer */
  return (blen % 2 == 0) ?  ((buf[(blen-1)/2] + buf[blen/2])/2) : buf[blen/2];
}