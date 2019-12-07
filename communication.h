#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm8s.h"

void init_com_interface(uint32_t baudrate);
void com_send(const char* str);
#endif
