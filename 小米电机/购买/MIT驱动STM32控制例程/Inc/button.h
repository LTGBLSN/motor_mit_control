#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32F0xx_hal.h"

void Button_Init(void);
int8_t get_bnt1_state(void);
int8_t get_bnt2_state(void);

#endif

