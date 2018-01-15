#ifndef _DELAY_H_
#define _DELAY_H_

#include "stm32f10x.h"



void Init_SysTick(void);
void delay_us(__IO uint32_t nTime);
void delay(__IO uint32_t nTime);
uint32_t millis(void);

#endif // !_DELAY_H_

