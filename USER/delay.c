#ifndef __DELAY_H_

#include "delay.h"

__IO uint32_t myTick;

void Init_SysTick(void) {
#if STM32F40_41XXX
    /* Init Systick */
    if(SysTick_Config(SystemCoreClock / 1000)) 
        while(1) {
            
        }
#elif STM32F10X_HD
    if (SysTick_Config(SystemCoreClock / 1000)) 
        while (1) {
            //Error;
        }    
#endif
SysTick_Config(SystemCoreClock / 1000);

}

void delay_us(__IO uint32_t nTime) {
    uint16_t i;
		while(nTime--) {
			 i = 10;
			 while(i--);
		}
}

void delay(__IO uint32_t nTime) {
    uint32_t start = myTick;
    while (myTick - start < nTime)
        ;
}

uint32_t millis(void) {
	return myTick;
}


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    if (myTick <= 0xffffffff)
        myTick++;
    else
        myTick = 0;
    //myTick = (myTick < 0xf) ?	myTick++ : 0;
}

#endif // !__DELAY_H_
