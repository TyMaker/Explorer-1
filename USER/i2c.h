#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f10x.h"
#include "ARM_SSD1306.h"

#define OLED_SCLK_Clr() GPIO_ResetBits(SSD1306_CLK_PORT,SSD1306_CLK_GPIO)//SCL
#define OLED_SCLK_Set() GPIO_SetBits(SSD1306_CLK_PORT,SSD1306_CLK_GPIO)

#define OLED_SDIN_Clr() GPIO_ResetBits(SSD1306_SDA_PORT,SSD1306_SDA_GPIO)//SDA
#define OLED_SDIN_Set() GPIO_SetBits(SSD1306_SDA_PORT,SSD1306_SDA_GPIO)


void WriteCmd(uint8_t i2caddr, uint8_t control, uint8_t command);
void WriteDat(uint8_t i2caddr, uint8_t control, uint8_t data);

#endif /* __i2c_h_ */
