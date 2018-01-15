#ifndef _ARM_SSD1306_H_
#define _ARM_SSD1306_H_

#include "stm32f10x.h"

#define BLACK 0
#define WHITE 1
#define INVERSE 2

#define SSD1306_I2C_ADDRESS   0x3C  // 011110+SA0+RW - 0x3C or 0x3D
// Address for 128x32 is 0x3C
// Address for 128x64 is 0x3D (default) or 0x3C (if SA0 is grounded)

/*=========================================================================
    SSD1306 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
    Select the appropriate display below to create an appropriately
    sized framebuffer, etc.

    SSD1306_128_64  128x64 pixel display

    SSD1306_128_32  128x32 pixel display

    SSD1306_96_16

    -----------------------------------------------------------------------*/
#define SSD1306_128_64
//#define SSD1306_128_32
//#define SSD1306_96_16
/*=========================================================================*/

#if defined SSD1306_128_64 && defined SSD1306_128_32
#error "Only one SSD1306 display can be specified at once in SSD1306.h"
#endif
#if !defined SSD1306_128_64 && !defined SSD1306_128_32 && !defined SSD1306_96_16
#error "At least one SSD1306 display must be specified in SSD1306.h"
#endif

#if defined SSD1306_128_64
#define SSD1306_LCDWIDTH                  128
#define SSD1306_LCDHEIGHT                 64
#endif
#if defined SSD1306_128_32
#define SSD1306_LCDWIDTH                  128
#define SSD1306_LCDHEIGHT                 32
#endif
#if defined SSD1306_96_16
#define SSD1306_LCDWIDTH                  96
#define SSD1306_LCDHEIGHT                 16
#endif



/******* DEFINE	GPIO PORT & PIN ********/
#define SSD1306_RES_PORT			GPIOB
#define SSD1306_RES_GPIO			GPIO_Pin_0

#define SSD1306_SCK_PORT			GPIOA
#define SSD1306_SCK_GPIO			GPIO_Pin_5

#define SSD1306_DAT_PORT			GPIOA
#define SSD1306_DAT_GPIO			GPIO_Pin_7

#define SSD1306_DC_PORT				GPIOB
#define SSD1306_DC_GPIO				GPIO_Pin_1

#define SSD1306_CS_PORT				GPIOA
#define SSD1306_CS_GPIO				GPIO_Pin_4

#define SSD1306_SDA_PORT			GPIOB
#define SSD1306_SDA_GPIO			GPIO_Pin_11

#define SSD1306_CLK_PORT			GPIOB
#define SSD1306_CLK_GPIO			GPIO_Pin_10

#define SSD1306_PORT_CLOCK		RCC_APB2Periph_GPIOB

#define READ_SDA							GPIO_ReadInputDataBit(SSD1306_SDA_PORT,SSD1306_SDA_GPIO)

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

#define	HARDWARE_SPI		0x01
#define SOFTWARE_SPI		0x02
#define SOFTWARE_I2C		0x04

#ifndef __STM32F10x_TYPE_H    
typedef enum { FALSE = 0, TRUE } BOOL;
#else
typedef bool BOOL;
#endif

int16_t width(void);
int16_t height(void);
void ssd1306_drawPixel(int16_t x, int16_t y, uint16_t color);
void ssd1306_begin(uint8_t _vccstate, uint8_t i2caddr, uint8_t hw_SPI, uint8_t reset);
void ssd1306_command(uint8_t c);
void ssd1306_startscrollright(uint8_t start, uint8_t stop);
void ssd1306_startscrollleft(uint8_t start, uint8_t stop);
void ssd1306_startscrolldiagright(uint8_t start, uint8_t stop);
void ssd1306_startscrolldiagleft(uint8_t start, uint8_t stop);
void ssd1306_stopscroll(void);
void ssd1306_dim(int8_t dim);
void ssd1306_display(void);
void ssd1306_clearDisplay(void);
void ssd1306_fastSPIwrite(uint8_t d);
void ssd1306_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void ssd1306_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void ssd1306_drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color);
void ssd1306_drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color);

#endif // !_ARM_SSD1306_H_
