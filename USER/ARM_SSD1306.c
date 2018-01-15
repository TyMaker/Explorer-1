/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers
*********************************************************************/

#include "ARM_GFX.h"
#include "ARM_SSD1306.h"
#include "delay.h"
#include "string.h"
#include "i2c.h"

__IO uint16_t   WIDTH = SSD1306_LCDWIDTH;
__IO uint16_t   HEIGHT = SSD1306_LCDHEIGHT;
__IO uint8_t    hwSPI;
__IO uint8_t    _vccstate, _i2caddr;

extern uint8_t rotation;

// the memory buffer for the LCD
static uint8_t buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x80, 0x80, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF,
#if (SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH > 96*16)
0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
0x80, 0xFF, 0xFF, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x8C, 0x8E, 0x84, 0x00, 0x00, 0x80, 0xF8,
0xF8, 0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80,
0x00, 0xE0, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xC7, 0x01, 0x01,
0x01, 0x01, 0x83, 0xFF, 0xFF, 0x00, 0x00, 0x7C, 0xFE, 0xC7, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF,
0xFF, 0xFF, 0x00, 0x38, 0xFE, 0xC7, 0x83, 0x01, 0x01, 0x01, 0x83, 0xC7, 0xFF, 0xFF, 0x00, 0x00,
0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x7F, 0xFF,
0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF,
0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x8F,
0x8F, 0x9F, 0xBF, 0xFF, 0xFF, 0xC3, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC,
0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x01, 0x03, 0x03, 0x03,
0x03, 0x03, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01,
0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x00, 0x00,
0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x03,
0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#if (SSD1306_LCDHEIGHT == 64)
0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F, 0x0F,
0x87, 0xC7, 0xF7, 0xFF, 0xFF, 0x1F, 0x1F, 0x3D, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0x7C, 0x7D, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x00, 0x30, 0x30, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xC0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0x1F,
0x0F, 0x07, 0x1F, 0x7F, 0xFF, 0xFF, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF8, 0xE0,
0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00,
0x00, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x0E, 0xFC, 0xF8, 0x00, 0x00, 0xF0, 0xF8, 0x1C, 0x0E,
0x06, 0x06, 0x06, 0x0C, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFC,
0xFE, 0xFC, 0x00, 0x18, 0x3C, 0x7E, 0x66, 0xE6, 0xCE, 0x84, 0x00, 0x00, 0x06, 0xFF, 0xFF, 0x06,
0x06, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x06, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0xC0, 0xF8,
0xFC, 0x4E, 0x46, 0x46, 0x46, 0x4E, 0x7C, 0x78, 0x40, 0x18, 0x3C, 0x76, 0xE6, 0xCE, 0xCC, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x03,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00,
0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x07, 0x0E, 0x0C,
0x18, 0x18, 0x0C, 0x06, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x01, 0x0F, 0x0E, 0x0C, 0x18, 0x0C, 0x0F,
0x07, 0x01, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00,
0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x07,
0x07, 0x0C, 0x0C, 0x18, 0x1C, 0x0C, 0x06, 0x06, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
#endif
};

#define ssd1306_swap(a, b) { int16_t t = a; a = b; b = t; }

// the most basic function, set a single pixel
void ssd1306_drawPixel(int16_t x, int16_t y, uint16_t color) {
    if ((x < 0) || (x >= width() || (y < 0) || (y >= height())))
        return;
    // check rotation, move pixel around if necessary
    switch (getRotation()) {
    case 1:
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        x = WIDTH  - x - 1;
        y = HEIGHT - y - 1;
        break;
    case 3:
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }
    
    // x is which column
    switch (color) {
        case WHITE:   buffer[x + (y / 8)*SSD1306_LCDWIDTH] |=  (1 << (y & 7)); break;
        case BLACK:   buffer[x + (y / 8)*SSD1306_LCDWIDTH] &= ~(1 << (y & 7)); break;
        case INVERSE: buffer[x + (y / 8)*SSD1306_LCDWIDTH] ^=  (1 << (y & 7)); break;
    }
}


void ssd1306_begin(uint8_t vccstate, uint8_t i2caddr, uint8_t hw_SPI, uint8_t reset) {

    // set pin directions
    GPIO_InitTypeDef GPIO_InitStructure;
    hwSPI = hw_SPI;
    _vccstate = vccstate;
    _i2caddr = i2caddr;
    
    if (hwSPI == SOFTWARE_SPI) {
        // set pins for software-SPI
			
			  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
				GPIO_InitStructure.GPIO_Pin = SSD1306_DC_GPIO;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(SSD1306_DC_PORT, &GPIO_InitStructure);
				
				GPIO_InitStructure.GPIO_Pin = SSD1306_CS_GPIO;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(SSD1306_CS_PORT, &GPIO_InitStructure);
				//GPIO_SetBits(SSD1306_DC_PORT, SSD1306_DC_GPIO); 
			
        GPIO_InitStructure.GPIO_Pin = SSD1306_SCK_GPIO;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SSD1306_SCK_PORT, &GPIO_InitStructure);
        
        GPIO_InitStructure.GPIO_Pin = SSD1306_DAT_GPIO;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SSD1306_DAT_PORT, &GPIO_InitStructure);

    }
    if (hwSPI == HARDWARE_SPI) {
        // set pins for hardware-SPI
        
    }
    else
    {
				// I2C Init
				
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
				
			  GPIO_InitStructure.GPIO_Pin = SSD1306_CLK_GPIO;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SSD1306_CLK_PORT, &GPIO_InitStructure);
			
			  GPIO_InitStructure.GPIO_Pin = SSD1306_SDA_GPIO;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SSD1306_SDA_PORT, &GPIO_InitStructure);
			
    }
    if ((reset) && (hwSPI != SOFTWARE_I2C)) {
      // Setup reset pin direction (used by both SPI and I2C)
        GPIO_InitStructure.GPIO_Pin = SSD1306_RES_GPIO;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(SSD1306_RES_PORT, &GPIO_InitStructure);
        GPIO_SetBits(SSD1306_RES_PORT, SSD1306_RES_GPIO);
        
        // VDD (3.3V) goes high at start, lets just chill for a ms
        delay(1);
        // bring reset low
        GPIO_ResetBits(SSD1306_RES_PORT, SSD1306_RES_GPIO);
        // wait 10ms
        delay(10);
        // bring out of reset
        GPIO_SetBits(SSD1306_RES_PORT, SSD1306_RES_GPIO);
        // turn on VCC (9V?)
    }

    // Init sequence
    ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    ssd1306_command(0x80);                                  // the suggested ratio 0x80

    ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
    ssd1306_command(SSD1306_LCDHEIGHT - 1);

    ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    ssd1306_command(0x0);                                   // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x10); }
    else
      { ssd1306_command(0x14); }
    ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
    ssd1306_command(0x00);                                  // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);

#if defined SSD1306_128_32
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x02);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    ssd1306_command(0x8F);

#elif defined SSD1306_128_64
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x12);
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x9F); }
    else
      { ssd1306_command(0xCF); }

#elif defined SSD1306_96_16
    ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
    ssd1306_command(0x2);   //ada x12
    ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x10); }
    else
      { ssd1306_command(0xAF); }

#endif

    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    if (vccstate == SSD1306_EXTERNALVCC)
      { ssd1306_command(0x22); }
    else
      { ssd1306_command(0xF1); }
    ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

    ssd1306_command(SSD1306_DEACTIVATE_SCROLL);

    ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
}

void ssd1306_invertDisplay(uint8_t i) {
    if (i) {
        ssd1306_command(SSD1306_INVERTDISPLAY);
    }
    else {
        ssd1306_command(SSD1306_NORMALDISPLAY);
    }
}

void ssd1306_command(uint8_t c) {
		//uint8_t i;
		uint8_t control;
    if (hwSPI == HARDWARE_SPI)
    {
      // SPI
        GPIO_ResetBits(SSD1306_DC_PORT, SSD1306_DC_GPIO);
        GPIO_ResetBits(SSD1306_CS_PORT, SSD1306_CS_GPIO);
        /* SPI Send 1 Byte */
        GPIO_SetBits(SSD1306_CS_PORT, SSD1306_CS_GPIO);
        GPIO_SetBits(SSD1306_DC_PORT, SSD1306_DC_GPIO);

    }
    else if (hwSPI == SOFTWARE_SPI)
    {
      // software spi
        GPIO_SetBits(SSD1306_CS_PORT, SSD1306_CS_GPIO);
        GPIO_ResetBits(SSD1306_DC_PORT, SSD1306_DC_GPIO);
        GPIO_ResetBits(SSD1306_CS_PORT, SSD1306_CS_GPIO);

        ssd1306_fastSPIwrite(c);
        GPIO_SetBits(SSD1306_CS_PORT, SSD1306_CS_GPIO);
    }
    else {
    		//i2c 
			control = 0x00;
			WriteCmd(_i2caddr, control, c);
			
    }
}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_startscrollright(uint8_t start, uint8_t stop) {
    ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X00);
    ssd1306_command(0XFF);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_startscrollleft(uint8_t start, uint8_t stop) {
    ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X00);
    ssd1306_command(0XFF);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}


// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_startscrolldiagright(uint8_t start, uint8_t stop) {
    ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
    ssd1306_command(0X00);
    ssd1306_command(SSD1306_LCDHEIGHT);
    ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X01);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_startscrolldiagleft(uint8_t start, uint8_t stop) {
    ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
    ssd1306_command(0X00);
    ssd1306_command(SSD1306_LCDHEIGHT);
    ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
    ssd1306_command(0X00);
    ssd1306_command(start);
    ssd1306_command(0X00);
    ssd1306_command(stop);
    ssd1306_command(0X01);
    ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

void ssd1306_stopscroll(void) {
    ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void ssd1306_dim(int8_t dim) {
    uint8_t contrast;

    if (dim) {
        contrast = 0; // Dimmed display
    }
    else {
        if (_vccstate == SSD1306_EXTERNALVCC) {
            contrast = 0x9F;
        }
        else {
            contrast = 0xCF;
        }
    }
    // the range of contrast to too small to be really useful
    // it is useful to dim the display
    ssd1306_command(SSD1306_SETCONTRAST);
    ssd1306_command(contrast);
}

void ssd1306_display(void) {
		uint16_t i;
	  uint8_t  control = 0x40;

		ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(0);   // Column start address (0 = reset)
    ssd1306_command(SSD1306_LCDWIDTH - 1); // Column end address (127 = reset)

    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(0); // Page start address (0 = reset)
#if SSD1306_LCDHEIGHT == 64
    ssd1306_command(7); // Page end address
#endif
#if SSD1306_LCDHEIGHT == 32
    ssd1306_command(3); // Page end address
#endif
#if SSD1306_LCDHEIGHT == 16
    ssd1306_command(1); // Page end address
#endif

    if ((hwSPI == HARDWARE_SPI) || (hwSPI == SOFTWARE_SPI))
    {
      // SPI
#ifdef HAVE_PORTREG
        //
#else
        GPIO_SetBits(SSD1306_CS_PORT, SSD1306_CS_GPIO);
        GPIO_SetBits(SSD1306_DC_PORT, SSD1306_DC_GPIO);
        GPIO_ResetBits(SSD1306_CS_PORT, SSD1306_CS_GPIO);
#endif

        for (i = 0; i < (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT / 8); i++) {
            ssd1306_fastSPIwrite(buffer[i]);
        }
#ifdef HAVE_PORTREG
        //*csport |= cspinmask;
#else
        GPIO_SetBits(SSD1306_CS_PORT, SSD1306_CS_GPIO);
#endif
    }
    else
    {
      // save I2C bitrate
#ifdef TWBR
        uint8_t twbrbackup = TWBR;
        TWBR = 12; // upgrade to 400KHz!
#endif

            //Serial.println(TWBR, DEC);
            //Serial.println(TWSR & 0x3, DEC);

                // I2C
        for (i = 0; i < (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT / 8); i++) {
          // send a bunch of data in one xmission
					//for(j = 0; j < 16; j++) {
							WriteDat(_i2caddr, control, buffer[i]);
					//}
            
        }
#ifdef TWBR
        TWBR = twbrbackup;
#endif
    }
}

// clear everything
void ssd1306_clearDisplay(void) {
    memset(buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT / 8));
}

void ssd1306_fastSPIwrite(uint8_t d) {

		uint8_t bit;
	
    if (hwSPI == HARDWARE_SPI) {
        //(void)SPI.transfer(d);
    }
    else {
        for (bit = 0x80; bit; bit >>= 1) {
#ifdef HAVE_PORTREG
            // spi
            
#else
            GPIO_ResetBits(SSD1306_SCK_PORT, SSD1306_SCK_GPIO);
            if (d & bit) GPIO_SetBits(SSD1306_DAT_PORT, SSD1306_DAT_GPIO);
            else         GPIO_ResetBits(SSD1306_DAT_PORT, SSD1306_DAT_GPIO);
            GPIO_SetBits(SSD1306_SCK_PORT, SSD1306_SCK_GPIO);
#endif
        }
    }
}

void ssd1306_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    BOOL bSwap = FALSE;
    switch (rotation) {
    case 0:
      // 0 degree rotation, do nothing
        break;
    case 1:
      // 90 degree rotation, swap x & y for rotation, then invert x
        bSwap = TRUE;
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
      // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        x -= (w - 1);
        break;
    case 3:
      // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
        bSwap = TRUE;
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        y -= (w - 1);
        break;
    }

    if (bSwap) {
        ssd1306_drawFastVLineInternal(x, y, w, color);
    }
    else {
        ssd1306_drawFastHLineInternal(x, y, w, color);
    }
}

void ssd1306_drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) {
	
		register uint8_t *pBuf;
		register uint8_t mask;
		// Do bounds/limit checks
    if (y < 0 || y >= HEIGHT) { return; }

      // make sure we don't try to draw below 0
    if (x < 0) {
        w += x;
        x = 0;
    }

      // make sure we don't go off the edge of the display
    if ((x + w) > WIDTH) {
        w = (WIDTH - x);
    }

      // if our width is now negative, punt
    if (w <= 0) { return; }

      // set up the pointer for  movement through the buffer
    pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    mask = 1 << (y & 7);

    switch (color)
    {
    case WHITE:         while (w--) { *pBuf++ |= mask; }
        ; break;
    case BLACK: mask = ~mask; while (w--) { *pBuf++ &= mask; }
        ; break;
    case INVERSE:         while (w--) { *pBuf++ ^= mask; }
        ; break;
    }
}

void ssd1306_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    BOOL bSwap = FALSE;
    switch (rotation) {
    case 0:
        break;
    case 1:
      // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
        bSwap = TRUE;
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        x -= (h - 1);
        break;
    case 2:
      // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        y -= (h - 1);
        break;
    case 3:
      // 270 degree rotation, swap x & y for rotation, then invert y
        bSwap = TRUE;
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    if (bSwap) {
        ssd1306_drawFastHLineInternal(x, y, h, color);
    }
    else {
        ssd1306_drawFastVLineInternal(x, y, h, color);
    }
}

void ssd1306_drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color) {

		register uint8_t y,h;
		register uint8_t *pBuf;
		register uint8_t mod;
		static uint8_t premask[8] = { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
		register uint8_t mask;
		static uint8_t postmask[8] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
		
		// do nothing if we're off the left or right side of the screen
    if (x < 0 || x >= WIDTH) { return; }

    // make sure we don't try to draw below 0
    if (__y < 0) {
        // __y is negative, this will subtract enough from __h to account for __y being 0
        __h += __y;
        __y = 0;

    }

    // make sure we don't go past the height of the display
    if ((__y + __h) > HEIGHT) {
        __h = (HEIGHT - __y);
    }

    // if our height is now negative, punt
    if (__h <= 0) {
        return;
    }

    // this display doesn't need ints for coordinates, use local byte registers for faster juggling
    y = __y;
    h = __h;


    // set up the pointer for fast movement through the buffer
    pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    mod = (y & 7);
    if (mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        
        mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if (h < mod) {
            mask &= (0XFF >> (mod - h));
        }

        switch (color)
        {
        case WHITE:   *pBuf |=  mask; break;
        case BLACK:   *pBuf &= ~mask; break;
        case INVERSE: *pBuf ^=  mask; break;
        }

        // fast exit if we're done here!
        if (h < mod) { return; }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH;
    }


    // write solid bytes while we can - effectively doing 8 rows at a time
    if (h >= 8) {
        if (color == INVERSE) {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
            do {
                *pBuf = ~(*pBuf);

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            } while (h >= 8);
        }
        else {
            // store a local value to work with
            register uint8_t val = (color == WHITE) ? 255 : 0;

            do {
                // write our value in
                *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            } while (h >= 8);
        }
    }

  // now do the final partial byte, if necessary
    if (h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        
        mask = postmask[mod];
        switch (color)
        {
        case WHITE:   *pBuf |=  mask; break;
        case BLACK:   *pBuf &= ~mask; break;
        case INVERSE: *pBuf ^=  mask; break;
        }
    }
}
