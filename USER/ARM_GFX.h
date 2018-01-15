#ifndef _ARM_GFX_H_

#include "stm32f10x.h"
#include "stm32f10x.h"
#include "glcdfont.c"
#include "gfxfont.h"
#include "stdlib.h"

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

void ARM_GFX(int16_t w, int16_t h); // Constructor

// This MUST be defined by the subclass:
//void drawPixel(int16_t x, int16_t y, uint16_t color) = 0;

// TRANSACTION API / CORE DRAW API
// These MAY be overridden by the subclass to provide device-specific
// optimized code.  Otherwise 'generic' versions are used.
void startWrite(void);
void writePixel(int16_t x, int16_t y, uint16_t color);
void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void endWrite(void);

// CONTROL API
// These MAY be overridden by the subclass to provide device-specific
// optimized code.  Otherwise 'generic' versions are used.
void setRotation(uint8_t r);
void invertDisplay(int8_t i);

// BASIC DRAW API
// These MAY be overridden by the subclass to provide device-specific
// optimized code.  Otherwise 'generic' versions are used.
void
  // It's good to implement those, even if using transaction API
  drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
  drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
  fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color),
  fillScreen(uint16_t color),
  // Optional and probably not necessary to change
  drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color),
  drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

// These exist only with Adafruit_GFX (no subclass overrides)
void
    drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color),
    drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
      uint16_t color),
    fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color),
    fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
      int16_t delta, uint16_t color),
    drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
      int16_t x2, int16_t y2, uint16_t color),
    fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
      int16_t x2, int16_t y2, uint16_t color),
    drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
      int16_t radius, uint16_t color),
    fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
      int16_t radius, uint16_t color),
    drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
      int16_t w, int16_t h, uint16_t color),
    drawBitmap_bg(int16_t x, int16_t y, const uint8_t bitmap[],
      int16_t w, int16_t h, uint16_t color, uint16_t bg),
    drawBitmap__(int16_t x, int16_t y, uint8_t *bitmap,
      int16_t w, int16_t h, uint16_t color),
    drawBitmap___(int16_t x, int16_t y, uint8_t *bitmap,
      int16_t w, int16_t h, uint16_t color, uint16_t bg),
    drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
      int16_t w, int16_t h, uint16_t color),
    drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
      int16_t w, int16_t h),
    drawGrayscaleBitmap_(int16_t x, int16_t y, uint8_t *bitmap,
      int16_t w, int16_t h),
    drawGrayscaleBitmap__(int16_t x, int16_t y,
      const uint8_t bitmap[], const uint8_t mask[],
      int16_t w, int16_t h),
    drawGrayscaleBitmap___(int16_t x, int16_t y,
      uint8_t *bitmap, uint8_t *mask, int16_t w, int16_t h),
    drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[],
      int16_t w, int16_t h),
    drawRGBBitmap1(int16_t x, int16_t y, uint16_t *bitmap,
      int16_t w, int16_t h),
    drawRGBBitmap2(int16_t x, int16_t y,
      const uint16_t bitmap[], const uint8_t mask[],
      int16_t w, int16_t h),
    drawRGBBitmap3(int16_t x, int16_t y,
      uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h),
    drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color,
      uint16_t bg, uint8_t size),
    setCursor(int16_t x, int16_t y),
    setTextColor(uint16_t c),
    setTextColorbg(uint16_t c, uint16_t bg),
    setTextSize(uint8_t s),
    setTextWrap(int8_t w),
    cp437(int8_t x),
    setFont(const GFXfont *f);


void write(uint8_t);
void write_string(char *text);
int16_t height(void);
int16_t width(void);

uint8_t getRotation(void);


#endif // !_ARM_GFX_H_

