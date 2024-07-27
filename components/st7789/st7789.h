#ifndef MAIN_ST7789_H_
#define MAIN_ST7789_H_

#include "driver/spi_master.h"

#define rgb565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

typedef struct {
	int16_t _dc;
	int16_t _bl;
	spi_device_handle_t _SPIHandle;
} TFT_t;

void spi_master_init(TFT_t * dev, int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS, int16_t GPIO_DC, int16_t GPIO_RESET, int16_t GPIO_BL, int clock_speed_hz);

void delayMS(int ms);
void lcdInit(TFT_t * dev);

void lcdDisplayOff(TFT_t * dev);
void lcdDisplayOn(TFT_t * dev);
void lcdBacklightOff(TFT_t * dev);
void lcdBacklightOn(TFT_t * dev);
void lcdInversionOff(TFT_t * dev);
void lcdInversionOn(TFT_t * dev);

void lcdDrawPixels(TFT_t* dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t* colors);

#endif /* MAIN_ST7789_H_ */

