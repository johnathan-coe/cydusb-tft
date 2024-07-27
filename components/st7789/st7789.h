#ifndef MAIN_ST7789_H_
#define MAIN_ST7789_H_

#include "driver/spi_master.h"

#define rgb565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

typedef enum
{
	RGB65K =   0b01010000,
	RGB262K =  0b01100000,
	CIC12BPP = 0b00000011,
	CIC16BPP = 0b00000101,
	CIC18BPP = 0b00000110,
} rgb_interface;

typedef struct {
	int16_t _dc;
	int16_t _bl;
	uint8_t _bytesPerPixel;
	spi_device_handle_t _handle;
} TFT_t;

void spi_master_init(TFT_t * dev, int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS, int16_t GPIO_DC, int16_t GPIO_RESET, int16_t GPIO_BL, int clock_speed_hz);

void delayMS(int ms);
void lcdInit(TFT_t * dev, rgb_interface rgb);

void lcdDisplayOff(TFT_t * dev);
void lcdDisplayOn(TFT_t * dev);
void lcdBacklightOff(TFT_t * dev);
void lcdBacklightOn(TFT_t * dev);
void lcdInversionOff(TFT_t * dev);
void lcdInversionOn(TFT_t * dev);

void lcdDrawPixels(TFT_t* dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t* colors);

#endif /* MAIN_ST7789_H_ */

