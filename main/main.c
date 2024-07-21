#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "st7789.h"

void ST7789(void *pvParameters)
{
	TFT_t dev;
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO, 55000000);
	lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);
	lcdInversionOff(&dev);

	bool a = true;
	while (true)
	{
		lcdFill(&dev, 0);

		for (int x = 0; x < 100; x++)
		{
			for (int y = 0; y < 100; y++)
			{
				lcdDrawPixel(&dev, x, y, a ? rgb565(0, 255, 0) : rgb565(255, 0, 0));
			}
		}

		for (int x = 100; x < 200; x++)
		{
			for (int y = 100; y < 200; y++)
			{
				lcdDrawPixel(&dev, x, y, a ? rgb565(255, 0, 0) : rgb565(0, 255, 0));
			}
		}

		for (int x = 0; x < 100; x++)
		{
			for (int y = 200; y < 300; y++)
			{
				lcdDrawPixel(&dev, x, y, a ? rgb565(0, 255, 0) : rgb565(255, 0, 0));
			}
		}

		lcdDrawFinish(&dev);
		delayMS(1000);
		a = !a;
	}
}

void app_main(void)
{
	xTaskCreate(ST7789, "ST7789", 1024 * 6, NULL, 0, NULL);
}
