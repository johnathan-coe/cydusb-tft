#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "st7789.h"

void ST7789(void *pvParameters)
{
	TFT_t dev;
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO, 55000000);
	lcdInit(&dev, RGB262K | CIC16BPP);
	lcdInversionOff(&dev);

	uint8_t buf[1800];
	memset(buf, 0, 1800);

	bool a = true;
	while (true)
	{
		TickType_t lastFrame = xTaskGetTickCount();
		lcdDrawPixels(&dev, 30, 30, 30, 30, buf);
		TickType_t thisFrame = xTaskGetTickCount();

		TickType_t diff = thisFrame - lastFrame;
		ESP_LOGI("MAIN", "%f", 1000.0f / (diff * portTICK_PERIOD_MS));
		a = !a;
	}
}

void app_main(void)
{
	xTaskCreate(ST7789, "ST7789", 1024 * 6, NULL, 0, NULL);
}
