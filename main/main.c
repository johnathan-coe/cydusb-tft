#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "st7789.h"
#include "jcfx.h"

#include "font8x8_basic.h"

void ST7789(void *pvParameters)
{
	TFT_t dev;
	
	rgb_interface colorMode = RGB262K | CIC16BPP;

	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO, 55000000);
	lcdInit(&dev, colorMode);
	lcdInversionOff(&dev);

	struct JCCTX_t* ctx = jcfx_init(&dev, 240, 320, colorMode);

	jcfx_draw_rect(ctx, JCFX_BLACK, 0, 0, 240, 320);
	jcfx_end_frame(ctx);

	char* text = "I love you Sammy";
	JCFXCOLOUR_t rgb[] = { JCFX_RED, JCFX_GREEN, JCFX_BLUE };

	for (size_t i = 0; i < strlen(text); i++)
	{
		jcfx_begin_frame(ctx);
		jcfx_blit(ctx, rgb[i % 3], i * 8 + 10, 10, font8x8_basic[(size_t) text[i]], 8);
		jcfx_end_frame(ctx);
	}

	while (true)
	{
		delayMS(1000);
	}
}

void app_main(void)
{
	xTaskCreate(ST7789, "ST7789", 1024 * 6, NULL, 0, NULL);
}
