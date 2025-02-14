#include "string.h"
#include "time.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_random.h"

#include "st7789.h"

#include "jcfx.h"
#include "jcfnt.h"

char* fire_init(int width, int height)
{
	char* fireBuf = malloc(width * height);
	memset(fireBuf, 0, width * (height - 1));
	memset(&fireBuf[width * (height - 1)], 36, width);
	return fireBuf;
}

void fire_iter(struct JCCTX_t* ctx, char* fire, int width, int height)
{
	jcfx_draw_rect(ctx, JCFX_BLACK, 0, 0, width, height);
	
	JCFXCOLOURBYTES_t palette[36];
	for (int i = 0; i < 36; i++)
	{
		JCFXCOLOUR_t colour =
		{
			.r = i * 5,
			.g = 0,
			.b = 0
		};

		palette[i] = jcfx_convert_colour_to_bytes(ctx, colour);
	}

	size_t idx = 0;
	for (size_t y = 0; y < height - 1; y++)
	{
		for (size_t x = 0; x < width; x++)
		{
			size_t from = idx + width;
			if (!fire[from])
			{
				fire[idx] = 0;
				idx++;
				continue;
			}

			char r = rand() % 3;
			char intensity = fire[from] - (r & 1);

			fire[idx] = intensity;
			jcfx_write_colour_bytes(ctx, palette[intensity - 1], x, y);
			idx++;
    	}
	}
}

void ST7789(void *pvParameters)
{
	srand(time(NULL));

	TFT_t dev;
	
	rgb_interface colorMode = RGB262K | CIC16BPP;

	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO, 20000000);
	lcdInit(&dev, colorMode);
	lcdInversionOff(&dev);

	struct JCCTX_t* ctx = jcfx_init(&dev, 240, 320, colorMode);

	time_t tmLastFrame = 0;
	uint16_t frameCount = 0;
	uint16_t frames = 0;
	char* fpsString = "";
	char* freeMemString = "";

	char* fireBuf = fire_init(240, 320);
	while (true)
	{
		fire_iter(ctx, fireBuf, 240, 320);
		jcfnt_write(ctx, fpsString, JCFX_WHITE, 0, 0, 240);
		jcfnt_write(ctx, freeMemString, JCFX_WHITE, 48, 0, 240);
		jcfx_end_frame(ctx);

		time_t tmNow;
		time(&tmNow);

		frameCount++;

		if (tmNow > tmLastFrame)
		{
			frames = frameCount;
			frameCount = 0;
			tmLastFrame = tmNow;

			//TODO: list of chnaged y as bools, write blocks

			char ones = frames % 10;
			char tens = (frames % 100) / 10;
			fpsString = (char[]) { '0' + tens, '0' + ones, 'F', 'P', 'S', 0 };

			size_t freeBytes = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
			freeBytes /= 1024;
			ones = freeBytes % 10;
			tens = (freeBytes % 100) / 10;
			freeMemString = (char[]) { '0' + tens, '0' + ones, 'k', ' ', 'F', 'r', 'e', 'e', 0 };
		}
	}
}

void app_main(void)
{
	xTaskCreate(ST7789, "ST7789", 1024 * 6, NULL, 0, NULL);
}
