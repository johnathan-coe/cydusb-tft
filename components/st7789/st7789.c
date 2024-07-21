#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "st7789.h"

#define TAG "ST7789"
#define	_DEBUG_ 0

#if CONFIG_SPI2_HOST
#define HOST_ID SPI2_HOST
#elif CONFIG_SPI3_HOST
#define HOST_ID SPI3_HOST
#endif

static const int SPI_Command_Mode = 0;
static const int SPI_Data_Mode = 1;

void delayMS(int ms) {
	int _ms = ms + (portTICK_PERIOD_MS - 1);
	TickType_t xTicksToDelay = _ms / portTICK_PERIOD_MS;
	ESP_LOGD(TAG, "ms=%d _ms=%d portTICK_PERIOD_MS=%"PRIu32" xTicksToDelay=%"PRIu32,ms,_ms,portTICK_PERIOD_MS,xTicksToDelay);
	vTaskDelay(xTicksToDelay);
}

void spi_master_init(TFT_t * dev, int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS, int16_t GPIO_DC, int16_t GPIO_RESET, int16_t GPIO_BL, int clock_speed_hz)
{
	esp_err_t ret;

	ESP_LOGI(TAG, "GPIO_CS=%d",GPIO_CS);
	if ( GPIO_CS >= 0 ) {
		gpio_reset_pin( GPIO_CS );
		gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_CS, 0 );
	}

	ESP_LOGI(TAG, "GPIO_DC=%d",GPIO_DC);
	gpio_reset_pin( GPIO_DC );
	gpio_set_direction( GPIO_DC, GPIO_MODE_OUTPUT );
	gpio_set_level( GPIO_DC, 0 );

	ESP_LOGI(TAG, "GPIO_RESET=%d",GPIO_RESET);
	if ( GPIO_RESET >= 0 ) {
		gpio_reset_pin( GPIO_RESET );
		gpio_set_direction( GPIO_RESET, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_RESET, 1 );
		delayMS(100);
		gpio_set_level( GPIO_RESET, 0 );
		delayMS(100);
		gpio_set_level( GPIO_RESET, 1 );
		delayMS(100);
	}

	ESP_LOGI(TAG, "GPIO_BL=%d",GPIO_BL);
	if ( GPIO_BL >= 0 ) {
		gpio_reset_pin(GPIO_BL);
		gpio_set_direction( GPIO_BL, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_BL, 0 );
	}

	ESP_LOGI(TAG, "GPIO_MOSI=%d",GPIO_MOSI);
	ESP_LOGI(TAG, "GPIO_SCLK=%d",GPIO_SCLK);
	spi_bus_config_t buscfg = {
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = -1,
		.sclk_io_num = GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0
	};

	ret = spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO );
	ESP_LOGD(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg));

	devcfg.clock_speed_hz = clock_speed_hz;
	devcfg.queue_size = 7;
	devcfg.mode = 3;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

	if ( GPIO_CS >= 0 ) {
		devcfg.spics_io_num = GPIO_CS;
	} else {
		devcfg.spics_io_num = -1;
	}
	
	spi_device_handle_t handle;
	ret = spi_bus_add_device(HOST_ID, &devcfg, &handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);

	dev->_dc = GPIO_DC;
	dev->_bl = GPIO_BL;
	dev->_SPIHandle = handle;
}

bool spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t* Data, size_t DataLength)
{
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;

		ret = spi_device_transmit(SPIHandle, &SPITransaction); // Could use polling transmit here
		assert(ret==ESP_OK); 
	}

	return true;
}

bool spi_master_write_command(TFT_t * dev, uint8_t cmd)
{
	static uint8_t Byte = 0;
	Byte = cmd;
	gpio_set_level( dev->_dc, SPI_Command_Mode );
	return spi_master_write_byte( dev->_SPIHandle, &Byte, 1 );
}

bool spi_master_write_data_byte(TFT_t * dev, uint8_t data)
{
	static uint8_t Byte = 0;
	Byte = data;
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, &Byte, 1 );
}

bool spi_master_write_addr(TFT_t * dev, uint16_t addr1, uint16_t addr2)
{
	static uint8_t Byte[4];
	Byte[0] = (addr1 >> 8) & 0xFF;
	Byte[1] = addr1 & 0xFF;
	Byte[2] = (addr2 >> 8) & 0xFF;
	Byte[3] = addr2 & 0xFF;
	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, 4);
}

bool spi_master_write_color(TFT_t * dev, uint16_t color, uint16_t size)
{
	static uint8_t Byte[1024];
	int index = 0;
	
	for(int i = 0; i < size; i++)
	{
		Byte[index++] = (color >> 8) & 0xFF;
		Byte[index++] = color & 0xFF;
	}

	gpio_set_level( dev->_dc, SPI_Data_Mode );
	return spi_master_write_byte( dev->_SPIHandle, Byte, size*2);
}

bool spi_master_write_colors(TFT_t* dev, uint8_t* colors, size_t size)
{
	gpio_set_level(dev->_dc, SPI_Data_Mode);
	return spi_master_write_byte(dev->_SPIHandle, colors, size);
}

void lcdInit(TFT_t * dev, int width, int height, int offsetx, int offsety)
{
	dev->_width = width;
	dev->_height = height;
	dev->_offsetx = offsetx;
	dev->_offsety = offsety;

	spi_master_write_command(dev, 0x01);	//Software Reset
	delayMS(150);

	spi_master_write_command(dev, 0x11);	//Sleep Out
	delayMS(255);
	
	spi_master_write_command(dev, 0x3A);	//Interface Pixel Format
	spi_master_write_data_byte(dev, 0x55);
	delayMS(10);
	
	spi_master_write_command(dev, 0x36);	//Memory Data Access Control
	spi_master_write_data_byte(dev, 0x00);

	spi_master_write_command(dev, 0x2A);	//Column Address Set
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0xF0);

	spi_master_write_command(dev, 0x2B);	//Row Address Set
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_data_byte(dev, 0xF0);

	spi_master_write_command(dev, 0x13);	//Normal Display Mode On
	delayMS(10);

	spi_master_write_command(dev, 0x29);	//Display ON
	delayMS(255);

	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 1 );
	}

	// This TFT uses RBG 565, which means 2 bytes per pixel
	dev->_frame_buffer = heap_caps_malloc(sizeof(uint8_t) * width * height * 2, MALLOC_CAP_DMA);

	if (dev->_frame_buffer == NULL)
	{
		ESP_LOGE(TAG, "heap_caps_malloc fail");
	}
	else
	{
		ESP_LOGI(TAG, "heap_caps_malloc success");
	}
}

void lcdDisplayOff(TFT_t * dev) {
	spi_master_write_command(dev, 0x28);
}
 
void lcdDisplayOn(TFT_t * dev) {
	spi_master_write_command(dev, 0x29);
}

void lcdBacklightOff(TFT_t * dev) {
	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 0 );
	}
}

void lcdBacklightOn(TFT_t * dev) {
	if(dev->_bl >= 0) {
		gpio_set_level( dev->_bl, 1 );
	}
}

void lcdInversionOff(TFT_t * dev) {
	spi_master_write_command(dev, 0x20);
}

void lcdInversionOn(TFT_t * dev) {
	spi_master_write_command(dev, 0x21);
}

void lcdDrawPixel(TFT_t * dev, uint16_t x, uint16_t y, uint16_t color)
{
	if (x >= dev->_width)
		return;

	if (y >= dev->_height)
		return;

	size_t buffer_offset = (y * dev->_width * 2) + x * 2;

	// Split RGB 565 into two bytes
	dev->_frame_buffer[buffer_offset] = (color >> 8) & 0xFF;
	dev->_frame_buffer[buffer_offset + 1] = color & 0xFF;
}

void lcdDrawPixels(TFT_t * dev, uint16_t x, uint16_t y, uint16_t size, uint16_t* colors) {
	if (x >= dev->_width)
	    return;
	
	if (y >= dev->_height)
		return;

	size_t buffer_offset = (y * dev->_width * 2) + x * 2;
	if (buffer_offset >= (dev->_height * dev->_width * 2) + dev->_width * 2)
	{
		return;
	}

	for (uint16_t i = 0; i < size; i++)
	{
		// Split RGB 565 into two bytes
		dev->_frame_buffer[buffer_offset] = (colors[i] >> 8) & 0xFF;
		dev->_frame_buffer[buffer_offset + 1] = colors[i] & 0xFF;
		buffer_offset += 2;
	}
}

void lcdDrawFinish(TFT_t *dev)
{
	spi_master_write_command(dev, 0x2A); // set column(x) address
	spi_master_write_addr(dev, dev->_offsetx, dev->_offsetx+dev->_width-1);
	spi_master_write_command(dev, 0x2B); // set Page(y) address
	spi_master_write_addr(dev, dev->_offsety, dev->_offsety+dev->_height-1);
	spi_master_write_command(dev, 0x2C); // Memory Write

	uint32_t size = dev->_width * dev->_height * 2;
	uint8_t *image = dev->_frame_buffer;

	while (size > 0)
	{
		uint16_t chunkSize = (size > 512) ? 512 : size;
		spi_master_write_colors(dev, image, chunkSize);

		size -= chunkSize;
		image += chunkSize;
	}

	return;
}
