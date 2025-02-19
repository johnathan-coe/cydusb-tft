#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "st7789.h"

#define LOG_TAG "ST7789"

#define TRANS_QUEUE_LENGTH 32
#define TRANS_DATA_BYTES 4096

#if CONFIG_SPI2_HOST
	#define HOST_ID SPI2_HOST
#elif CONFIG_SPI3_HOST
	#define HOST_ID SPI3_HOST
#endif

typedef enum
{
	COMMAND_MODE = 0,
	DATA_MODE = 1
} SPI_MODE_t;

enum command
{
	SWRESET = 0x01,
	SLEEP_OUT = 0x11,
	NORMAL_DISPLAY_MODE = 0x13,
	INVERSION_OFF = 0x20,
	INVERSION_ON = 0x21,
	COLUMN_ADDRESS_SET = 0x2A,
	ROW_ADDRESS_SET = 0x2B,
	MEMORY_WRITE = 0x2C,
	DISPLAY_OFF = 0x28,
	DISPLAY_ON = 0x29,
	SET_RGB_INTERFACE = 0x3A,
	SET_MEMORY_DATA_ACCESS = 0x36
};

void delayMS(int ms) {
	int _ms = ms + (portTICK_PERIOD_MS - 1);
	TickType_t xTicksToDelay = _ms / portTICK_PERIOD_MS;
	vTaskDelay(xTicksToDelay);
}

void spi_master_init(TFT_t * dev, int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS, int16_t GPIO_DC, int16_t GPIO_RESET, int16_t GPIO_BL, int clock_speed_hz)
{
	esp_err_t ret;
	
	if (GPIO_CS >= 0)
	{
		gpio_reset_pin( GPIO_CS );
		gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_CS, 0 );
	}

	gpio_reset_pin( GPIO_DC );
	gpio_set_direction( GPIO_DC, GPIO_MODE_OUTPUT );
	gpio_set_level( GPIO_DC, 0 );

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

	if ( GPIO_BL >= 0 ) {
		gpio_reset_pin(GPIO_BL);
		gpio_set_direction( GPIO_BL, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_BL, 0 );
	}

	spi_bus_config_t buscfg = {
		.mosi_io_num = GPIO_MOSI,
		.sclk_io_num = GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = TRANS_DATA_BYTES,
		.flags = 0
	};

	ret = spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO );
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg));

	devcfg.clock_speed_hz = clock_speed_hz;
	devcfg.queue_size = TRANS_QUEUE_LENGTH;
	devcfg.mode = 3;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

	if (GPIO_CS >= 0)
	{
		devcfg.spics_io_num = GPIO_CS;
	}
	else
	{
		devcfg.spics_io_num = -1;
	}
	
	spi_device_handle_t handle;
	ret = spi_bus_add_device(HOST_ID, &devcfg, &handle);
	assert(ret==ESP_OK);

	dev->_dc = GPIO_DC;
	dev->_bl = GPIO_BL;
	dev->_handle = handle;
}

bool spi_master_write_bytes(spi_device_handle_t SPIHandle, uint8_t* data, size_t length)
{
	if (length == 0)
		return true;

	if (length <= TRANS_DATA_BYTES)
	{
		spi_transaction_t transaction;
		memset(&transaction, 0, sizeof(spi_transaction_t));

		transaction.length = length * 8;
		transaction.tx_buffer = data;

		esp_err_t ret = spi_device_polling_transmit(SPIHandle, &transaction);
		assert(ret == ESP_OK);
		return true;
	}
	
	spi_transaction_t transRing[TRANS_QUEUE_LENGTH];
	size_t ringIdx = 0;
	bool ringIdxOverflowed = false;

	spi_device_acquire_bus(SPIHandle, portMAX_DELAY);

	uint8_t* dataOffset = data;
	while (length > 0)
	{
		// Wait for transaction in ring
		if (ringIdxOverflowed)
		{
			spi_transaction_t* transComplete;
			spi_device_get_trans_result(SPIHandle, &transComplete, portMAX_DELAY);
		}

		// Determine the number of bytes to send in this transaction
		uint16_t chunkSize = (length > TRANS_DATA_BYTES)
			? TRANS_DATA_BYTES
			: length;

		// Send a new transaction
		memset(&transRing[ringIdx], 0, sizeof(spi_transaction_t));
		transRing[ringIdx].length = chunkSize * 8;
		transRing[ringIdx].tx_buffer = dataOffset;
		spi_device_queue_trans(SPIHandle, &transRing[ringIdx], portMAX_DELAY);

		ringIdx++;
		ringIdxOverflowed |= ringIdx == TRANS_QUEUE_LENGTH;
		ringIdx %= TRANS_QUEUE_LENGTH;

		dataOffset += chunkSize;
		length -= chunkSize;
	}

	size_t waitRangeIdx = ringIdxOverflowed
		? TRANS_QUEUE_LENGTH
		: ringIdx;

	for (int i = 0; i < waitRangeIdx; i++)
	{
		spi_transaction_t* transComplete;
		spi_device_get_trans_result(SPIHandle, &transComplete, portMAX_DELAY);
	}
	
	spi_device_release_bus(SPIHandle);
	return true;
}

void spi_set_mode(TFT_t* dev, SPI_MODE_t mode)
{
	gpio_set_level(dev->_dc, mode);
}

bool spi_master_write_command(TFT_t * dev, uint8_t cmd)
{
	spi_set_mode(dev, COMMAND_MODE);
	return spi_master_write_bytes(dev->_handle, &cmd, 1);
}

bool spi_master_write_data_byte(TFT_t * dev, uint8_t data)
{
	spi_set_mode(dev, DATA_MODE);
	return spi_master_write_bytes(dev->_handle, &data, 1);
}

bool spi_master_write_addr(TFT_t * dev, uint16_t addr1, uint16_t addr2)
{
	static uint8_t bytes[4];
	bytes[0] = (addr1 >> 8) & 0xFF;
	bytes[1] = addr1 & 0xFF;
	bytes[2] = (addr2 >> 8) & 0xFF;
	bytes[3] = addr2 & 0xFF;

	spi_set_mode(dev, DATA_MODE);
	return spi_master_write_bytes(dev->_handle, bytes, 4);
}

void lcdInit(TFT_t * dev, rgb_interface rgb)
{
	dev->_bytesPerPixel = ((rgb & CIC18BPP) == CIC18BPP)
		? 3
		: 2;

	spi_master_write_command(dev, SWRESET);
	spi_master_write_command(dev, SLEEP_OUT);	
	spi_master_write_command(dev, SET_RGB_INTERFACE);
	spi_master_write_data_byte(dev, rgb);	
	spi_master_write_command(dev, SET_MEMORY_DATA_ACCESS);
	spi_master_write_data_byte(dev, 0x00);
	spi_master_write_command(dev, NORMAL_DISPLAY_MODE);
	lcdDisplayOn(dev);
	lcdBacklightOn(dev);
}

void lcdDisplayOff(TFT_t * dev) {
	spi_master_write_command(dev, DISPLAY_OFF);
}
 
void lcdDisplayOn(TFT_t * dev) {
	spi_master_write_command(dev, DISPLAY_ON);
}

void lcdBacklightOff(TFT_t * dev) {
	if (dev->_bl < 0)
		return;

	gpio_set_level( dev->_bl, 0 );
}

void lcdBacklightOn(TFT_t * dev) {
	if (dev->_bl < 0)
		return;
	
	gpio_set_level( dev->_bl, 1 );
}

void lcdInversionOff(TFT_t * dev) {
	spi_master_write_command(dev, INVERSION_OFF);
}

void lcdInversionOn(TFT_t * dev) {
	spi_master_write_command(dev, INVERSION_ON);
}

void lcdDrawPixels(TFT_t* dev, uint16_t x, uint16_t y, uint16_t x1, uint16_t y1, uint8_t* colors, size_t length)
{
	spi_master_write_command(dev, COLUMN_ADDRESS_SET);
	spi_master_write_addr(dev, x, x1);

	spi_master_write_command(dev, ROW_ADDRESS_SET);
	spi_master_write_addr(dev, y, y1);
	
	spi_master_write_command(dev, MEMORY_WRITE);
	spi_set_mode(dev, DATA_MODE);

	spi_master_write_bytes(dev->_handle, colors, length);
}
