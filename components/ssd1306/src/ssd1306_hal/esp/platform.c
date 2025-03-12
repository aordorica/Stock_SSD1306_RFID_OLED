/*
    MIT License

    Copyright (c) 2018-2019, Alexey Dynda

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include "ssd1306_hal/io.h"

#if defined(SSD1306_ESP_PLATFORM)

#include "intf/ssd1306_interface.h"
#include "intf/spi/ssd1306_spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#if 1

#if defined(CONFIG_PLATFORM_SPI_AVAILABLE) && defined(CONFIG_PLATFORM_SPI_ENABLE)
static void platform_spi_send_cache();
#endif

// TODO: To complete support. Any help is welcome
int  digitalRead(int pin)   // digitalRead()
{
    return gpio_get_level(pin);
}

void digitalWrite(int pin, int level)  // digitalWrite()
{
#if defined(CONFIG_PLATFORM_SPI_AVAILABLE) && defined(CONFIG_PLATFORM_SPI_ENABLE)
    if (s_ssd1306_dc == pin)
    {
        platform_spi_send_cache();
    }
#endif
    gpio_set_level(pin, level);
}

void pinMode(int pin, int mode)
{
    if (mode == INPUT)
        gpio_set_direction(pin, GPIO_MODE_INPUT);
    else if (mode == OUTPUT)
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

uint32_t millis(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void delay(uint32_t ms)     // delay()
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

#endif

////////////////////////////////////////////////////////////////////////////////////////
// !!! PLATFORM I2C IMPLEMENTATION OPTIONAL !!!
#if defined(CONFIG_PLATFORM_I2C_AVAILABLE) && defined(CONFIG_PLATFORM_I2C_ENABLE)

#include <stdio.h>
#include "driver/i2c_master.h"   // New I2C Master Bus API header
#include "esp_log.h"
#include "intf/ssd1306_interface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "SSD1306_PLATFORM_NEW";
#define I2C_MASTER_TIMEOUT_MS (1000 / portTICK_PERIOD_MS)
i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_dev_handle_t i2c_dev = NULL;
static uint8_t s_i2c_addr = 0x3C;

/* Forward declarations for interface functions */
esp_err_t platform_i2c_send(uint8_t data);
esp_err_t platform_i2c_send_buffer(const uint8_t *data, uint16_t len);
void platform_i2c_close(void);

/**
 * @brief Send a single byte over I2C using the new API.
 *
 * @param data The byte to send.
 * @return esp_err_t result of the transaction.
 */
esp_err_t platform_i2c_send(uint8_t data) {
    ESP_LOGI(TAG, "Sending data %02x", data);
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev, &data, 1, I2C_MASTER_TIMEOUT_MS));
    return ESP_OK;
}

/**
 * @brief Send a buffer of data over I2C using the new API.
 *
 * @param data Pointer to the data buffer.
 * @param len  Length of the data buffer.
 * @return esp_err_t result of the transaction.
 */
esp_err_t platform_i2c_send_buffer(const uint8_t *data, uint16_t len) {
    ESP_LOG_BUFFER_HEX(TAG, data, len);
    ESP_LOG_BUFFER_CHAR(TAG, data, len);
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev, data, len, I2C_MASTER_TIMEOUT_MS));
    return ESP_OK;
}

/**
 * @brief Close the I2C bus by removing the device and deleting the bus.
 */
void platform_i2c_close(void) {
    if (i2c_dev) {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(i2c_dev));
        i2c_dev = NULL;
    }
    if (i2c_bus) {
        ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus));
        i2c_bus = NULL;
    }
    ESP_LOGI(TAG, "I2C bus removed");
}


/**
 * @brief Initialize the I2C bus using the new I2C Master Bus API
 *        and configure the SSD1306 interface.
 *
 * @param busId Unused if you always use a fixed bus (e.g. I2C_NUM_0); otherwise, pass the desired bus.
 * @param addr  I2C address of the SSD1306 display.
 * @param cfg   Pointer to a configuration structure that contains the SDA and SCL pin numbers.
 */
void ssd1306_platform_i2cInit(int8_t busId, uint8_t addr, ssd1306_platform_i2cConfig_t *cfg) {
    ESP_LOGI(TAG, "ssd1306_platform_i2cInit: busId: %d, addr: %02x", busId, addr);
    // Set the I2C address if provided
    if (addr) s_i2c_addr = addr;
    
    // Set up the interface function pointers for I2C communication
    ssd1306_intf.spi = 0;
    ssd1306_intf.send  = &platform_i2c_send;
    ssd1306_intf.send_buffer = &platform_i2c_send_buffer;
    ssd1306_intf.close = &platform_i2c_close;
    
    // Set up the I2C bus configuration
    ESP_LOGI(TAG, "Initializing I2C bus using new API");
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,  // Ensure busId is one of I2C_NUM_0, I2C_NUM_1, etc.
        .sda_io_num = cfg->sda,
        .scl_io_num = cfg->scl,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags.enable_internal_pullup = true,
    };

    // Initialize the I2C bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    ESP_LOGI(TAG, "I2C master bus created");

    // Set up the I2C device configuration for the SSD1306
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = s_i2c_addr,
        .scl_speed_hz    = 400000,  // I2C clock speed (400 kHz)
    };

    // Add the device to the I2C bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &i2c_dev));
    ESP_LOGI(TAG, "I2C device added");

    // If everything is successful, log the initialization
    ESP_LOGI(TAG, "I2C bus and device initialized successfully");
}

#endif

////////////////////////////////////////////////////////////////////////////////////////
// !!! PLATFORM SPI IMPLEMENTATION OPTIONAL !!!
#if defined(CONFIG_PLATFORM_SPI_AVAILABLE) && defined(CONFIG_PLATFORM_SPI_ENABLE)

#include "intf/spi/ssd1306_spi.h"
#include "driver/spi_master.h"

// Store spi handle globally for all spi callbacks
static spi_device_handle_t s_spi;
static int8_t s_spi_bus_id;
// s_first_spi_session is used for delayed spi initialization.
// Some oled displays have slow max SPI speed, so display init function can change
// spi frequency s_ssd1306_spi_clock. Register device, only when frequency is known.
static uint8_t s_first_spi_session = 0;
static uint8_t s_spi_cache[1024];
static int s_spi_cached_count = 0;

static void platform_spi_send_cache()
{
    /* TODO: Yeah, sending single bytes is too slow, but *
     * need to figure out how to detect data/command bytes *
     * to send bytes as one block */
    uint8_t *data = s_spi_cache;
    while (s_spi_cached_count)
    {
        size_t sz = s_spi_cached_count > 32 ? 32: s_spi_cached_count;
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length=8*sz;          // 8 bits
        t.tx_buffer=data;
        // ... Send byte to spi communication channel
        // We do not care here about DC line state, because
        // ssd1306 library already set DC pin via ssd1306_spiDataMode() before call to send().
        spi_device_transmit(s_spi, &t);
        data+=sz;
        s_spi_cached_count-=sz;
    }
    s_spi_cached_count = 0;
}

static void platform_spi_start(void)
{
    // ... Open spi channel for your device with specific s_ssd1306_cs, s_ssd1306_dc
    if (s_first_spi_session)
    {
        spi_device_interface_config_t devcfg=
        {
            .clock_speed_hz = s_ssd1306_spi_clock,
            .mode=0,
            .spics_io_num=s_ssd1306_cs,
            .queue_size=7,       // max 7 transactions at a time
        };
        spi_bus_add_device(s_spi_bus_id ? SPI3_HOST : SPI2_HOST, &devcfg, &s_spi);
        s_first_spi_session = 0;
    }
    s_spi_cached_count = 0;
}

static void platform_spi_stop(void)
{
    // ... Complete spi communication
    // no complete actions required for this implementation
    platform_spi_send_cache();
}

static void platform_spi_send(uint8_t data)
{
    s_spi_cache[s_spi_cached_count] = data;
    s_spi_cached_count++;
    if ( s_spi_cached_count >= sizeof( s_spi_cache ) )
    {
        platform_spi_send_cache();
    }
}

static void platform_spi_close(void)
{
    // ... free all spi resources here
    if (!s_first_spi_session)
    {
        spi_bus_remove_device( s_spi );
    }
    spi_bus_free( s_spi_bus_id ? SPI3_HOST : SPI2_HOST );
}

static void platform_spi_send_buffer(const uint8_t *data, uint16_t len)
{
    while (len--)
    {
        platform_spi_send(*data);
        data++;
    }
}

void ssd1306_platform_spiInit(int8_t busId,
                              int8_t cesPin,
                              int8_t dcPin)
{
    // Use VSPI by default
    if (busId < 0) busId = 1;
    s_spi_bus_id = busId;

    // If cesPin is not provided, select by default
    if (cesPin < 0)
    {
        cesPin = s_spi_bus_id ? 5 : 15;
    }
    s_ssd1306_cs = cesPin;
    if (dcPin>=0) s_ssd1306_dc = dcPin;

    if (cesPin >=0) pinMode(cesPin, OUTPUT);
    if (dcPin >= 0) pinMode(dcPin, OUTPUT);

    ssd1306_intf.spi = 1;
    ssd1306_intf.start = &platform_spi_start;
    ssd1306_intf.stop  = &platform_spi_stop;
    ssd1306_intf.send  = &platform_spi_send;
    ssd1306_intf.close = &platform_spi_close;
    ssd1306_intf.send_buffer = &platform_spi_send_buffer;

    // init your interface here
    spi_bus_config_t buscfg=
    {
        .miso_io_num= s_spi_bus_id ? 19 : 12,
        .mosi_io_num= s_spi_bus_id ? 23 : 13,
        .sclk_io_num= s_spi_bus_id ? 18 : 14,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=32
    };
    spi_bus_initialize(s_spi_bus_id ? SPI3_HOST : SPI2_HOST, &buscfg, 0); // 0 -no dma
    s_first_spi_session = 1;
}
#endif

#endif // SSD1306_ESP_PLATFORM
