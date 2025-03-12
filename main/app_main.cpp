#include <stdio.h>
#include "esp_gdbstub.h"        // GDB stub header
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"   // New I2C Master Bus API header
#include "esp_log.h"
#include "driver/gpio.h"
#include "ssd1306.h"             // SSD1306 library header (from oled_ssd1306.c)
#include "ssd1306_fonts.h"       // Font definitions

// I2C and OLED configuration
#define I2C_BUS_NUM             I2C_NUM_0
#define OLED_ADDR               0x3C
#define I2C_MASTER_SDA_IO       GPIO_NUM_8
#define I2C_MASTER_SCL_IO       GPIO_NUM_9
#define I2C_MASTER_TIMEOUT_MS   (1000 / portTICK_PERIOD_MS)

static const char *TAG = "MAIN_V2";

extern "C" void app_main(void)
{
    esp_gdbstub_init();
    ssd1306_128x64_i2c_initEx(I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, OLED_ADDR);
    ssd1306_setFixedFont(ssd1306xled_font6x8);

    ssd1306_clearScreen();
    printf("Starting firs tstatic text sent to display\n");
    ssd1306_printFixed(0, 0, "Hello, OLED V2!", STYLE_NORMAL);

    // Simple animation: move text down the screen.
    for (int i = 0; i < 10; i++) {
        ssd1306_clearScreen();
        char buf[32];
        sprintf(buf, "Moving Text %d", i);
        ssd1306_printFixed(0, i * 8, buf, STYLE_NORMAL);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }\

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}