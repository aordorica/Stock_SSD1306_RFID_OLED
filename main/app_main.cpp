#include <stdio.h>
#include "esp_gdbstub.h" // GDB stub header
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h" // New I2C Master Bus API header
#include "esp_log.h"
#include "driver/gpio.h"
#include "lcdgfx.h"
// #include "ssd1306.h"       // SSD1306 library header (from oled_ssd1306.c)
// #include "ssd1306_fonts.h" // Font definitions

using namespace std;

// I2C and OLED configuration
#define I2C_BUS_NUM I2C_NUM_0
#define OLED_ADDR 0x3C
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_TIMEOUT_MS (1000 / portTICK_PERIOD_MS)

static const char *TAG = "MAIN_V2";

namespace animation
{
    extern "C" void printAnimation(DisplaySSD1306_128x64_I2C display)
    {
        // Simple animation: move text down the screen.
        // for (int i = 0; i < 10; i++)
        // {
        //     ssd1306_clearScreen();
        //     char buf[32];
        //     sprintf(buf, "Moving Text %d", i);
        //     ssd1306_printFixed(0, i * 8, buf, STYLE_NORMAL);
        //     vTaskDelay(500 / portTICK_PERIOD_MS);
        // }
        display.drawWindow(0, 40, 0, 0, "Loading...", true);
    }

    int progress = 0;

    void loop(DisplaySSD1306_128x64_I2C display)
    {
        display.drawProgressBar(progress);
        progress++;
        if (progress > 100)
        {
            progress = 0;
            lcd_delay(2000);
        }
        else
        {
            lcd_delay(50);
        }
    }
}

extern "C" void app_main(void)
{
    DisplaySSD1306_128x64_I2C display(-1, {I2C_BUS_NUM, OLED_ADDR, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO});
    // esp_gdbstub_init();

    display.begin();
    display.fill(0x00);
    display.setFixedFont(ssd1306xled_font6x8);

    printf("Now starting Animation print...");
    animation::printAnimation(display);

    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (animation::progress < 101)
    {
        animation::loop(display);
    }
}
