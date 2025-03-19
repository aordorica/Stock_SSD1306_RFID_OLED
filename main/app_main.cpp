#include <stdio.h>
#include <iostream>
#include <iterator>

#include "esp_gdbstub.h" // GDB stub header
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h" // New I2C Master Bus API header
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "lcdgfx.h"
#include "gif_frames_all.h"
#include "nano_engine_v2.h"
#include "nano_gfx_types.h"

using namespace std;
#define TAG "FRAMES"

// I2C and OLED configuration
#define I2C_BUS_NUM I2C_NUM_0
#define OLED_ADDR 0x3C
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_TIMEOUT_MS (1000 / portTICK_PERIOD_MS)

// static const char *TAG = "MAIN_V2";
const int frameSize = 350;
const int totalFrames = 29;
int currentFrame = 0;

auto it = frame_list.begin();
auto next_it = std::next(it);

DisplaySSD1306_128x64_I2C display(-1, {I2C_BUS_NUM, OLED_ADDR, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO});
NanoEngine1<DisplaySSD1306_128x64_I2C> engine(display);
NanoSprite<NanoEngine1<DisplaySSD1306_128x64_I2C>::TilerT> sprite({0, 14}, {128, 36}, *it);

namespace animation
{
    bool drawAll()
    {
        try
        {
            engine.getCanvas().clear();
            engine.getCanvas().setMode(0);
            engine.getCanvas().setColor(1);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception Caught in drawAll(): " << e.what() << '\n';
        }

        return true;
    }

    void setup()
    {
        display.begin();
        engine.begin();
        engine.setFrameRate(30);
        engine.drawCallback(drawAll);
        engine.refresh();
        if (it == frame_list.end())
        {
            ESP_LOGW(TAG, "WARNING: Begining Frame is End()!");
            return;
        }
        engine.insert(sprite);
    }

    void loop()
    {
        try
        {
            if (!engine.nextFrame() && it == frame_list.end())
                return;
            engine.display();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception Caught in loop(): " << e.what() << '\n';
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    void nextFrame()
    {
        try
        {
            ++it;
            next_it = std::next(it);
            sprite.setBitmap(*it);
            engine.refresh();
            drawAll();
            engine.display();
            loop();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception Caught in nextFrame(): " << e.what() << '\n';
        }
    }

    void resetFrame()
    {
        try
        {
            it = frame_list.begin();
            sprite.setBitmap(*it);
            engine.refresh();
            drawAll();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception Caught: " << e.what() << '\n';
        }
    }

    extern "C" void app_main(void)
    {
        // esp_gdbstub_init();
        display.setFixedFont(ssd1306xled_font6x8);

        printf("Now starting Animation print...");

        // Setup Engine and Display
        setup();
        while (1)
        {
            if (next_it == frame_list.end())
                resetFrame();
            engine.display();
            loop();
            try
            {
                nextFrame();
            }
            catch (const std::exception &e)
            {
                std::cerr << "Exception accessing nextFrame(): " << e.what() << '\n';
            }
            ESP_LOGD(TAG, "\nValues -- Frame: %s, it: %s", *it, it.operator*());
        }
    }
}