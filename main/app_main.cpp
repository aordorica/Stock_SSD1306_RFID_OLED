#include <stdio.h>

#include <iostream>
#include <iterator>
#include <memory>

// #include "abseil-cpp"
#include "animations.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_gdbstub.h"  // GDB stub header
#include "gif_frames_all.h"

using namespace std;
namespace anim = animations;

#define TAG "FRAMES"

// I2C and OLED configuration
#define I2C_BUS_NUM I2C_NUM_0
#define OLED_ADDR 0x3C
#define I2C_MASTER_SDA_IO GPIO_NUM_8
#define I2C_MASTER_SCL_IO GPIO_NUM_9
#define I2C_MASTER_TIMEOUT_MS (1000 / portTICK_PERIOD_MS)
#define SPRITE_POS {0, 0}
#define SPRITE_SIZE {23, 32}

void playGif() {
  auto it = frame_list.begin();
  auto next_it = std::next(it);

  anim::Config disp_config = {.sda = I2C_MASTER_SDA_IO,
                              .scl = I2C_MASTER_SCL_IO,
                              .bus_num = I2C_BUS_NUM,
                              .i2c_addr = OLED_ADDR,
                              .pos = SPRITE_POS,
                              .size = SPRITE_SIZE,
                              .bitmap = *it};

  anim::init(disp_config);
  while (true) {
    anim::bounce(3, disp_config.pos);
    if (it == frame_list.end()) {
      // Reset Frame
      it = frame_list.begin();
    };
    anim::updateFrames(*it);
    ++it;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

extern "C" void app_main(void) { playGif(); }