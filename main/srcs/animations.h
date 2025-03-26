#ifndef ANIMATIONS_H
#define ANIMATIONS_H

#include <stdio.h>

#include <iostream>
#include <iterator>
#include <memory>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcdgfx.h"
#include "nano_engine_v2.h"
#include "nano_gfx_types.h"

using MonoI2CDisplay = DisplaySSD1306_128x64_I2C;
using MonoEngine1 = NanoEngine1<MonoI2CDisplay>;
using MonoEngineSprite = NanoSprite<NanoEngine1<MonoI2CDisplay>::TilerT>;

namespace animations {
extern std::unique_ptr<MonoI2CDisplay> display;
extern std::unique_ptr<MonoEngine1> engine;
extern std::unique_ptr<MonoEngineSprite> sprite;

struct FloatVelocity {
  float x;
  float y;
};

// Configurtion Structure
struct Config {
  gpio_num_t sda;
  gpio_num_t scl;
  i2c_port_t bus_num;
  uint8_t i2c_addr;
  FloatVelocity pos;
  FloatVelocity size;
  const uint8_t* bitmap;
};

struct BounceState {
  FloatVelocity currentPos;
  FloatVelocity targetPos;
  FloatVelocity velocity;  // Use integer arithmetic or floats, as needed
  bool firstRun;
};

// Initialize display hardware configs
void init(const Config& config);

void bounce(int speed, FloatVelocity pos);
void updateBounce(BounceState& bounceState, FloatVelocity displaySize,
                  FloatVelocity spriteSize, int speed);

FloatVelocity computeVelocity(const FloatVelocity& current,
                              const FloatVelocity& target, float speed);

FloatVelocity pickRandomEdgeTarget(int displayWidth, int displayHeight,
                                   int spriteWidth, int spriteHeight);

void updateFrames(const uint8_t* frames);

bool drawAll();

}  // namespace animations

#endif  // ANIMATIONS_H