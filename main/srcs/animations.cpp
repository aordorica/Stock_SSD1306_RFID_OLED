#include "animations.h"

#include <memory>

#include "cmath"
#include "gif_frames_all.h"
#include "iterator"

#define TAG "FRAMES"
namespace animations {

std::unique_ptr<MonoI2CDisplay> display = nullptr;
std::unique_ptr<MonoEngine1> engine = nullptr;
std::unique_ptr<MonoEngineSprite> sprite = nullptr;

void init(const Config &config) {
  // Set display hardware GPIO and begin display driver.

  SPlatformI2cConfig i2c_config = {static_cast<int8_t>(config.bus_num),
                                   config.i2c_addr, config.scl, config.sda, 0};
  display = std::make_unique<MonoI2CDisplay>(-1, i2c_config);
  display->setFixedFont(ssd1306xled_font6x8);
  display->begin();

  // Initialize NanoEngine
  engine = std::make_unique<MonoEngine1>(*display);
  engine->begin();
  engine->setFrameRate(60);
  engine->drawCallback(animations::drawAll);
  engine->refresh();

  // Initialize new NanoSprite object.

  NanoPoint nPos = {static_cast<uint8_t>(config.pos.x),
                    static_cast<uint8_t>(config.pos.y)};

  NanoPoint nSize = {static_cast<uint8_t>(config.size.x),
                     static_cast<uint8_t>(config.size.y)};
  sprite = std::make_unique<MonoEngineSprite>(nPos, nSize, config.bitmap);
  engine->insert(*sprite);
}

void updateFrames(const uint8_t *frame) {
  sprite->setBitmap(frame);
  engine->refresh();
  drawAll();
  engine->display();
  engine->nextFrame();
}

void updateBounce(BounceState &bounceState, FloatVelocity displaySize,
                  FloatVelocity spriteSize, int speed) {
  int displayWidth = displaySize.x, displayHeight = displaySize.y;
  int spriteWidth = spriteSize.x, spriteHeight = spriteSize.y;

  if (bounceState.firstRun) {
    bounceState.targetPos = pickRandomEdgeTarget(displayWidth, displayHeight,
                                                 spriteWidth, spriteHeight);
    bounceState.velocity =
        computeVelocity(bounceState.currentPos, bounceState.targetPos, speed);
    bounceState.firstRun = false;
  }

  // Calculate the next position
  FloatVelocity nextPos;
  nextPos.x = bounceState.currentPos.x + bounceState.velocity.x;
  nextPos.y = bounceState.currentPos.y + bounceState.velocity.y;

  // For each coordinate, if nextPos would overshoot the target, clamp it
  if ((bounceState.velocity.x > 0 && nextPos.x > bounceState.targetPos.x) ||
      (bounceState.velocity.x < 0 && nextPos.x < bounceState.targetPos.x)) {
    nextPos.x = bounceState.targetPos.x;
  }
  if ((bounceState.velocity.y > 0 && nextPos.y > bounceState.targetPos.y) ||
      (bounceState.velocity.y < 0 && nextPos.y < bounceState.targetPos.y)) {
    nextPos.y = bounceState.targetPos.y;
  }
  bounceState.currentPos = nextPos;

  // Check if we've reached the target (within some threshold)
  int threshold = 2;
  if (abs(bounceState.currentPos.x - bounceState.targetPos.x) < threshold &&
      abs(bounceState.currentPos.y - bounceState.targetPos.y) < threshold) {
    bounceState.targetPos = pickRandomEdgeTarget(displayWidth, displayHeight,
                                                 spriteWidth, spriteHeight);
    bounceState.velocity =
        computeVelocity(bounceState.currentPos, bounceState.targetPos, speed);
  }

  sprite->moveTo(
      {(int)bounceState.currentPos.x, (int)bounceState.currentPos.y});
}

FloatVelocity pickRandomEdgeTarget(int displayWidth, int displayHeight,
                                   int spriteWidth, int spriteHeight) {
  // Randomly pick one of 4 edges: 0=top, 1=bottom, 2=left, 3=right.
  int edge = lcd_random(0, 3);
  FloatVelocity target;
  switch (edge) {
    case 0:  // top edge
      target.y = 0;
      target.x = lcd_random(0, displayWidth - spriteWidth);
      printf("\nBouncing to TOP edge: {%f, %f}", target.x, target.y);
      break;
    case 1:  // bottom edge
      target.y = displayHeight - spriteHeight;
      target.x = lcd_random(0, displayWidth - spriteWidth);
      printf("\nBouncing to BOTTOM edge: {%f, %f}", target.x, target.y);
      break;
    case 2:  // left edge
      target.x = 0;
      target.y = lcd_random(0, displayHeight - spriteHeight);
      printf("\nBouncing to LEFT edge: {%f, %f}", target.x, target.y);
      break;
    case 3:  // right edge
      target.x = displayWidth - spriteWidth;
      target.y = lcd_random(0, displayHeight - spriteHeight);
      printf("\nBouncing to RIGHT edge: {%f, %f}", target.x, target.y);
      break;
  }
  return target;
}

FloatVelocity computeVelocity(const FloatVelocity &current,
                              const FloatVelocity &target, float speed) {
  // Compute a simple velocity vector. If using integers, this is a very
  // simplified version.
  FloatVelocity vel;
  float dx = target.x - current.x;
  float dy = target.y - current.y;
  // Compute the "length" (avoid floating point if possible or cast to float)
  float dist =
      sqrt(pow(dx, 2) + pow(dy, 2));  // Manhattan distance for simplicity
  if (dist == 0) return {0, 0};
  // Scale the dx and dy to the desired speed.
  vel.x = (dx / dist) * speed;
  vel.y = (dy / dist) * speed;
  return vel;
}

void bounce(int speed, FloatVelocity pos) {
  NanoPoint startPos = {lcd_random(0, display->width()),
                        lcd_random(0, display->height())};
  static BounceState bounceState = {pos, {0, 0}, {0, 0}, true};
  FloatVelocity displaySize = {static_cast<float>(display->width()),
                               static_cast<float>(display->height())};
  FloatVelocity spriteSize = {static_cast<float>(sprite->width()),
                              static_cast<float>(sprite->height())};
  updateBounce(bounceState, displaySize, spriteSize, speed);
}

bool drawAll() {
  try {
    engine->getCanvas().clear();
    engine->getCanvas().setMode(0);
    engine->getCanvas().setColor(1);
  } catch (const std::exception &e) {
    std::cerr << "Exception Caught in drawAll(): " << e.what() << '\n';
  }

  return true;
}
}  // namespace animations
