#pragma once

#include <Arduino.h>
#include <WS2812Serial.h>

class Led {
 public:
  Led(uint8_t serial_pin, uint16_t led_num, unsigned long update_ms)
      : serial_pin_(serial_pin), led_num_(led_num), update_ms_(update_ms) {}

  void init() {
    if (pixels_ != nullptr) {
      return;
    }
    drawing_memory_ = new uint8_t[led_num_ * 3];
    display_memory_ = new uint8_t[led_num_ * 12];
    pixels_ = new WS2812Serial(led_num_, display_memory_, drawing_memory_, serial_pin_, WS2812_GRB);
    pixels_->begin();
    pixels_->clear();
    pixels_->show();
  }

  void write(uint8_t value) {
    mode_ = value;
  }

  void update() {
    const unsigned long now_ms = millis();
    if ((now_ms - last_update_ms_) < update_ms_) {
      return;
    }
    last_update_ms_ = now_ms;
    if (pixels_ == nullptr) {
      return;
    }

    if (mode_ == 0) {
      pixels_->clear();
      pixels_->show();
      return;
    }

    switch (mode_) {
      case 1:
        pixels_->setBrightness(32);
        for (uint16_t i = 0; i < led_num_; ++i) {
          const uint8_t wheel_pos = static_cast<uint8_t>((i * 256U / led_num_ + rainbow_offset_) & 0xFFU);
          pixels_->setPixel(i, wheel(wheel_pos));
        }
        pixels_->show();
        rainbow_offset_ = static_cast<uint8_t>(rainbow_offset_ + 1U);
        return;
      default:
        pixels_->clear();
        pixels_->show();
        return;
    }
  }

 private:
  uint32_t wheel(uint8_t wheel_pos) const {
    wheel_pos = 255U - wheel_pos;
    if (wheel_pos < 85U) {
      return color(255U - wheel_pos * 3U, 0U, wheel_pos * 3U);
    }
    if (wheel_pos < 170U) {
      wheel_pos = static_cast<uint8_t>(wheel_pos - 85U);
      return color(0U, wheel_pos * 3U, 255U - wheel_pos * 3U);
    }
    wheel_pos = static_cast<uint8_t>(wheel_pos - 170U);
    return color(wheel_pos * 3U, 255U - wheel_pos * 3U, 0U);
  }

  static uint32_t color(uint8_t red, uint8_t green, uint8_t blue) {
    return (static_cast<uint32_t>(red) << 16) |
           (static_cast<uint32_t>(green) << 8) |
           static_cast<uint32_t>(blue);
  }

  uint8_t serial_pin_;
  uint16_t led_num_;
  unsigned long update_ms_;
  uint8_t *drawing_memory_ = nullptr;
  uint8_t *display_memory_ = nullptr;
  WS2812Serial *pixels_ = nullptr;
  unsigned long last_update_ms_ = 0;
  uint8_t mode_ = 0;
  uint8_t rainbow_offset_ = 0;
};
