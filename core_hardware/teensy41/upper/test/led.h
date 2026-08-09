#pragma once

#include <Arduino.h>
#include <WS2812Serial.h>

class Led {
 public:
  Led(uint8_t serial_pin, uint16_t led_num, unsigned long update_ms,
      float chase_speed_leds_per_sec, float multi_chase_speed_leds_per_sec,
      float blink_trail_period_sec, float blink_trail_rise_ratio,
      uint8_t brightness)
      : serial_pin_(serial_pin),
        led_num_(led_num),
        update_ms_(update_ms),
        chase_speed_leds_per_sec_(chase_speed_leds_per_sec),
        multi_chase_speed_leds_per_sec_(multi_chase_speed_leds_per_sec),
        blink_trail_period_sec_(blink_trail_period_sec),
        blink_trail_rise_ratio_(blink_trail_rise_ratio),
        brightness_(brightness) {}

  enum Mode : uint8_t {
    kOff = 0,
    kGaming = 1,
    kChase = 2,
    kMultiChase = 3,
    kRainbowChase = 4,
    kRainbowMultiChase = 5,
    kCenterOutChase = 6,
    kBlinkTrail = 7,
    kFadeRed = 8,
    kFadeBlue = 9,
    kFadeGreen = 10,
  };

  void init() {
    if (pixels_ != nullptr || led_num_ == 0) {
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
    if (mode_ == value) {
      return;
    }
    mode_ = value;
    resetAnimationState();
  }

  void setChaseSpeed(float leds_per_sec) {
    chase_speed_leds_per_sec_ = leds_per_sec < 0.0f ? 0.0f : leds_per_sec;
  }

  void setBrightness(uint8_t brightness) {
    brightness_ = brightness;
  }

  void setMultiChaseSpeed(float leds_per_sec) {
    multi_chase_speed_leds_per_sec_ = leds_per_sec < 0.0f ? 0.0f : leds_per_sec;
  }

  void setBlinkTrailPeriod(float period_sec) {
    blink_trail_period_sec_ = period_sec <= 0.0f ? 1.0f : period_sec;
  }

  void setBlinkTrailRiseRatio(float rise_ratio) {
    if (rise_ratio <= 0.0f) {
      blink_trail_rise_ratio_ = 0.05f;
      return;
    }
    if (rise_ratio >= 1.0f) {
      blink_trail_rise_ratio_ = 0.95f;
      return;
    }
    blink_trail_rise_ratio_ = rise_ratio;
  }

  void update() {
    const unsigned long now_ms = millis();
    if ((now_ms - last_update_ms_) < update_ms_) {
      return;
    }
    float dt_sec = 0.0f;
    if (last_update_ms_ != 0) {
      dt_sec = static_cast<float>(now_ms - last_update_ms_) / 1000.0f;
    }
    last_update_ms_ = now_ms;
    if (pixels_ == nullptr) {
      return;
    }

    switch (mode_) {
      case kOff:
        renderOff();
        return;
      case kGaming:
        renderGaming();
        return;
      case kChase:
        renderChase(dt_sec);
        return;
      case kMultiChase:
        renderMultiChase(dt_sec);
        return;
      case kRainbowChase:
        renderChase(dt_sec, true);
        return;
      case kRainbowMultiChase:
        renderMultiChase(dt_sec, true);
        return;
      case kCenterOutChase:
        renderCenterOutChase(dt_sec);
        return;
      case kBlinkTrail:
        renderBlinkTrail(dt_sec);
        return;
      case kFadeRed:
        renderFade(255U, 0U, 0U, dt_sec);
        return;
      case kFadeBlue:
        renderFade(0U, 0U, 255U, dt_sec);
        return;
      case kFadeGreen:
        renderFade(0U, 255U, 0U, dt_sec);
        return;
      default:
        renderOff();
        return;
    }
  }

 private:
  void resetAnimationState() {
    rainbow_offset_ = 0;
    chase_head_position_ = -static_cast<float>(kChaseBandLength);
    chase_waiting_ = false;
    chase_wait_started_ms_ = 0;
    fade_phase_ = 0.0f;
    blink_phase_ = 0.0f;
  }

  void renderOff() {
    pixels_->setBrightness(brightness_);
    pixels_->clear();
    pixels_->show();
  }

  void renderGaming() {
    pixels_->setBrightness(brightness_);
    for (uint16_t i = 0; i < led_num_; ++i) {
      const uint16_t scaled_index = led_num_ > 0 ? (i * 256U / led_num_) : 0U;
      const uint8_t wheel_pos =
          static_cast<uint8_t>((scaled_index + rainbow_offset_) & 0xFFU);
      pixels_->setPixel(i, wheel(wheel_pos));
    }
    pixels_->show();
    rainbow_offset_ = static_cast<uint8_t>(rainbow_offset_ + 1U);
  }

  void renderChase(float dt_sec, bool rainbow = false) {
    pixels_->setBrightness(brightness_);
    if (led_num_ == 0) {
      pixels_->clear();
      pixels_->show();
      return;
    }

    const unsigned long now_ms = millis();
    if (chase_waiting_) {
      if ((now_ms - chase_wait_started_ms_) >= kChaseLoopWaitMs) {
        chase_waiting_ = false;
        chase_head_position_ = -static_cast<float>(kChaseBandLength);
      }
    } else if (dt_sec > 0.0f && chase_speed_leds_per_sec_ > 0.0f) {
      chase_head_position_ += chase_speed_leds_per_sec_ * dt_sec;
      if (chase_head_position_ > static_cast<float>(led_num_ - 1 + kChaseBandLength)) {
        chase_waiting_ = true;
        chase_wait_started_ms_ = now_ms;
      }
    }

    for (uint16_t i = 0; i < led_num_; ++i) {
      uint8_t intensity = kChaseBaseBrightness;
      uint32_t pixel_color =
          rainbow ? scaleColor(rainbowColorAt(i), intensity)
                  : color(intensity, intensity, intensity);
      const float distance_from_head = chase_head_position_ - static_cast<float>(i);
      if (distance_from_head >= 0.0f &&
          distance_from_head < static_cast<float>(kChaseBandLength)) {
        const float blend =
            1.0f - (distance_from_head / static_cast<float>(kChaseBandLength));
        intensity = static_cast<uint8_t>(
            kChaseBaseBrightness +
            (kChasePeakBrightness - kChaseBaseBrightness) * blend);
        pixel_color =
            rainbow ? scaleColor(rainbowColorAt(i), intensity)
                    : color(intensity, intensity, intensity);
      }
      pixels_->setPixel(i, pixel_color);
    }

    pixels_->show();
    if (rainbow) {
      rainbow_offset_ = static_cast<uint8_t>(rainbow_offset_ + 1U);
    }
  }

  void renderFade(uint8_t red, uint8_t green, uint8_t blue, float dt_sec) {
    pixels_->setBrightness(brightness_);
    if (dt_sec > 0.0f) {
      fade_phase_ += dt_sec / kFadePeriodSec;
      while (fade_phase_ >= 1.0f) {
        fade_phase_ -= 1.0f;
      }
    }

    const float triangle =
        fade_phase_ < 0.5f ? fade_phase_ * 2.0f : (1.0f - fade_phase_) * 2.0f;
    const uint8_t scaled_red = static_cast<uint8_t>(red * triangle);
    const uint8_t scaled_green = static_cast<uint8_t>(green * triangle);
    const uint8_t scaled_blue = static_cast<uint8_t>(blue * triangle);
    const uint32_t fade_color = color(scaled_red, scaled_green, scaled_blue);

    for (uint16_t i = 0; i < led_num_; ++i) {
      pixels_->setPixel(i, fade_color);
    }
    pixels_->show();
  }

  void renderCenterOutChase(float dt_sec) {
    pixels_->setBrightness(brightness_);
    if (led_num_ == 0) {
      pixels_->clear();
      pixels_->show();
      return;
    }

    const unsigned long now_ms = millis();
    const float center = static_cast<float>(led_num_ - 1) * 0.5f;
    const float chase_span = center + static_cast<float>(kChaseBandLength);

    if (chase_waiting_) {
      if ((now_ms - chase_wait_started_ms_) >= kChaseLoopWaitMs) {
        chase_waiting_ = false;
        chase_head_position_ = 0.0f;
      }
    } else if (dt_sec > 0.0f && chase_speed_leds_per_sec_ > 0.0f) {
      chase_head_position_ += chase_speed_leds_per_sec_ * dt_sec;
      if (chase_head_position_ > chase_span) {
        chase_waiting_ = true;
        chase_wait_started_ms_ = now_ms;
      }
    }

    for (uint16_t i = 0; i < led_num_; ++i) {
      uint8_t intensity = kChaseBaseBrightness;
      const float distance_from_center = fabsf(static_cast<float>(i) - center);
      const float distance_from_head = chase_head_position_ - distance_from_center;
      if (distance_from_head >= 0.0f &&
          distance_from_head < static_cast<float>(kChaseBandLength)) {
        const float blend =
            1.0f - (distance_from_head / static_cast<float>(kChaseBandLength));
        intensity = static_cast<uint8_t>(
            kChaseBaseBrightness +
            (kChasePeakBrightness - kChaseBaseBrightness) * blend);
      }
      pixels_->setPixel(i, color(intensity, intensity, intensity));
    }

    pixels_->show();
  }

  void renderBlinkTrail(float dt_sec) {
    pixels_->setBrightness(brightness_);
    if (dt_sec > 0.0f && blink_trail_period_sec_ > 0.0f) {
      blink_phase_ += dt_sec / blink_trail_period_sec_;
      while (blink_phase_ >= 1.0f) {
        blink_phase_ -= 1.0f;
      }
    }

    const float intensity_ratio =
        blink_phase_ < blink_trail_rise_ratio_ ? 1.0f : 0.0f;
    const uint8_t intensity =
        static_cast<uint8_t>(kChasePeakBrightness * intensity_ratio);
    const uint32_t blink_color = color(intensity, intensity, intensity);

    for (uint16_t i = 0; i < led_num_; ++i) {
      pixels_->setPixel(i, blink_color);
    }
    pixels_->show();
  }

  void renderMultiChase(float dt_sec, bool rainbow = false) {
    pixels_->setBrightness(brightness_);
    if (led_num_ == 0) {
      pixels_->clear();
      pixels_->show();
      return;
    }

    if (dt_sec > 0.0f && multi_chase_speed_leds_per_sec_ > 0.0f) {
      chase_head_position_ += multi_chase_speed_leds_per_sec_ * dt_sec;
      const float loop_length = static_cast<float>(led_num_) / kMultiChaseBandCount;
      while (chase_head_position_ >= loop_length) {
        chase_head_position_ -= loop_length;
      }
    }

    const float spacing = static_cast<float>(led_num_) / kMultiChaseBandCount;
    for (uint16_t i = 0; i < led_num_; ++i) {
      uint8_t intensity = kChaseBaseBrightness;
      uint32_t pixel_color =
          rainbow ? scaleColor(rainbowColorAt(i), intensity)
                  : color(intensity, intensity, intensity);
      for (uint8_t band = 0; band < kMultiChaseBandCount; ++band) {
        const float head_position = chase_head_position_ + spacing * band;
        float distance_from_head = head_position - static_cast<float>(i);
        while (distance_from_head < 0.0f) {
          distance_from_head += static_cast<float>(led_num_);
        }
        while (distance_from_head >= static_cast<float>(led_num_)) {
          distance_from_head -= static_cast<float>(led_num_);
        }
        if (distance_from_head < static_cast<float>(kChaseBandLength)) {
          const float blend =
              1.0f - (distance_from_head / static_cast<float>(kChaseBandLength));
          const uint8_t band_intensity = static_cast<uint8_t>(
              kChaseBaseBrightness +
              (kChasePeakBrightness - kChaseBaseBrightness) * blend);
          if (band_intensity > intensity) {
            intensity = band_intensity;
            pixel_color =
                rainbow ? scaleColor(rainbowColorAt(i), intensity)
                        : color(intensity, intensity, intensity);
          }
        }
      }
      pixels_->setPixel(i, pixel_color);
    }

    pixels_->show();
    if (rainbow) {
      rainbow_offset_ = static_cast<uint8_t>(rainbow_offset_ + 1U);
    }
  }

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

  uint32_t rainbowColorAt(uint16_t index) const {
    const uint16_t scaled_index = led_num_ > 0 ? (index * 256U / led_num_) : 0U;
    const uint8_t wheel_pos =
        static_cast<uint8_t>((scaled_index + rainbow_offset_) & 0xFFU);
    return wheel(wheel_pos);
  }

  static uint32_t scaleColor(uint32_t rgb, uint8_t intensity) {
    const uint8_t red = static_cast<uint8_t>(((rgb >> 16) & 0xFFU) * intensity / 255U);
    const uint8_t green = static_cast<uint8_t>(((rgb >> 8) & 0xFFU) * intensity / 255U);
    const uint8_t blue = static_cast<uint8_t>((rgb & 0xFFU) * intensity / 255U);
    return color(red, green, blue);
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
  float chase_head_position_ = 0.0f;
  float chase_speed_leds_per_sec_ = 0.0f;
  float multi_chase_speed_leds_per_sec_ = 0.0f;
  float blink_trail_period_sec_ = 1.0f;
  float blink_trail_rise_ratio_ = 0.8f;
  bool chase_waiting_ = false;
  unsigned long chase_wait_started_ms_ = 0;
  float fade_phase_ = 0.0f;
  float blink_phase_ = 0.0f;
  uint8_t brightness_ = 32;
  static constexpr float kFadePeriodSec = 2.5f;
  static constexpr uint8_t kChaseBaseBrightness = 18U;
  static constexpr uint8_t kChasePeakBrightness = 255U;
  static constexpr uint8_t kChaseBandLength = 10U;
  static constexpr unsigned long kChaseLoopWaitMs = 250UL;
  static constexpr uint8_t kMultiChaseBandCount = 5U;
};
