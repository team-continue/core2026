#pragma once

#include <Arduino.h>
#include <WS2812Serial.h>

// テープ 1 本の出力。区間 (Led) が setPixel で描き、全区間を描いたあとで show() で送る
class LedStrip {
 public:
  LedStrip(uint8_t serial_pin, uint16_t led_num, uint8_t brightness)
      : serial_pin_(serial_pin), led_num_(led_num), brightness_(brightness) {}

  void init() {
    if (pixels_ != nullptr || led_num_ == 0) {
      return;
    }
    drawing_memory_ = new uint8_t[led_num_ * 3];
    display_memory_ = new uint8_t[led_num_ * 12];
    pixels_ = new WS2812Serial(led_num_, display_memory_, drawing_memory_, serial_pin_, WS2812_GRB);
    pixels_->begin();
    pixels_->setBrightness(brightness_);
    pixels_->clear();
    pixels_->show();
  }

  bool ready() const { return pixels_ != nullptr; }
  uint16_t size() const { return led_num_; }

  void setBrightness(uint8_t brightness) { brightness_ = brightness; }

  // 範囲外の番号は無視する
  void setPixel(uint16_t index, uint32_t rgb) {
    if (pixels_ == nullptr || index >= led_num_) {
      return;
    }
    pixels_->setPixel(index, rgb);
  }

  void show() {
    if (pixels_ == nullptr) {
      return;
    }
    pixels_->setBrightness(brightness_);
    pixels_->show();
  }

 private:
  uint8_t serial_pin_;
  uint16_t led_num_;
  uint8_t brightness_;
  uint8_t *drawing_memory_ = nullptr;
  uint8_t *display_memory_ = nullptr;
  WS2812Serial *pixels_ = nullptr;
};

// テープの一部 (first から count 個) に 1 つのパターンを描く区間。
// パターンの長さ・中心はこの区間の中で決まる。
// reverse = true なら、一方向に流れるパターン (kChase / kRainbowChase) を区間の末尾から先頭へ流す
class Led {
 public:
  Led(LedStrip &strip, uint16_t first, uint16_t count, unsigned long update_ms,
      float chase_speed_leds_per_sec, float multi_chase_speed_leds_per_sec,
      float blink_period_sec, float double_flash_period_sec,
      uint8_t center_out_multi_base_brightness, bool reverse = false)
      : strip_(strip),
        first_(first),
        led_num_(count),
        update_ms_(update_ms),
        chase_speed_leds_per_sec_(chase_speed_leds_per_sec),
        multi_chase_speed_leds_per_sec_(multi_chase_speed_leds_per_sec),
        blink_period_sec_(blink_period_sec),
        double_flash_period_sec_(double_flash_period_sec),
        center_out_multi_base_brightness_(center_out_multi_base_brightness),
        reverse_(reverse) {}

  // 指令の下位 8 bit。2 は一方向の流れ (以前の両端から中央への流れは 15 に移した)。
  // 砲塔の区間には 4 bit で送るので、0〜15 に収める
  enum Pattern : uint8_t {
    kOff = 0,  // ROS からの指令では「指令なし」の意味でも使う
    kGaming = 1,
    kChase = 2,  // 一方向に流れる。向きは区間の reverse で決まる
    kMultiChase = 3,
    kRainbowChase = 4,
    kRainbowMultiChase = 5,
    kCenterOutChase = 6,
    kCenterOutMultiChase = 7,
    kBlink = 8,
    kDoubleFlash = 9,
    kFade = 10,
    kSingleFlash = 11,
    kTripleFlash = 12,
    kBlinkMedium = 13,
    kBlinkSlow = 14,
    kInwardChase = 15,  // 両端から中央へ流れる
  };

  // 指令の上位 8 bit。レインボー系のパターンでは無視する。
  enum Color : uint8_t {
    kWhite = 0,
    kRed = 1,
    kYellow = 2,
    kGreen = 3,
    kBlue = 4,
  };

  void write(uint8_t pattern, uint8_t color) {
    if (pattern_ == pattern && color_ == color) {
      return;
    }
    pattern_ = pattern;
    color_ = color;
    resetAnimationState();
  }

  void setChaseSpeed(float leds_per_sec) {
    chase_speed_leds_per_sec_ = leds_per_sec < 0.0f ? 0.0f : leds_per_sec;
  }

  void setMultiChaseSpeed(float leds_per_sec) {
    multi_chase_speed_leds_per_sec_ = leds_per_sec < 0.0f ? 0.0f : leds_per_sec;
  }

  void setBlinkPeriod(float period_sec) {
    blink_period_sec_ = period_sec <= 0.0f ? 0.5f : period_sec;
  }

  void setDoubleFlashPeriod(float period_sec) {
    double_flash_period_sec_ = period_sec <= 0.0f ? 0.9f : period_sec;
  }

  void setCenterOutMultiBaseBrightness(uint8_t brightness) {
    center_out_multi_base_brightness_ = brightness;
  }

  // 描いたら true。呼び出し側は、どれかの区間が true を返したら LedStrip::show() を呼ぶ
  bool update(unsigned long now_ms) {
    if ((now_ms - last_update_ms_) < update_ms_) {
      return false;
    }
    float dt_sec = 0.0f;
    if (last_update_ms_ != 0) {
      dt_sec = static_cast<float>(now_ms - last_update_ms_) / 1000.0f;
    }
    last_update_ms_ = now_ms;
    if (!strip_.ready()) {
      return false;
    }
    render(dt_sec);
    return true;
  }

 private:
  void render(float dt_sec) {

    const uint32_t base_color = paletteColor(color_);
    switch (pattern_) {
      case kOff:
        renderOff();
        return;
      case kGaming:
        renderGaming();
        return;
      case kChase:
        renderChase(dt_sec, base_color);
        return;
      case kInwardChase:
        renderInwardChase(dt_sec, base_color);
        return;
      case kMultiChase:
        renderMultiChase(dt_sec, base_color);
        return;
      case kRainbowChase:
        renderChase(dt_sec, base_color, true);
        return;
      case kRainbowMultiChase:
        renderMultiChase(dt_sec, base_color, true);
        return;
      case kCenterOutChase:
        renderCenterOutChase(dt_sec, base_color);
        return;
      case kCenterOutMultiChase:
        renderCenterOutMultiChase(dt_sec, base_color);
        return;
      case kBlink:
        renderBlink(dt_sec, blink_period_sec_, base_color);
        return;
      case kDoubleFlash:
        renderFlash(dt_sec, 2U, base_color);
        return;
      case kFade:
        renderFade(dt_sec, base_color);
        return;
      case kSingleFlash:
        renderFlash(dt_sec, 1U, base_color);
        return;
      case kTripleFlash:
        renderFlash(dt_sec, 3U, base_color);
        return;
      case kBlinkMedium:
        renderBlink(dt_sec, kBlinkMediumPeriodSec, base_color);
        return;
      case kBlinkSlow:
        renderBlink(dt_sec, kBlinkSlowPeriodSec, base_color);
        return;
      default:
        renderOff();
        return;
    }
  }

  void resetAnimationState() {
    rainbow_offset_ = 0;
    chase_head_position_ = -static_cast<float>(kChaseBandLength);
    chase_waiting_ = false;
    chase_wait_started_ms_ = 0;
    blink_phase_ = 0.0f;
    flash_elapsed_sec_ = 0.0f;
    fade_phase_ = 0.0f;
  }

  void renderOff() {
    fill(0U);
  }

  void renderGaming() {
    for (uint16_t i = 0; i < led_num_; ++i) {
      const uint16_t scaled_index = led_num_ > 0 ? (i * 256U / led_num_) : 0U;
      const uint8_t wheel_pos =
          static_cast<uint8_t>((scaled_index + rainbow_offset_) & 0xFFU);
      setPixel(i, wheel(wheel_pos));
    }
    rainbow_offset_ = static_cast<uint8_t>(rainbow_offset_ + 1U);
  }

  void renderChase(float dt_sec, uint32_t base_color, bool rainbow = false) {
    if (led_num_ == 0) {
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
      // reverse なら末尾を流れの始まりとして数える
      const uint16_t position = reverse_ ? static_cast<uint16_t>(led_num_ - 1U - i) : i;
      const float distance_from_head = chase_head_position_ - static_cast<float>(position);
      if (distance_from_head >= 0.0f &&
          distance_from_head < static_cast<float>(kChaseBandLength)) {
        const float blend =
            1.0f - (distance_from_head / static_cast<float>(kChaseBandLength));
        intensity = static_cast<uint8_t>(
            kChaseBaseBrightness +
            (kChasePeakBrightness - kChaseBaseBrightness) * blend);
      }
      const uint32_t pixel_base = rainbow ? rainbowColorAt(i) : base_color;
      setPixel(i, scaleColor(pixel_base, intensity));
    }

    if (rainbow) {
      rainbow_offset_ = static_cast<uint8_t>(rainbow_offset_ + 1U);
    }
  }

  void renderInwardChase(float dt_sec, uint32_t base_color) {
    if (led_num_ == 0) {
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
      const float distance_to_nearest_edge =
          min(static_cast<float>(i), static_cast<float>(led_num_ - 1 - i));
      const float distance_from_head = chase_head_position_ - distance_to_nearest_edge;
      if (distance_from_head >= 0.0f &&
          distance_from_head < static_cast<float>(kChaseBandLength)) {
        const float blend =
            1.0f - (distance_from_head / static_cast<float>(kChaseBandLength));
        intensity = static_cast<uint8_t>(
            kChaseBaseBrightness +
            (kChasePeakBrightness - kChaseBaseBrightness) * blend);
      }
      setPixel(i, scaleColor(base_color, intensity));
    }

  }

  void renderCenterOutChase(float dt_sec, uint32_t base_color) {
    if (led_num_ == 0) {
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
      setPixel(i, scaleColor(base_color, intensity));
    }

  }

  void renderCenterOutMultiChase(float dt_sec, uint32_t base_color) {
    if (led_num_ == 0) {
      return;
    }

    const float center = static_cast<float>(led_num_ - 1) * 0.5f;
    const float radial_span = center + 1.0f;
    const float spacing = radial_span / kMultiChaseBandCount;
    if (dt_sec > 0.0f && multi_chase_speed_leds_per_sec_ > 0.0f) {
      chase_head_position_ += multi_chase_speed_leds_per_sec_ * dt_sec;
      while (chase_head_position_ >= spacing) {
        chase_head_position_ -= spacing;
      }
    }

    for (uint16_t i = 0; i < led_num_; ++i) {
      uint8_t intensity = kChaseBaseBrightness;
      const float distance_from_center = fabsf(static_cast<float>(i) - center);
      for (uint8_t band = 0; band < kMultiChaseBandCount; ++band) {
        const float head_distance = chase_head_position_ + spacing * band;
        float distance_from_head = head_distance - distance_from_center;
        while (distance_from_head < 0.0f) {
          distance_from_head += radial_span;
        }
        while (distance_from_head >= radial_span) {
          distance_from_head -= radial_span;
        }
        if (distance_from_head < static_cast<float>(kChaseBandLength)) {
          const float blend =
              1.0f - (distance_from_head / static_cast<float>(kChaseBandLength));
          const uint8_t band_intensity = static_cast<uint8_t>(
              center_out_multi_base_brightness_ +
              (kChasePeakBrightness - center_out_multi_base_brightness_) * blend);
          if (band_intensity > intensity) {
            intensity = band_intensity;
          }
        }
      }
      setPixel(i, scaleColor(base_color, intensity));
    }

  }

  void renderMultiChase(float dt_sec, uint32_t base_color, bool rainbow = false) {
    if (led_num_ == 0) {
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
          }
        }
      }
      const uint32_t pixel_base = rainbow ? rainbowColorAt(i) : base_color;
      setPixel(i, scaleColor(pixel_base, intensity));
    }

    if (rainbow) {
      rainbow_offset_ = static_cast<uint8_t>(rainbow_offset_ + 1U);
    }
  }

  void renderBlink(float dt_sec, float period_sec, uint32_t base_color) {
    if (dt_sec > 0.0f && period_sec > 0.0f) {
      blink_phase_ += dt_sec / period_sec;
      while (blink_phase_ >= 1.0f) {
        blink_phase_ -= 1.0f;
      }
    }

    const bool is_on = blink_phase_ < 0.5f;
    const uint32_t blink_color = scaleColor(base_color, is_on ? kChasePeakBrightness : 0U);
    fill(blink_color);
  }

  // flash_count 回光ってから休む。1 回の点灯と消灯はそれぞれ周期の 12%、
  // 最後の点灯の後の休みは周期の 64% (いずれも double_flash_period_sec_ 基準)。
  // flash_count = 2 のとき従来の kDoubleFlash と同じ見た目になる。
  void renderFlash(float dt_sec, uint8_t flash_count, uint32_t base_color) {
    const float unit_sec = double_flash_period_sec_ * kFlashUnitRatio;
    const float cycle_sec =
        double_flash_period_sec_ * (kFlashUnitRatio * 2.0f * flash_count + kFlashRestRatio -
                                    kFlashUnitRatio);
    if (dt_sec > 0.0f && cycle_sec > 0.0f) {
      flash_elapsed_sec_ += dt_sec;
      while (flash_elapsed_sec_ >= cycle_sec) {
        flash_elapsed_sec_ -= cycle_sec;
      }
    }

    bool is_on = false;
    for (uint8_t k = 0; k < flash_count; ++k) {
      const float on_start_sec = unit_sec * 2.0f * k;
      if (flash_elapsed_sec_ >= on_start_sec && flash_elapsed_sec_ < on_start_sec + unit_sec) {
        is_on = true;
        break;
      }
    }
    fill(scaleColor(base_color, is_on ? kChasePeakBrightness : 0U));
  }

  void renderFade(float dt_sec, uint32_t base_color) {
    if (dt_sec > 0.0f) {
      fade_phase_ += dt_sec / kFadePeriodSec;
      while (fade_phase_ >= 1.0f) {
        fade_phase_ -= 1.0f;
      }
    }

    const float triangle =
        fade_phase_ < 0.5f ? fade_phase_ * 2.0f : (1.0f - fade_phase_) * 2.0f;
    const uint8_t intensity = static_cast<uint8_t>(kChasePeakBrightness * triangle);
    fill(scaleColor(base_color, intensity));
  }

  void fill(uint32_t pixel_color) {
    for (uint16_t i = 0; i < led_num_; ++i) {
      setPixel(i, pixel_color);
    }
  }

  // 区間の中の番号で描く
  void setPixel(uint16_t index, uint32_t pixel_color) {
    strip_.setPixel(static_cast<uint16_t>(first_ + index), pixel_color);
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

  // 範囲外の色番号は白として扱う。
  static uint32_t paletteColor(uint8_t color_index) {
    switch (color_index) {
      case kRed:
        return color(255U, 0U, 0U);
      case kYellow:
        return color(255U, 180U, 0U);
      case kGreen:
        return color(0U, 255U, 0U);
      case kBlue:
        return color(0U, 0U, 255U);
      case kWhite:
      default:
        return color(255U, 255U, 255U);
    }
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

  LedStrip &strip_;
  uint16_t first_;
  uint16_t led_num_;  // 区間の長さ
  unsigned long update_ms_;
  unsigned long last_update_ms_ = 0;
  uint8_t pattern_ = kOff;
  uint8_t color_ = kWhite;
  uint8_t rainbow_offset_ = 0;
  float chase_head_position_ = 0.0f;
  float blink_phase_ = 0.0f;
  float flash_elapsed_sec_ = 0.0f;
  float fade_phase_ = 0.0f;
  float chase_speed_leds_per_sec_ = 0.0f;
  float multi_chase_speed_leds_per_sec_ = 0.0f;
  float blink_period_sec_ = 0.5f;
  float double_flash_period_sec_ = 0.9f;
  uint8_t center_out_multi_base_brightness_ = 18U;
  bool reverse_ = false;
  bool chase_waiting_ = false;
  unsigned long chase_wait_started_ms_ = 0;
  static constexpr float kFadePeriodSec = 2.5f;
  static constexpr float kBlinkMediumPeriodSec = 0.5f;
  static constexpr float kBlinkSlowPeriodSec = 1.0f;
  static constexpr float kFlashUnitRatio = 0.12f;
  static constexpr float kFlashRestRatio = 0.64f;
  static constexpr uint8_t kChaseBaseBrightness = 18U;
  static constexpr uint8_t kChasePeakBrightness = 255U;
  static constexpr uint8_t kChaseBandLength = 10U;
  static constexpr unsigned long kChaseLoopWaitMs = 250UL;
  static constexpr uint8_t kMultiChaseBandCount = 5U;
};
