/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2026-06-12 11:58:10
 * @LastEditTime: 2026-06-12 13:39:32
 * @License: GPL 3.0
 */
#include <array>
#include <cstdint>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "lilygo_device_driver_library.h"

namespace {

namespace t_can485_c5 = lilygo_device_driver::t_can485_c5;

static const char* const kTag = "ws2812";
constexpr int kWs2812ResolutionHz = 10 * 1000 * 1000;
constexpr uint8_t kBrightness = 32;

struct Rgb {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

uint8_t ScaleColor(uint8_t value)
{
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * kBrightness) /
                              UINT8_MAX);
}

esp_err_t InitWs2812(led_strip_handle_t* led_strip)
{
  led_strip_config_t strip_config = {};
  strip_config.strip_gpio_num = t_can485_c5::gpio::ws2812::kData;
  strip_config.max_leds = t_can485_c5::device::ws2812::kLedCount;
  strip_config.led_model = LED_MODEL_WS2812;
  strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

  led_strip_rmt_config_t rmt_config = {};
  rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
  rmt_config.resolution_hz = kWs2812ResolutionHz;
  rmt_config.mem_block_symbols = 64;

  ESP_RETURN_ON_ERROR(
      led_strip_new_rmt_device(&strip_config, &rmt_config, led_strip),
      kTag, "create LED strip failed");

  ESP_LOGI(kTag, "WS2812 initialized on GPIO%d",
           t_can485_c5::gpio::ws2812::kData);
  return ESP_OK;
}

esp_err_t SetWs2812Color(led_strip_handle_t led_strip, const Rgb& color)
{
  ESP_RETURN_ON_ERROR(
      led_strip_set_pixel(led_strip, 0, ScaleColor(color.red),
                          ScaleColor(color.green), ScaleColor(color.blue)),
      kTag, "set LED pixel failed");
  return led_strip_refresh(led_strip);
}

}  // namespace

extern "C" void app_main(void)
{
  led_strip_handle_t led_strip = nullptr;
  ESP_ERROR_CHECK(InitWs2812(&led_strip));

  constexpr std::array<Rgb, 5> kColors = {
      Rgb{255, 0, 0},
      Rgb{0, 255, 0},
      Rgb{0, 0, 255},
      Rgb{255, 255, 255},
      Rgb{0, 0, 0},
  };

  while (true) {
    for (const auto& color : kColors) {
      ESP_ERROR_CHECK(SetWs2812Color(led_strip, color));
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}
