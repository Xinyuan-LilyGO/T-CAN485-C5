/*
 * @Description: ADC voltage read example
 * @Author: LILYGO_L
 * @Date: 2026-06-12 12:00:00
 * @LastEditTime: 2026-06-12 12:00:00
 * @License: GPL 3.0
 */

#include <array>
#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lilygo_device_driver_library.h"

namespace {

namespace t_can485_c5 = lilygo_device_driver::t_can485_c5;

constexpr adc_atten_t kAdcAtten = ADC_ATTEN_DB_12;
constexpr adc_bitwidth_t kAdcBitwidth = ADC_BITWIDTH_12;
constexpr int kReadPeriodMs = 500;
constexpr int kAdcMaxRaw = 4095;
constexpr int kAdcApproxMaxVoltageMv = 3300;

struct AdcInput {
  const char* name;
  int gpio;
  adc_unit_t unit = ADC_UNIT_1;
  adc_channel_t channel = ADC_CHANNEL_0;
  adc_cali_handle_t calibration = nullptr;
};

void InitLed()
{
  const uint64_t pin_mask =
      (1ULL << t_can485_c5::gpio::led::kRed) |
      (1ULL << t_can485_c5::gpio::led::kGreen) |
      (1ULL << t_can485_c5::gpio::led::kBlue);

  gpio_config_t config = {};
  config.pin_bit_mask = pin_mask;
  config.mode = GPIO_MODE_OUTPUT;
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&config));

  ESP_ERROR_CHECK(gpio_set_level(
      static_cast<gpio_num_t>(t_can485_c5::gpio::led::kRed), 0));
  ESP_ERROR_CHECK(gpio_set_level(
      static_cast<gpio_num_t>(t_can485_c5::gpio::led::kGreen), 0));
  ESP_ERROR_CHECK(gpio_set_level(
      static_cast<gpio_num_t>(t_can485_c5::gpio::led::kBlue), 0));
}

bool InitCalibration(const AdcInput& input, adc_cali_handle_t* calibration)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t calibration_config = {};
  calibration_config.unit_id = input.unit;
  calibration_config.chan = input.channel;
  calibration_config.atten = kAdcAtten;
  calibration_config.bitwidth = kAdcBitwidth;
  return adc_cali_create_scheme_curve_fitting(&calibration_config,
             calibration) == ESP_OK;
#else
  (void)input;
  (void)calibration;
  return false;
#endif
}

bool InitAdcInput(adc_oneshot_unit_handle_t adc_handle, AdcInput* input)
{
  esp_err_t err =
      adc_oneshot_io_to_channel(input->gpio, &input->unit, &input->channel);
  if (err != ESP_OK) {
    printf("%s GPIO%d is not ADC capable: %s\n", input->name, input->gpio,
           esp_err_to_name(err));
    return false;
  }
  if (input->unit != ADC_UNIT_1) {
    printf("%s GPIO%d is on ADC unit %d, this example expects ADC1\n",
           input->name, input->gpio, static_cast<int>(input->unit));
    return false;
  }

  adc_oneshot_chan_cfg_t channel_config = {};
  channel_config.atten = kAdcAtten;
  channel_config.bitwidth = kAdcBitwidth;
  err = adc_oneshot_config_channel(adc_handle, input->channel, &channel_config);
  if (err != ESP_OK) {
    printf("Config %s failed: %s\n", input->name, esp_err_to_name(err));
    return false;
  }

  if (!InitCalibration(*input, &input->calibration)) {
    printf("%s calibration unavailable, raw value will still be printed\n",
           input->name);
  }
  return true;
}

int ReadVoltage(adc_oneshot_unit_handle_t adc_handle, const AdcInput& input)
{
  if (input.calibration != nullptr) {
    int voltage_mv = 0;
    if (adc_oneshot_get_calibrated_result(adc_handle, input.calibration,
            input.channel, &voltage_mv) == ESP_OK) {
      return voltage_mv;
    }
  }

  int raw = 0;
  if (adc_oneshot_read(adc_handle, input.channel, &raw) != ESP_OK) {
    return -1;
  }
  return raw * kAdcApproxMaxVoltageMv / kAdcMaxRaw;
}

}  // namespace

extern "C" void app_main(void)
{
  printf("Ciallo\n");
  InitLed();

  adc_oneshot_unit_init_cfg_t unit_config = {};
  unit_config.unit_id = ADC_UNIT_1;
  adc_oneshot_unit_handle_t adc_handle = nullptr;
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

  std::array<AdcInput, 3> inputs = {
      AdcInput{"ADC_1", t_can485_c5::gpio::adc::kChannel1},
      AdcInput{"ADC_2", t_can485_c5::gpio::adc::kChannel2},
      AdcInput{"ADC_3", t_can485_c5::gpio::adc::kChannel3},
  };

  for (auto& input : inputs) {
    ESP_ERROR_CHECK(InitAdcInput(adc_handle, &input) ? ESP_OK : ESP_FAIL);
  }

  while (true) {
    for (const auto& input : inputs) {
      const int voltage_mv = ReadVoltage(adc_handle, input);
      if (voltage_mv >= 0) {
        printf("%s GPIO%d: %d mV\n", input.name, input.gpio, voltage_mv);
      } else {
        printf("%s GPIO%d: read failed\n", input.name, input.gpio);
      }
    }
    printf("\n");
    vTaskDelay(pdMS_TO_TICKS(kReadPeriodMs));
  }
}
