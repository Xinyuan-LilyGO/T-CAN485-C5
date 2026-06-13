/*
 * @Description: General board test
 * @Author: LILYGO_L
 * @Date: 2026-06-12 17:30:00
 * @LastEditTime: 2026-06-12 17:30:00
 * @License: GPL 3.0
 */

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "lilygo_device_driver_library.h"
#include "nvs_flash.h"
#include "sd_protocol_defs.h"
#include "sdmmc_cmd.h"

namespace {

namespace t_can485_c5 = lilygo_device_driver::t_can485_c5;

constexpr char kWifiStaSsid[] = "LilyGo-AABB";
constexpr char kWifiStaPassword[] = "xinyuandianzi";
constexpr char kWifiApSsid[] = "T-CAN485-C5";
constexpr char kWifiApPassword[] = "12345678";
constexpr int kWifiApChannel = 6;
constexpr int kWifiApMaxConnection = 4;

constexpr char kMountPoint[] = "/sdcard";
constexpr int kSdPollPeriodMs = 2000;
constexpr int kSdSpiMaxFreqKhz = 10000;

constexpr adc_atten_t kAdcAtten = ADC_ATTEN_DB_12;
constexpr adc_bitwidth_t kAdcBitwidth = ADC_BITWIDTH_12;
constexpr int kAdcReadPeriodMs = 500;
constexpr int kAdcMaxRaw = 4095;
constexpr int kAdcApproxMaxVoltageMv = 3300;

constexpr uart_port_t kRs485UartPort = UART_NUM_1;
constexpr int kRs485BaudRate = 115200;
constexpr int kRs485RxBufferSize = 4096;
constexpr int kRs485TxBufferSize = 4096;
constexpr int kRs485PayloadSize = 240;
constexpr uint8_t kRs485Header0 = 0xAA;
constexpr uint8_t kRs485Header1 = 0x55;
constexpr size_t kRs485HeaderSize = 2;
constexpr size_t kRs485SequenceSize = 4;
constexpr size_t kRs485LengthSize = 2;
constexpr size_t kRs485CrcSize = 2;
constexpr size_t kRs485PacketOverhead = kRs485HeaderSize + kRs485SequenceSize +
                                        kRs485LengthSize + kRs485CrcSize;
constexpr size_t kRs485PacketSize = kRs485PacketOverhead + kRs485PayloadSize;
constexpr int kRs485TxDoneWaitMs = 20;
constexpr char kRs485TestChar = 'R';

constexpr int kCanDataLength = 8;
constexpr int kCanTxWaitMs = 0;
constexpr int kCanTxIntervalMs = 20;
constexpr int kCanBusOffDelayMs = 500;
constexpr int kCanRecoverRetryIntervalMs = 1000;
constexpr int kCanPollMs = 100;
constexpr int kCanMaxReceiveFramesPerLoop = 32;
constexpr uint32_t kCanTestId = 0x0F1;
constexpr char kCanTestChar = 'C';
constexpr int kCanBitrate = 1000000;
constexpr int kCanTxQueueDepth = 16;
constexpr int kCanRxQueueDepth = 32;

constexpr uint8_t kWs2812Brightness = 32;
constexpr int kWs2812PeriodMs = 1000;
constexpr int kPrintIntervalMs = 3000;
constexpr int kTaskStackSize = 6 * 1024;
constexpr UBaseType_t kTaskPriority = 5;

enum class TestMode {
  kOff,
  kSend,
  kReceive,
};

struct Rgb {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

struct AdcInput {
  const char* name;
  int gpio;
  adc_unit_t unit = ADC_UNIT_1;
  adc_channel_t channel = ADC_CHANNEL_0;
  adc_cali_handle_t calibration = nullptr;
};

struct CanRxFrame {
  uint32_t id;
  uint8_t data[kCanDataLength];
  uint8_t data_length;
  bool is_remote;
  bool is_extended;
};

volatile TestMode g_rs485_mode = TestMode::kOff;
volatile TestMode g_can_mode = TestMode::kOff;
volatile bool g_wifi_sta_connected = false;

int g_adc_mv[3] = {};
bool g_sd_mounted = false;
uint64_t g_sd_size_mb = 0;
char g_sd_type[24] = "Not mounted";
char g_sta_ip[16] = "0.0.0.0";
char g_ap_ip[16] = "192.168.4.1";

size_t g_rs485_total_size = 0;
size_t g_rs485_bytes_this_time = 0;
bool g_rs485_data_ok = true;
uint32_t g_rs485_crc_error_count = 0;
uint32_t g_rs485_sequence_error_count = 0;
uint32_t g_rs485_tx_sequence = 0;
uint32_t g_rs485_expected_sequence = 0;
bool g_rs485_has_expected_sequence = false;
std::vector<uint8_t> g_rs485_rx_stream;

size_t g_can_total_size = 0;
size_t g_can_bytes_this_time = 0;
bool g_can_data_ok = true;
uint32_t g_can_bus_error_count = 0;
char g_can_state_text[24] = "off";

bool g_sd_spi_bus_ready = false;
sdmmc_card_t* g_sd_card = nullptr;
twai_node_handle_t g_can_node = nullptr;
QueueHandle_t g_can_rx_queue = nullptr;
volatile uint32_t g_can_error_flags = 0;
volatile twai_error_state_t g_can_state = TWAI_ERROR_ACTIVE;
volatile bool g_can_recovering = false;
int64_t g_can_last_recover_us = 0;

const char* ModeName(TestMode mode)
{
  switch (mode) {
    case TestMode::kSend:
      return "send";
    case TestMode::kReceive:
      return "receive";
    default:
      return "stop";
  }
}

const char* CanStateName(twai_error_state_t state)
{
  switch (state) {
    case TWAI_ERROR_ACTIVE:
      return "active";
    case TWAI_ERROR_WARNING:
      return "warning";
    case TWAI_ERROR_PASSIVE:
      return "passive";
    case TWAI_ERROR_BUS_OFF:
      return "bus_off";
    default:
      return "unknown";
  }
}

uint8_t ScaleColor(uint8_t value)
{
  return static_cast<uint8_t>(
      (static_cast<uint16_t>(value) * kWs2812Brightness) / UINT8_MAX);
}

void InitAdcLed()
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

esp_err_t InitWs2812(led_strip_handle_t* led_strip)
{
  led_strip_config_t strip_config = {};
  strip_config.strip_gpio_num = t_can485_c5::gpio::ws2812::kData;
  strip_config.max_leds = t_can485_c5::device::ws2812::kLedCount;
  strip_config.led_model = LED_MODEL_WS2812;
  strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

  led_strip_rmt_config_t rmt_config = {};
  rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
  rmt_config.resolution_hz = t_can485_c5::device::ws2812::kResolutionHz;
  rmt_config.mem_block_symbols = 64;
  return led_strip_new_rmt_device(&strip_config, &rmt_config, led_strip);
}

void SetWs2812Color(led_strip_handle_t led_strip, const Rgb& color)
{
  ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, ScaleColor(color.red),
                                      ScaleColor(color.green),
                                      ScaleColor(color.blue)));
  ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void Ws2812Task(void* param)
{
  (void)param;
  led_strip_handle_t led_strip = nullptr;
  ESP_ERROR_CHECK(InitWs2812(&led_strip));
  constexpr std::array<Rgb, 4> kColors = {
      Rgb{255, 0, 0},
      Rgb{0, 255, 0},
      Rgb{0, 0, 255},
      Rgb{255, 255, 255},
  };

  while (true) {
    for (const auto& color : kColors) {
      SetWs2812Color(led_strip, color);
      vTaskDelay(pdMS_TO_TICKS(kWs2812PeriodMs));
    }
  }
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
  if (err != ESP_OK || input->unit != ADC_UNIT_1) {
    return false;
  }

  adc_oneshot_chan_cfg_t channel_config = {};
  channel_config.atten = kAdcAtten;
  channel_config.bitwidth = kAdcBitwidth;
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, input->channel,
                                             &channel_config));
  InitCalibration(*input, &input->calibration);
  return true;
}

int ReadVoltage(adc_oneshot_unit_handle_t adc_handle, const AdcInput& input)
{
  if (input.calibration != nullptr) {
    int voltage_mv = 0;
    if (adc_oneshot_get_calibrated_result(adc_handle, input.calibration,
                                          input.channel,
                                          &voltage_mv) == ESP_OK) {
      return voltage_mv;
    }
  }

  int raw = 0;
  if (adc_oneshot_read(adc_handle, input.channel, &raw) != ESP_OK) {
    return -1;
  }
  return raw * kAdcApproxMaxVoltageMv / kAdcMaxRaw;
}

void AdcTask(void* param)
{
  (void)param;
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
    for (size_t i = 0; i < inputs.size(); ++i) {
      g_adc_mv[i] = ReadVoltage(adc_handle, inputs[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(kAdcReadPeriodMs));
  }
}

bool InitSdSpiBus()
{
  if (g_sd_spi_bus_ready) {
    return true;
  }

  spi_bus_config_t bus_config = {};
  bus_config.mosi_io_num = t_can485_c5::gpio::sd::kMosi;
  bus_config.miso_io_num = t_can485_c5::gpio::sd::kMiso;
  bus_config.sclk_io_num = t_can485_c5::gpio::sd::kSclk;
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.max_transfer_sz = 4000;

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  esp_err_t err = spi_bus_initialize(static_cast<spi_host_device_t>(host.slot),
                                     &bus_config, SDSPI_DEFAULT_DMA);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return false;
  }
  g_sd_spi_bus_ready = true;
  return true;
}

const char* CardTypeName(const sdmmc_card_t* card)
{
  if (card == nullptr) {
    return "Unknown";
  }
  if (card->is_sdio) {
    return "SDIO";
  }
  if (card->is_mmc) {
    return "MMC";
  }
  return (card->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC";
}

bool MountSdCard()
{
  if (!InitSdSpiBus()) {
    return false;
  }

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.max_freq_khz = kSdSpiMaxFreqKhz;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.host_id = static_cast<spi_host_device_t>(host.slot);
  slot_config.gpio_cs = static_cast<gpio_num_t>(t_can485_c5::gpio::sd::kCs);

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
  mount_config.format_if_mount_failed = false;
  mount_config.max_files = 4;
  mount_config.allocation_unit_size = 16 * 1024;
  mount_config.disk_status_check_enable = true;

  sdmmc_card_t* card = nullptr;
  esp_err_t err = esp_vfs_fat_sdspi_mount(kMountPoint, &host, &slot_config,
                                          &mount_config, &card);
  if (err != ESP_OK) {
    return false;
  }

  g_sd_card = card;
  g_sd_mounted = true;
  g_sd_size_mb = static_cast<uint64_t>(card->csd.capacity) *
                 card->csd.sector_size / (1024ULL * 1024ULL);
  std::snprintf(g_sd_type, sizeof(g_sd_type), "%s", CardTypeName(card));
  printf("[sd] mounted type=%s size=%llu MB\n", g_sd_type,
         static_cast<unsigned long long>(g_sd_size_mb));
  return true;
}

bool MountedCardStillReadable()
{
  return g_sd_mounted && g_sd_card != nullptr &&
         sdmmc_get_status(g_sd_card) == ESP_OK;
}

void UnmountSdCard()
{
  if (g_sd_mounted && g_sd_card != nullptr) {
    esp_vfs_fat_sdcard_unmount(kMountPoint, g_sd_card);
  }
  g_sd_card = nullptr;
  g_sd_mounted = false;
  g_sd_size_mb = 0;
  std::snprintf(g_sd_type, sizeof(g_sd_type), "%s", "Not mounted");
  printf("[sd] removed or unavailable\n");
}

void SdTask(void* param)
{
  (void)param;
  while (true) {
    if (!g_sd_mounted) {
      MountSdCard();
    } else if (!MountedCardStillReadable()) {
      UnmountSdCard();
    }
    vTaskDelay(pdMS_TO_TICKS(kSdPollPeriodMs));
  }
}

uint16_t Crc16Modbus(const uint8_t* data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

void WriteLe16(uint8_t* data, uint16_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void WriteLe32(uint8_t* data, uint32_t value)
{
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

uint16_t ReadLe16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t ReadLe32(const uint8_t* data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

std::array<uint8_t, kRs485PacketSize> BuildRs485Packet(uint32_t sequence)
{
  std::array<uint8_t, kRs485PacketSize> packet = {};
  packet[0] = kRs485Header0;
  packet[1] = kRs485Header1;
  WriteLe32(&packet[2], sequence);
  WriteLe16(&packet[6], kRs485PayloadSize);
  for (size_t i = 0; i < kRs485PayloadSize; ++i) {
    packet[8 + i] = kRs485TestChar;
  }
  const uint16_t crc = Crc16Modbus(&packet[2],
                                   kRs485SequenceSize + kRs485LengthSize +
                                       kRs485PayloadSize);
  WriteLe16(&packet[8 + kRs485PayloadSize], crc);
  return packet;
}

size_t ProcessRs485Stream(const uint8_t* data, size_t len)
{
  g_rs485_rx_stream.insert(g_rs485_rx_stream.end(), data, data + len);
  size_t valid_payload_size = 0;

  while (g_rs485_rx_stream.size() >= kRs485PacketOverhead) {
    size_t header_pos = 0;
    while (header_pos + 1 < g_rs485_rx_stream.size() &&
           (g_rs485_rx_stream[header_pos] != kRs485Header0 ||
            g_rs485_rx_stream[header_pos + 1] != kRs485Header1)) {
      ++header_pos;
    }
    if (header_pos > 0) {
      g_rs485_rx_stream.erase(g_rs485_rx_stream.begin(),
                              g_rs485_rx_stream.begin() + header_pos);
    }
    if (g_rs485_rx_stream.size() < kRs485PacketOverhead) {
      break;
    }
    if (g_rs485_rx_stream[0] != kRs485Header0 ||
        g_rs485_rx_stream[1] != kRs485Header1) {
      break;
    }

    const uint16_t payload_len = ReadLe16(&g_rs485_rx_stream[6]);
    if (payload_len == 0 || payload_len > kRs485PayloadSize) {
      g_rs485_data_ok = false;
      g_rs485_rx_stream.erase(g_rs485_rx_stream.begin());
      continue;
    }

    const size_t packet_size = kRs485PacketOverhead + payload_len;
    if (g_rs485_rx_stream.size() < packet_size) {
      break;
    }

    const uint32_t sequence = ReadLe32(&g_rs485_rx_stream[2]);
    const uint16_t expected_crc =
        ReadLe16(&g_rs485_rx_stream[8 + payload_len]);
    const uint16_t actual_crc =
        Crc16Modbus(&g_rs485_rx_stream[2],
                    kRs485SequenceSize + kRs485LengthSize + payload_len);
    if (actual_crc != expected_crc) {
      ++g_rs485_crc_error_count;
      g_rs485_data_ok = false;
      g_rs485_rx_stream.erase(g_rs485_rx_stream.begin(),
                              g_rs485_rx_stream.begin() + packet_size);
      continue;
    }

    if (g_rs485_has_expected_sequence &&
        sequence != g_rs485_expected_sequence) {
      ++g_rs485_sequence_error_count;
      g_rs485_data_ok = false;
    }
    g_rs485_expected_sequence = sequence + 1;
    g_rs485_has_expected_sequence = true;

    valid_payload_size += payload_len;

    g_rs485_rx_stream.erase(g_rs485_rx_stream.begin(),
                            g_rs485_rx_stream.begin() + packet_size);
  }

  return valid_payload_size;
}

bool InitRs485()
{
  uart_config_t uart_config = {};
  uart_config.baud_rate = kRs485BaudRate;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  esp_err_t err = uart_driver_install(kRs485UartPort, kRs485RxBufferSize,
                                      kRs485TxBufferSize, 0, nullptr, 0);
  if (err != ESP_OK) {
    return false;
  }
  err = uart_param_config(kRs485UartPort, &uart_config);
  if (err != ESP_OK) {
    uart_driver_delete(kRs485UartPort);
    return false;
  }
  err = uart_set_pin(kRs485UartPort, t_can485_c5::gpio::rs485::kTx,
                     t_can485_c5::gpio::rs485::kRx, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    uart_driver_delete(kRs485UartPort);
    return false;
  }
  return true;
}

void Rs485Task(void* param)
{
  (void)param;
  TestMode active_mode = TestMode::kOff;
  bool initialized = false;
  uint32_t last_print_ms = 0;

  while (true) {
    const TestMode requested_mode = g_rs485_mode;
    if (requested_mode != active_mode) {
      if (initialized) {
        uart_driver_delete(kRs485UartPort);
        initialized = false;
      }
      active_mode = requested_mode;
      if (active_mode != TestMode::kOff) {
        g_rs485_total_size = 0;
        g_rs485_bytes_this_time = 0;
        g_rs485_data_ok = true;
        g_rs485_crc_error_count = 0;
        g_rs485_sequence_error_count = 0;
        g_rs485_tx_sequence = 0;
        g_rs485_expected_sequence = 0;
        g_rs485_has_expected_sequence = false;
        g_rs485_rx_stream.clear();
        initialized = InitRs485();
        printf("[rs485] mode=%s init=%s\n", ModeName(active_mode),
               initialized ? "ok" : "failed");
      }
    }

    if (!initialized || active_mode == TestMode::kOff) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (active_mode == TestMode::kSend) {
      const auto packet = BuildRs485Packet(g_rs485_tx_sequence++);
      const int len = uart_write_bytes(kRs485UartPort, packet.data(),
                                       packet.size());
      if (len > 0) {
        uart_wait_tx_done(kRs485UartPort, pdMS_TO_TICKS(kRs485TxDoneWaitMs));
        g_rs485_bytes_this_time += len;
        g_rs485_total_size += len;
      }
    } else {
      size_t rx_len = 0;
      uart_get_buffered_data_len(kRs485UartPort, &rx_len);
      if (rx_len > 0) {
        std::string buffer(rx_len, '\0');
        const int read_len =
            uart_read_bytes(kRs485UartPort, buffer.data(), rx_len,
                            pdMS_TO_TICKS(20));
        if (read_len > 0) {
          const size_t valid_size = ProcessRs485Stream(
              reinterpret_cast<const uint8_t*>(buffer.data()), read_len);
          g_rs485_bytes_this_time += valid_size;
          g_rs485_total_size += valid_size;
        }
      }
    }

    const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_print_ms >= kPrintIntervalMs) {
      if (active_mode == TestMode::kSend) {
        printf("[rs485 %s] total %zu B\n", ModeName(active_mode),
               g_rs485_total_size);
      } else {
        printf("[rs485 %s] total %zu B | crc error %lu | seq error %lu\n",
               ModeName(active_mode), g_rs485_total_size,
               static_cast<unsigned long>(g_rs485_crc_error_count),
               static_cast<unsigned long>(g_rs485_sequence_error_count));
      }
      g_rs485_bytes_this_time = 0;
      last_print_ms = now;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

bool IRAM_ATTR OnCanRxDone(twai_node_handle_t handle,
                           const twai_rx_done_event_data_t* event,
                           void* user_ctx)
{
  (void)event;
  (void)user_ctx;
  if (g_can_rx_queue == nullptr) {
    return false;
  }

  uint8_t data[TWAI_FRAME_MAX_LEN] = {};
  twai_frame_t frame = {};
  frame.buffer = data;
  frame.buffer_len = sizeof(data);
  if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) {
    return false;
  }

  CanRxFrame rx_frame = {};
  rx_frame.id = frame.header.id;
  rx_frame.is_remote = frame.header.rtr;
  rx_frame.is_extended = frame.header.ide;
  uint16_t data_length = twaifd_dlc2len(frame.header.dlc);
  if (data_length > kCanDataLength) {
    data_length = kCanDataLength;
  }
  rx_frame.data_length = static_cast<uint8_t>(data_length);
  for (uint16_t i = 0; i < data_length; ++i) {
    rx_frame.data[i] = data[i];
  }

  BaseType_t high_task_wakeup = pdFALSE;
  xQueueSendFromISR(g_can_rx_queue, &rx_frame, &high_task_wakeup);
  return high_task_wakeup == pdTRUE;
}

bool IRAM_ATTR OnCanError(twai_node_handle_t handle,
                          const twai_error_event_data_t* event,
                          void* user_ctx)
{
  (void)handle;
  (void)user_ctx;
  g_can_error_flags |= event->err_flags.val;
  return false;
}

bool IRAM_ATTR OnCanStateChange(twai_node_handle_t handle,
                                const twai_state_change_event_data_t* event,
                                void* user_ctx)
{
  (void)handle;
  (void)user_ctx;
  g_can_state = event->new_sta;
  if (event->new_sta == TWAI_ERROR_ACTIVE) {
    g_can_recovering = false;
  }
  return false;
}

bool InitCan()
{
  g_can_error_flags = 0;
  g_can_state = TWAI_ERROR_ACTIVE;
  g_can_recovering = false;
  g_can_last_recover_us = 0;
  std::snprintf(g_can_state_text, sizeof(g_can_state_text), "%s", "active");

  g_can_rx_queue = xQueueCreate(kCanRxQueueDepth, sizeof(CanRxFrame));
  if (g_can_rx_queue == nullptr) {
    return false;
  }

  twai_onchip_node_config_t node_config = {};
  node_config.io_cfg.tx = static_cast<gpio_num_t>(t_can485_c5::gpio::can::kTx);
  node_config.io_cfg.rx = static_cast<gpio_num_t>(t_can485_c5::gpio::can::kRx);
  node_config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  node_config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  node_config.bit_timing.bitrate = kCanBitrate;
  node_config.fail_retry_cnt = 0;
  node_config.tx_queue_depth = kCanTxQueueDepth;

  esp_err_t err = twai_new_node_onchip(&node_config, &g_can_node);
  if (err != ESP_OK) {
    vQueueDelete(g_can_rx_queue);
    g_can_rx_queue = nullptr;
    return false;
  }

  twai_mask_filter_config_t filter_config = {};
  filter_config.id = 0;
  filter_config.mask = 0;
  ESP_ERROR_CHECK(twai_node_config_mask_filter(g_can_node, 0,
                                               &filter_config));

  twai_event_callbacks_t callbacks = {};
  callbacks.on_rx_done = OnCanRxDone;
  callbacks.on_error = OnCanError;
  callbacks.on_state_change = OnCanStateChange;
  ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_can_node, &callbacks,
                                                     nullptr));
  ESP_ERROR_CHECK(twai_node_enable(g_can_node));
  return true;
}

void DeinitCan()
{
  if (g_can_node != nullptr) {
    twai_node_transmit_wait_all_done(g_can_node, 100);
    twai_node_disable(g_can_node);
    twai_node_delete(g_can_node);
    g_can_node = nullptr;
  }
  if (g_can_rx_queue != nullptr) {
    vQueueDelete(g_can_rx_queue);
    g_can_rx_queue = nullptr;
  }
  std::snprintf(g_can_state_text, sizeof(g_can_state_text), "%s", "off");
}

void ServiceCanState()
{
  if (g_can_node == nullptr) {
    return;
  }

  twai_node_status_t status = {};
  twai_node_record_t record = {};
  if (twai_node_get_info(g_can_node, &status, &record) != ESP_OK) {
    return;
  }
  g_can_bus_error_count = record.bus_err_num;
  std::snprintf(g_can_state_text, sizeof(g_can_state_text), "%s",
                CanStateName(status.state));

  const int64_t now = esp_timer_get_time();
  const bool can_retry_recover =
      now - g_can_last_recover_us >= kCanRecoverRetryIntervalMs * 1000;
  if (status.state == TWAI_ERROR_BUS_OFF && !g_can_recovering &&
      can_retry_recover) {
    if (twai_node_recover(g_can_node) == ESP_OK) {
      g_can_recovering = true;
      g_can_last_recover_us = now;
    }
  }
}

void CanTask(void* param)
{
  (void)param;
  TestMode active_mode = TestMode::kOff;
  bool initialized = false;
  uint32_t last_print_ms = 0;
  uint8_t tx_data[kCanDataLength] = {};
  for (int i = 0; i < kCanDataLength; ++i) {
    tx_data[i] = kCanTestChar;
  }

  while (true) {
    const TestMode requested_mode = g_can_mode;
    if (requested_mode != active_mode) {
      if (initialized) {
        DeinitCan();
        initialized = false;
      }
      active_mode = requested_mode;
      if (active_mode != TestMode::kOff) {
        g_can_total_size = 0;
        g_can_bytes_this_time = 0;
        g_can_data_ok = true;
        g_can_bus_error_count = 0;
        initialized = InitCan();
        printf("[can] mode=%s init=%s\n", ModeName(active_mode),
               initialized ? "ok" : "failed");
      }
    }

    if (!initialized || active_mode == TestMode::kOff) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    ServiceCanState();
    if (active_mode == TestMode::kSend) {
      twai_node_status_t status = {};
      twai_node_record_t record = {};
      if (twai_node_get_info(g_can_node, &status, &record) == ESP_OK &&
          status.state != TWAI_ERROR_BUS_OFF &&
          status.tx_queue_remaining > 0) {
        twai_frame_t frame = {};
        frame.header.id = kCanTestId;
        frame.header.dlc = kCanDataLength;
        frame.buffer = tx_data;
        frame.buffer_len = sizeof(tx_data);
        if (twai_node_transmit(g_can_node, &frame, kCanTxWaitMs) == ESP_OK) {
          g_can_bytes_this_time += kCanDataLength;
          g_can_total_size += kCanDataLength;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(kCanTxIntervalMs));
    } else {
      CanRxFrame rx_frame = {};
      int receive_count = 0;
      while (receive_count < kCanMaxReceiveFramesPerLoop &&
             xQueueReceive(g_can_rx_queue, &rx_frame,
                           pdMS_TO_TICKS(kCanPollMs)) == pdTRUE) {
        ++receive_count;
        if (!rx_frame.is_remote && !rx_frame.is_extended &&
            rx_frame.id == kCanTestId &&
            rx_frame.data_length == kCanDataLength) {
          g_can_bytes_this_time += rx_frame.data_length;
          g_can_total_size += rx_frame.data_length;
        } else {
          g_can_data_ok = false;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_print_ms >= kPrintIntervalMs) {
      printf("[can %s] total %zu B | bus error %lu | state %s\n",
             ModeName(active_mode), g_can_total_size,
             static_cast<unsigned long>(g_can_bus_error_count),
             g_can_state_text);
      g_can_bytes_this_time = 0;
      last_print_ms = now;
    }
  }
}

void StartSntp()
{
  setenv("TZ", "HKT-8", 1);
  tzset();
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_setservername(1, "time.google.com");
  esp_sntp_init();
}

void WifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id,
                      void* event_data)
{
  (void)arg;
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    g_wifi_sta_connected = false;
    std::snprintf(g_sta_ip, sizeof(g_sta_ip), "%s", "0.0.0.0");
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    const ip_event_got_ip_t* event =
        static_cast<const ip_event_got_ip_t*>(event_data);
    g_wifi_sta_connected = true;
    std::snprintf(g_sta_ip, sizeof(g_sta_ip), IPSTR,
                  IP2STR(&event->ip_info.ip));
  }
}

void InitWifi()
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();

  esp_netif_ip_info_t ap_ip = {};
  esp_netif_get_ip_info(ap_netif, &ap_ip);
  std::snprintf(g_ap_ip, sizeof(g_ap_ip), IPSTR, IP2STR(&ap_ip.ip));

  wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init_config));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &WifiEventHandler, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &WifiEventHandler, nullptr));

  wifi_config_t sta_config = {};
  std::snprintf(reinterpret_cast<char*>(sta_config.sta.ssid),
                sizeof(sta_config.sta.ssid), "%s", kWifiStaSsid);
  std::snprintf(reinterpret_cast<char*>(sta_config.sta.password),
                sizeof(sta_config.sta.password), "%s", kWifiStaPassword);
  sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  wifi_config_t ap_config = {};
  std::snprintf(reinterpret_cast<char*>(ap_config.ap.ssid),
                sizeof(ap_config.ap.ssid), "%s", kWifiApSsid);
  ap_config.ap.ssid_len = std::strlen(kWifiApSsid);
  std::snprintf(reinterpret_cast<char*>(ap_config.ap.password),
                sizeof(ap_config.ap.password), "%s", kWifiApPassword);
  ap_config.ap.channel = kWifiApChannel;
  ap_config.ap.max_connection = kWifiApMaxConnection;
  ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  StartSntp();
}

std::string CurrentTimeString()
{
  std::time_t now = 0;
  std::time(&now);
  struct tm time_info = {};
  localtime_r(&now, &time_info);
  if (time_info.tm_year < (2024 - 1900)) {
    return "syncing";
  }

  char buffer[32] = {};
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &time_info);
  return buffer;
}

std::string StatusJson()
{
  char buffer[1536] = {};
  const TestMode rs485_mode = g_rs485_mode;
  const TestMode can_mode = g_can_mode;
  const std::string time_text = CurrentTimeString();

  std::snprintf(
      buffer, sizeof(buffer),
      "{"
      "\"time\":\"%s\","
      "\"wifi\":{\"sta\":%s,\"sta_ip\":\"%s\",\"ap_ssid\":\"%s\","
      "\"ap_ip\":\"%s\"},"
      "\"sd\":{\"mounted\":%s,\"type\":\"%s\",\"size_mb\":%llu},"
      "\"adc\":[%d,%d,%d],"
      "\"rs485\":{\"mode\":\"%s\",\"total\":%u,\"ok\":%s,"
      "\"crc_errors\":%lu,\"sequence_errors\":%lu},"
      "\"can\":{\"mode\":\"%s\",\"total\":%u,\"ok\":%s,\"state\":\"%s\","
      "\"bus_errors\":%lu}"
      "}",
      time_text.c_str(), g_wifi_sta_connected ? "true" : "false", g_sta_ip,
      kWifiApSsid, g_ap_ip, g_sd_mounted ? "true" : "false", g_sd_type,
      static_cast<unsigned long long>(g_sd_size_mb), g_adc_mv[0],
      g_adc_mv[1], g_adc_mv[2], ModeName(rs485_mode),
      static_cast<unsigned>(g_rs485_total_size),
      g_rs485_data_ok ? "true" : "false",
      static_cast<unsigned long>(g_rs485_crc_error_count),
      static_cast<unsigned long>(g_rs485_sequence_error_count),
      ModeName(can_mode),
      static_cast<unsigned>(g_can_total_size), g_can_data_ok ? "true" : "false",
      g_can_state_text, static_cast<unsigned long>(g_can_bus_error_count));
  return buffer;
}

constexpr char kIndexHtml[] = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>T-CAN485-C5 Control Center</title>
<style>
body{margin:0;font-family:Arial,sans-serif;background:#101418;color:#e8eef2}
header{padding:22px 24px;background:#17212b;border-bottom:1px solid #2a3a48}
h1{margin:0;font-size:24px}main{padding:18px;display:grid;gap:14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:14px}
.card{background:#18222c;border:1px solid #2b3a47;border-radius:8px;padding:16px}
.label{color:#91a4b4;font-size:13px}.value{font-size:26px;margin-top:6px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
button{background:#263746;color:#e8eef2;border:1px solid #41566a;border-radius:6px;padding:10px 14px}
button.active{background:#1f8f5f;border-color:#32b878}
.ok{color:#63d68a}.bad{color:#ff7c7c}.small{font-size:14px;color:#a9bac7}
</style>
</head>
<body>
<header><h1>T-CAN485-C5 Control Center</h1><div class="small">AP and STA are enabled</div></header>
<main>
<section class="grid">
<div class="card"><div class="label">Time</div><div class="value" id="time">-</div></div>
<div class="card"><div class="label">STA</div><div class="value" id="sta">-</div><div class="small" id="staip"></div></div>
<div class="card"><div class="label">AP</div><div class="value" id="ap">-</div><div class="small" id="apip"></div></div>
</section>
<section class="grid">
<div class="card"><div class="label">SD Card</div><div class="value" id="sd">-</div><div class="small" id="sdinfo"></div></div>
<div class="card"><div class="label">ADC 1</div><div class="value" id="adc0">-</div></div>
<div class="card"><div class="label">ADC 2</div><div class="value" id="adc1">-</div></div>
<div class="card"><div class="label">ADC 3</div><div class="value" id="adc2">-</div></div>
</section>
<section class="grid">
<div class="card"><div class="label">RS485</div><div class="value" id="rs485">-</div><div class="small" id="rs485t"></div><div class="row"><button onclick="setMode('rs485','send')">Send</button><button onclick="setMode('rs485','receive')">Receive</button><button onclick="setMode('rs485','stop')">Stop</button></div></div>
<div class="card"><div class="label">CAN</div><div class="value" id="can">-</div><div class="small" id="cant"></div><div class="row"><button onclick="setMode('can','send')">Send</button><button onclick="setMode('can','receive')">Receive</button><button onclick="setMode('can','stop')">Stop</button></div></div>
</section>
</main>
<script>
async function setMode(target,mode){await fetch(`/api/set?target=${target}&mode=${mode}`);refresh();}
async function refresh(){
 const s=await (await fetch('/api/status')).json();
 time.textContent=s.time; sta.textContent=s.wifi.sta?'Connected':'Connecting'; sta.className='value '+(s.wifi.sta?'ok':'bad');
 staip.textContent=s.wifi.sta_ip; ap.textContent=s.wifi.ap_ssid; apip.textContent=s.wifi.ap_ip;
 sd.textContent=s.sd.mounted?'Mounted':'Not mounted'; sd.className='value '+(s.sd.mounted?'ok':'bad'); sdinfo.textContent=`${s.sd.type} ${s.sd.size_mb} MB`;
 adc0.textContent=s.adc[0]+' mV'; adc1.textContent=s.adc[1]+' mV'; adc2.textContent=s.adc[2]+' mV';
 rs485.textContent=s.rs485.mode; rs485.className='value '+(s.rs485.ok?'ok':'bad'); rs485t.textContent=s.rs485.mode==='send'?`total ${s.rs485.total} B`:`total ${s.rs485.total} B | crc error ${s.rs485.crc_errors} | seq error ${s.rs485.sequence_errors}`;
 can.textContent=s.can.mode; can.className='value '+(s.can.ok?'ok':'bad'); cant.textContent=`total ${s.can.total} B | bus error ${s.can.bus_errors}`;
}
setInterval(refresh,1000);refresh();
</script>
</body>
</html>
)HTML";

esp_err_t RootHandler(httpd_req_t* request)
{
  httpd_resp_set_type(request, "text/html");
  return httpd_resp_send(request, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

esp_err_t StatusHandler(httpd_req_t* request)
{
  const std::string json = StatusJson();
  httpd_resp_set_type(request, "application/json");
  return httpd_resp_send(request, json.c_str(), json.size());
}

TestMode ParseMode(const char* mode)
{
  if (std::strcmp(mode, "send") == 0) {
    return TestMode::kSend;
  }
  if (std::strcmp(mode, "receive") == 0) {
    return TestMode::kReceive;
  }
  if (std::strcmp(mode, "stop") == 0) {
    return TestMode::kOff;
  }
  return TestMode::kOff;
}

esp_err_t SetHandler(httpd_req_t* request)
{
  char query[96] = {};
  char target[16] = {};
  char mode[16] = {};
  if (httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK) {
    httpd_query_key_value(query, "target", target, sizeof(target));
    httpd_query_key_value(query, "mode", mode, sizeof(mode));
  }

  if (std::strcmp(target, "rs485") == 0) {
    g_rs485_mode = ParseMode(mode);
  } else if (std::strcmp(target, "can") == 0) {
    g_can_mode = ParseMode(mode);
  }

  httpd_resp_set_type(request, "application/json");
  return httpd_resp_sendstr(request, "{\"ok\":true}");
}

void StartHttpServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_handle_t server = nullptr;
  ESP_ERROR_CHECK(httpd_start(&server, &config));

  httpd_uri_t root_uri = {};
  root_uri.uri = "/";
  root_uri.method = HTTP_GET;
  root_uri.handler = RootHandler;
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_uri));

  httpd_uri_t status_uri = {};
  status_uri.uri = "/api/status";
  status_uri.method = HTTP_GET;
  status_uri.handler = StatusHandler;
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status_uri));

  httpd_uri_t set_uri = {};
  set_uri.uri = "/api/set";
  set_uri.method = HTTP_GET;
  set_uri.handler = SetHandler;
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &set_uri));
}

}  // namespace

extern "C" void app_main(void)
{
  printf("Ciallo\n");
  InitAdcLed();
  InitWifi();
  StartHttpServer();

  xTaskCreate(Ws2812Task, "ws2812_task", kTaskStackSize, nullptr,
              kTaskPriority, nullptr);
  xTaskCreate(AdcTask, "adc_task", kTaskStackSize, nullptr, kTaskPriority,
              nullptr);
  xTaskCreate(SdTask, "sd_task", kTaskStackSize, nullptr, kTaskPriority,
              nullptr);
  xTaskCreate(Rs485Task, "rs485_task", kTaskStackSize, nullptr,
              kTaskPriority, nullptr);
  xTaskCreate(CanTask, "can_task", kTaskStackSize, nullptr, kTaskPriority,
              nullptr);

  printf("HTTP control center: connect AP %s password %s, open http://%s\n",
         kWifiApSsid, kWifiApPassword, g_ap_ip);
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
