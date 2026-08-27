/*
 * @Description: General board test
 * @Author: LILYGO_L
 * @Date: 2026-06-12 17:30:00
 * @LastEditTime: 2026-08-27 12:00:00
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
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_mac.h"
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

extern "C" esp_err_t __wrap_sdmmc_init_spi_crc(sdmmc_card_t* card)
{
  (void)card;
  return ESP_OK;
}

namespace {

namespace t_can485_c5 = lilygo_device_driver::t_can485_c5;

constexpr char kWifiStaSsid[] = "LilyGo-AABB";
constexpr char kWifiStaPassword[] = "xinyuandianzi";
constexpr char kWifiApSsid[] = "T-CAN485-C5";
constexpr char kWifiApPassword[] = "12345678";
constexpr int kWifiApChannel = 6;
constexpr int kWifiApMaxConnection = 4;
constexpr int kWifiInfoPeriodMs = 1000;
constexpr int kTimeUpdatePeriodMs = 20000;

constexpr char kMountPoint[] = "/sdcard";
constexpr int kSdStatusPollPeriodMs = 3000;
constexpr int kSdRetryPeriodMs = 5000;
constexpr int kSdSpiMaxFreqKhz = 10000;

constexpr spi_host_device_t kSharedSpiHost = SPI2_HOST;
constexpr int kW5500SpiClockMhz = 10;
constexpr int kW5500RxTaskStackSize = 4096;
constexpr int kW5500ResetAssertMs = 10;
constexpr int kW5500ResetRecoveryMs = 100;

constexpr int kDigitalInputPollPeriodMs = 20;
constexpr int kButtonPollPeriodMs = 20;
constexpr int kButtonDebounceSamples = 3;

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

constexpr int kWs2812ResolutionHz = 10 * 1000 * 1000;
constexpr uint8_t kWs2812Brightness = 32;
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

constexpr std::array<Rgb, 5> kWs2812Colors = {
    Rgb{255, 0, 0},
    Rgb{0, 255, 0},
    Rgb{0, 0, 255},
    Rgb{255, 255, 255},
    Rgb{0, 0, 0},
};
constexpr std::array<const char*, 5> kWs2812ColorNames = {
    "red", "green", "blue", "white", "black",
};
constexpr size_t kWs2812InitialColorIndex = kWs2812Colors.size() - 1;

struct AdcInput {
  const char* name;
  int gpio;
  adc_unit_t unit = ADC_UNIT_1;
  adc_channel_t channel = ADC_CHANNEL_0;
  adc_cali_handle_t calibration = nullptr;
};

struct EthernetState {
  volatile bool started = false;
  volatile bool link_up = false;
  volatile bool has_ip = false;
  char mac[18] = "00:00:00:00:00:00";
  char ip[16] = "0.0.0.0";
  char netmask[16] = "0.0.0.0";
  char gateway[16] = "0.0.0.0";
  char dns[16] = "0.0.0.0";
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

int g_adc_pin_mv = -1;
int g_adc_input_mv = -1;
volatile bool g_digital_input_active = false;
volatile bool g_boot_button_pressed = false;
bool g_sd_mounted = false;
uint64_t g_sd_size_mb = 0;
uint32_t g_sd_sector_size = 0;
uint64_t g_sd_sector_count = 0;
char g_sd_type[24] = "Not mounted";
char g_sd_name[16] = "-";
char g_sd_error[96] = "No card";
char g_sta_ip[16] = "0.0.0.0";
char g_sta_ssid[33] = "-";
int8_t g_sta_rssi = 0;
char g_ap_ip[16] = "192.168.4.1";
char g_ap_ssid[33] = "T-CAN485-C5";
char g_time_text[32] = "syncing";
uint32_t g_time_age_seconds = 0;
char g_status_json[3072] = {};

EthernetState g_ethernet;
esp_eth_handle_t g_eth_handle = nullptr;
esp_netif_t* g_eth_netif = nullptr;

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

bool g_shared_spi_bus_ready = false;
sdmmc_card_t* g_sd_card = nullptr;
twai_node_handle_t g_can_node = nullptr;
QueueHandle_t g_can_rx_queue = nullptr;
volatile uint32_t g_can_error_flags = 0;
volatile twai_error_state_t g_can_state = TWAI_ERROR_ACTIVE;
volatile bool g_can_recovering = false;
int64_t g_can_last_recover_us = 0;
led_strip_handle_t g_ws2812 = nullptr;
size_t g_ws2812_color_index = kWs2812InitialColorIndex;

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

esp_err_t SetWs2812Color(const Rgb& color)
{
  if (g_ws2812 == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = led_strip_set_pixel(g_ws2812, 0, ScaleColor(color.red),
                                      ScaleColor(color.green),
                                      ScaleColor(color.blue));
  if (err != ESP_OK) {
    return err;
  }
  return led_strip_refresh(g_ws2812);
}

esp_err_t InitWs2812()
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
  esp_err_t err =
      led_strip_new_rmt_device(&strip_config, &rmt_config, &g_ws2812);
  if (err != ESP_OK) {
    return err;
  }
  g_ws2812_color_index = kWs2812InitialColorIndex;
  return SetWs2812Color(kWs2812Colors[g_ws2812_color_index]);
}

esp_err_t NextWs2812Color()
{
  const size_t next_color_index =
      (g_ws2812_color_index + 1) % kWs2812Colors.size();
  const esp_err_t err = SetWs2812Color(kWs2812Colors[next_color_index]);
  if (err == ESP_OK) {
    g_ws2812_color_index = next_color_index;
  }
  return err;
}

const char* Ws2812ColorName()
{
  return kWs2812ColorNames[g_ws2812_color_index];
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

int ScaleInputVoltage(int adc_voltage_mv)
{
  return adc_voltage_mv *
         t_can485_c5::device::adc::kInputScaleNumerator /
         t_can485_c5::device::adc::kInputScaleDenominator;
}

void AdcTask(void* param)
{
  (void)param;
  adc_oneshot_unit_init_cfg_t unit_config = {};
  unit_config.unit_id = ADC_UNIT_1;
  adc_oneshot_unit_handle_t adc_handle = nullptr;
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

  AdcInput input{"0-24V", t_can485_c5::gpio::adc::kVoltage};
  ESP_ERROR_CHECK(InitAdcInput(adc_handle, &input) ? ESP_OK : ESP_FAIL);

  while (true) {
    g_adc_pin_mv = ReadVoltage(adc_handle, input);
    if (g_adc_pin_mv >= 0) {
      g_adc_input_mv = ScaleInputVoltage(g_adc_pin_mv);
    }
    vTaskDelay(pdMS_TO_TICKS(kAdcReadPeriodMs));
  }
}

void InitDigitalInput()
{
  gpio_config_t config = {};
  config.pin_bit_mask =
      1ULL << t_can485_c5::gpio::digital_input::kOptocoupler;
  config.mode = GPIO_MODE_INPUT;
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&config));
}

void DigitalInputTask(void* param)
{
  (void)param;
  while (true) {
    g_digital_input_active =
        gpio_get_level(static_cast<gpio_num_t>(
            t_can485_c5::gpio::digital_input::kOptocoupler)) ==
        t_can485_c5::device::digital_input::kActiveLevel;
    vTaskDelay(pdMS_TO_TICKS(kDigitalInputPollPeriodMs));
  }
}

void InitBootButton()
{
  gpio_config_t config = {};
  config.pin_bit_mask =
      1ULL << t_can485_c5::gpio::button::kEsp32c5Boot;
  config.mode = GPIO_MODE_INPUT;
  config.pull_up_en = GPIO_PULLUP_ENABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&config));
  g_boot_button_pressed =
      gpio_get_level(static_cast<gpio_num_t>(
          t_can485_c5::gpio::button::kEsp32c5Boot)) == 0;
}

void ButtonTask(void* param)
{
  (void)param;
  bool candidate = g_boot_button_pressed;
  int candidate_count = 0;

  while (true) {
    const bool pressed =
        gpio_get_level(static_cast<gpio_num_t>(
            t_can485_c5::gpio::button::kEsp32c5Boot)) == 0;
    if (pressed == candidate) {
      ++candidate_count;
    } else {
      candidate = pressed;
      candidate_count = 1;
    }
    if (candidate_count >= kButtonDebounceSamples) {
      g_boot_button_pressed = candidate;
      candidate_count = kButtonDebounceSamples;
    }
    vTaskDelay(pdMS_TO_TICKS(kButtonPollPeriodMs));
  }
}

bool InitSharedSpiBus()
{
  if (g_shared_spi_bus_ready) {
    return true;
  }

  gpio_config_t cs_config = {};
  cs_config.pin_bit_mask =
      (1ULL << t_can485_c5::gpio::sd::kCs) |
      (1ULL << t_can485_c5::gpio::w5500::kCs);
  cs_config.mode = GPIO_MODE_OUTPUT;
  cs_config.pull_up_en = GPIO_PULLUP_DISABLE;
  cs_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cs_config.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&cs_config) != ESP_OK ||
      gpio_set_level(static_cast<gpio_num_t>(t_can485_c5::gpio::sd::kCs),
                     1) != ESP_OK ||
      gpio_set_level(
          static_cast<gpio_num_t>(t_can485_c5::gpio::w5500::kCs), 1) !=
          ESP_OK) {
    return false;
  }

  spi_bus_config_t bus_config = {};
  bus_config.mosi_io_num = t_can485_c5::gpio::spi::kMosi;
  bus_config.miso_io_num = t_can485_c5::gpio::spi::kMiso;
  bus_config.sclk_io_num = t_can485_c5::gpio::spi::kSclk;
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.max_transfer_sz = 4000;

  esp_err_t err =
      spi_bus_initialize(kSharedSpiHost, &bus_config, SPI_DMA_CH_AUTO);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return false;
  }
  g_shared_spi_bus_ready = true;
  return true;
}

void UpdateEthernetIpInfo()
{
  if (g_eth_netif == nullptr) {
    return;
  }

  esp_netif_ip_info_t ip_info = {};
  if (esp_netif_get_ip_info(g_eth_netif, &ip_info) == ESP_OK) {
    std::snprintf(g_ethernet.ip, sizeof(g_ethernet.ip), IPSTR,
                  IP2STR(&ip_info.ip));
    std::snprintf(g_ethernet.netmask, sizeof(g_ethernet.netmask), IPSTR,
                  IP2STR(&ip_info.netmask));
    std::snprintf(g_ethernet.gateway, sizeof(g_ethernet.gateway), IPSTR,
                  IP2STR(&ip_info.gw));
  }

  esp_netif_dns_info_t dns_info = {};
  if (esp_netif_get_dns_info(g_eth_netif, ESP_NETIF_DNS_MAIN, &dns_info) ==
          ESP_OK &&
      dns_info.ip.type == ESP_IPADDR_TYPE_V4) {
    std::snprintf(g_ethernet.dns, sizeof(g_ethernet.dns), IPSTR,
                  IP2STR(&dns_info.ip.u_addr.ip4));
  }
}

void ClearEthernetIpInfo()
{
  std::snprintf(g_ethernet.ip, sizeof(g_ethernet.ip), "%s", "0.0.0.0");
  std::snprintf(g_ethernet.netmask, sizeof(g_ethernet.netmask), "%s",
                "0.0.0.0");
  std::snprintf(g_ethernet.gateway, sizeof(g_ethernet.gateway), "%s",
                "0.0.0.0");
  std::snprintf(g_ethernet.dns, sizeof(g_ethernet.dns), "%s", "0.0.0.0");
}

void EthernetEventHandler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
  (void)arg;
  (void)event_base;
  (void)event_data;

  switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
      g_ethernet.link_up = true;
      printf("[ethernet] link up\n");
      break;
    case ETHERNET_EVENT_DISCONNECTED:
      g_ethernet.link_up = false;
      g_ethernet.has_ip = false;
      ClearEthernetIpInfo();
      printf("[ethernet] link down\n");
      break;
    case ETHERNET_EVENT_START:
      g_ethernet.started = true;
      printf("[ethernet] started\n");
      break;
    case ETHERNET_EVENT_STOP:
      g_ethernet.started = false;
      g_ethernet.link_up = false;
      g_ethernet.has_ip = false;
      ClearEthernetIpInfo();
      printf("[ethernet] stopped\n");
      break;
    default:
      break;
  }
}

void EthernetIpEventHandler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
  (void)arg;
  (void)event_base;

  if (event_id == IP_EVENT_ETH_GOT_IP) {
    const ip_event_got_ip_t* event =
        static_cast<const ip_event_got_ip_t*>(event_data);
    g_ethernet.has_ip = true;
    std::snprintf(g_ethernet.ip, sizeof(g_ethernet.ip), IPSTR,
                  IP2STR(&event->ip_info.ip));
    std::snprintf(g_ethernet.netmask, sizeof(g_ethernet.netmask), IPSTR,
                  IP2STR(&event->ip_info.netmask));
    std::snprintf(g_ethernet.gateway, sizeof(g_ethernet.gateway), IPSTR,
                  IP2STR(&event->ip_info.gw));
    UpdateEthernetIpInfo();
    printf("[ethernet] DHCP IP %s\n", g_ethernet.ip);
  } else if (event_id == IP_EVENT_ETH_LOST_IP) {
    g_ethernet.has_ip = false;
    ClearEthernetIpInfo();
    printf("[ethernet] DHCP address lost\n");
  }
}

esp_err_t ResetW5500()
{
  gpio_config_t reset_config = {};
  reset_config.pin_bit_mask = 1ULL << t_can485_c5::gpio::w5500::kReset;
  reset_config.mode = GPIO_MODE_OUTPUT;
  reset_config.pull_up_en = GPIO_PULLUP_DISABLE;
  reset_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  reset_config.intr_type = GPIO_INTR_DISABLE;

  esp_err_t err = gpio_config(&reset_config);
  if (err != ESP_OK) {
    return err;
  }
  err = gpio_set_level(
      static_cast<gpio_num_t>(t_can485_c5::gpio::w5500::kReset), 0);
  if (err != ESP_OK) {
    return err;
  }
  vTaskDelay(pdMS_TO_TICKS(kW5500ResetAssertMs));
  err = gpio_set_level(
      static_cast<gpio_num_t>(t_can485_c5::gpio::w5500::kReset), 1);
  if (err != ESP_OK) {
    return err;
  }
  vTaskDelay(pdMS_TO_TICKS(kW5500ResetRecoveryMs));
  return ESP_OK;
}

esp_err_t InitEthernet()
{
  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return err;
  }

  err = ResetW5500();
  if (err != ESP_OK) {
    return err;
  }
  if (!InitSharedSpiBus()) {
    return ESP_FAIL;
  }

  spi_device_interface_config_t device_config = {};
  device_config.mode = 0;
  device_config.clock_speed_hz = kW5500SpiClockMhz * 1000 * 1000;
  device_config.spics_io_num = t_can485_c5::gpio::w5500::kCs;
  device_config.queue_size = 20;

  eth_w5500_config_t w5500_config =
      ETH_W5500_DEFAULT_CONFIG(kSharedSpiHost, &device_config);
  w5500_config.int_gpio_num = t_can485_c5::gpio::w5500::kInterrupt;

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  mac_config.rx_task_stack_size = kW5500RxTaskStackSize;
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = t_can485_c5::device::w5500::kPhyAddress;
  phy_config.reset_gpio_num = -1;

  esp_eth_mac_t* mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
  esp_eth_phy_t* phy = esp_eth_phy_new_w5500(&phy_config);
  if (mac == nullptr || phy == nullptr) {
    if (mac != nullptr) {
      mac->del(mac);
    }
    if (phy != nullptr) {
      phy->del(phy);
    }
    return ESP_ERR_NO_MEM;
  }

  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
  err = esp_eth_driver_install(&eth_config, &g_eth_handle);
  if (err != ESP_OK) {
    mac->del(mac);
    phy->del(phy);
    return err;
  }

  uint8_t base_mac[ETH_ADDR_LEN] = {};
  uint8_t local_mac[ETH_ADDR_LEN] = {};
  ESP_ERROR_CHECK(esp_read_mac(base_mac, ESP_MAC_ETH));
  ESP_ERROR_CHECK(esp_derive_local_mac(local_mac, base_mac));
  ESP_ERROR_CHECK(esp_eth_ioctl(g_eth_handle, ETH_CMD_S_MAC_ADDR, local_mac));
  std::snprintf(g_ethernet.mac, sizeof(g_ethernet.mac),
                "%02X:%02X:%02X:%02X:%02X:%02X", local_mac[0],
                local_mac[1], local_mac[2], local_mac[3], local_mac[4],
                local_mac[5]);

  esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
  g_eth_netif = esp_netif_new(&netif_config);
  if (g_eth_netif == nullptr) {
    return ESP_ERR_NO_MEM;
  }
  ESP_ERROR_CHECK(
      esp_netif_set_hostname(g_eth_netif, "t-can485-c5-ethernet"));
  auto eth_glue = esp_eth_new_netif_glue(g_eth_handle);
  if (eth_glue == nullptr) {
    return ESP_ERR_NO_MEM;
  }
  ESP_ERROR_CHECK(esp_netif_attach(g_eth_netif, eth_glue));
  ESP_ERROR_CHECK(esp_event_handler_register(
      ETH_EVENT, ESP_EVENT_ANY_ID, &EthernetEventHandler, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(
      IP_EVENT, ESP_EVENT_ANY_ID, &EthernetIpEventHandler, nullptr));
  return esp_eth_start(g_eth_handle);
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

void ClearSdCardInfo(const char* error)
{
  g_sd_mounted = false;
  g_sd_size_mb = 0;
  g_sd_sector_size = 0;
  g_sd_sector_count = 0;
  std::snprintf(g_sd_type, sizeof(g_sd_type), "%s", "Not mounted");
  std::snprintf(g_sd_name, sizeof(g_sd_name), "%s", "-");
  std::snprintf(g_sd_error, sizeof(g_sd_error), "%s", error);
}

bool MountSdCard()
{
  if (g_sd_mounted && g_sd_card != nullptr) {
    return true;
  }
  if (!InitSharedSpiBus()) {
    ClearSdCardInfo("SPI bus init failed");
    return false;
  }

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = kSharedSpiHost;
  host.max_freq_khz = kSdSpiMaxFreqKhz;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.host_id = static_cast<spi_host_device_t>(host.slot);
  slot_config.gpio_cs = static_cast<gpio_num_t>(t_can485_c5::gpio::sd::kCs);

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
  mount_config.format_if_mount_failed = false;
  mount_config.max_files = 4;
  mount_config.allocation_unit_size = 16 * 1024;
  mount_config.disk_status_check_enable = true;

  esp_err_t err = esp_vfs_fat_sdspi_mount(kMountPoint, &host, &slot_config,
                                          &mount_config, &g_sd_card);
  if (err != ESP_OK) {
    g_sd_card = nullptr;
    char error[96] = {};
    std::snprintf(error, sizeof(error), "%s (0x%x)", esp_err_to_name(err),
                  static_cast<unsigned>(err));
    ClearSdCardInfo(error);
    return false;
  }

  g_sd_mounted = true;
  g_sd_size_mb = static_cast<uint64_t>(g_sd_card->csd.capacity) *
                 g_sd_card->csd.sector_size / (1024ULL * 1024ULL);
  g_sd_sector_size = g_sd_card->csd.sector_size;
  g_sd_sector_count = g_sd_card->csd.capacity;
  std::snprintf(g_sd_type, sizeof(g_sd_type), "%s",
                CardTypeName(g_sd_card));
  std::snprintf(g_sd_name, sizeof(g_sd_name), "%.5s",
                reinterpret_cast<const char*>(g_sd_card->cid.name));
  std::snprintf(g_sd_error, sizeof(g_sd_error), "%s", "OK");
  return true;
}

bool MountedSdCardStillReadable()
{
  return g_sd_mounted && g_sd_card != nullptr &&
         sdmmc_get_status(g_sd_card) == ESP_OK;
}

void UnmountSdCard(const char* error)
{
  if (g_sd_mounted && g_sd_card != nullptr) {
    esp_vfs_fat_sdcard_unmount(kMountPoint, g_sd_card);
  }
  g_sd_card = nullptr;
  ClearSdCardInfo(error);
}

void SdTask(void* param)
{
  (void)param;
  while (true) {
    int delay_ms = kSdStatusPollPeriodMs;
    if (!g_sd_mounted) {
      if (MountSdCard()) {
        printf("[sd] mounted: %s, %llu MB\n", g_sd_name,
               static_cast<unsigned long long>(g_sd_size_mb));
      }
      delay_ms = kSdRetryPeriodMs;
    } else if (!MountedSdCardStillReadable()) {
      printf("[sd] removed or unavailable; waiting for card\n");
      UnmountSdCard("Removed; waiting for card");
      delay_ms = kSdRetryPeriodMs;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
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
  esp_sntp_set_sync_interval(kTimeUpdatePeriodMs);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_setservername(1, "time.google.com");
  esp_sntp_init();
}

void UpdateTimeText()
{
  std::time_t now = 0;
  std::time(&now);
  struct tm time_info = {};
  localtime_r(&now, &time_info);
  if (time_info.tm_year < (2024 - 1900)) {
    std::snprintf(g_time_text, sizeof(g_time_text), "%s", "syncing");
    return;
  }

  std::strftime(g_time_text, sizeof(g_time_text), "%Y-%m-%d %H:%M:%S",
                &time_info);
  g_time_age_seconds = 0;
}

void TimeTask(void* param)
{
  (void)param;
  uint32_t elapsed_ms = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    elapsed_ms += 1000;
    if (elapsed_ms >= kTimeUpdatePeriodMs) {
      UpdateTimeText();
      elapsed_ms = 0;
    } else if (std::strcmp(g_time_text, "syncing") != 0) {
      ++g_time_age_seconds;
    }
  }
}

void WifiInfoTask(void* param)
{
  (void)param;
  while (true) {
    wifi_ap_record_t ap_info = {};
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
      g_wifi_sta_connected = true;
      std::snprintf(g_sta_ssid, sizeof(g_sta_ssid), "%.32s",
                    reinterpret_cast<const char*>(ap_info.ssid));
      g_sta_rssi = ap_info.rssi;
    } else {
      g_wifi_sta_connected = false;
      std::snprintf(g_sta_ssid, sizeof(g_sta_ssid), "%s", "-");
      g_sta_rssi = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(kWifiInfoPeriodMs));
  }
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
    std::snprintf(g_sta_ssid, sizeof(g_sta_ssid), "%s", "-");
    g_sta_rssi = 0;
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    const ip_event_got_ip_t* event =
        static_cast<const ip_event_got_ip_t*>(event_data);
    g_wifi_sta_connected = true;
    std::snprintf(g_sta_ssid, sizeof(g_sta_ssid), "%s", kWifiStaSsid);
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
  uint8_t ap_mac[6] = {};
  ESP_ERROR_CHECK(esp_read_mac(ap_mac, ESP_MAC_WIFI_SOFTAP));
  std::snprintf(g_ap_ssid, sizeof(g_ap_ssid),
                "%s_%02X%02X%02X%02X%02X%02X", kWifiApSsid, ap_mac[0],
                ap_mac[1], ap_mac[2], ap_mac[3], ap_mac[4], ap_mac[5]);
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
  size_t ap_ssid_len = std::strlen(g_ap_ssid);
  if (ap_ssid_len > sizeof(ap_config.ap.ssid)) {
    ap_ssid_len = sizeof(ap_config.ap.ssid);
  }
  std::memcpy(ap_config.ap.ssid, g_ap_ssid, ap_ssid_len);
  ap_config.ap.ssid_len = ap_ssid_len;
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

const char* StatusJson()
{
  const TestMode rs485_mode = g_rs485_mode;
  const TestMode can_mode = g_can_mode;

  std::snprintf(
      g_status_json, sizeof(g_status_json),
      "{"
      "\"time\":\"%s\",\"time_age_seconds\":%lu,"
      "\"wifi\":{\"sta\":%s,\"sta_ssid\":\"%s\",\"sta_ip\":\"%s\","
      "\"sta_rssi\":%d,\"ap_ssid\":\"%s\",\"ap_ip\":\"%s\"},"
      "\"ethernet\":{\"started\":%s,\"link\":%s,\"has_ip\":%s,"
      "\"mac\":\"%s\",\"ip\":\"%s\",\"netmask\":\"%s\","
      "\"gateway\":\"%s\",\"dns\":\"%s\"},"
      "\"sd\":{\"mounted\":%s,\"name\":\"%s\",\"type\":\"%s\","
      "\"size_mb\":%llu,\"sector_size\":%lu,\"sector_count\":%llu,"
      "\"error\":\"%s\"},"
      "\"adc\":{\"measured_mv\":%d,\"theoretical_mv\":%d},"
      "\"digital_input\":{\"active\":%s},"
      "\"button\":{\"pressed\":%s,\"gpio\":%d},"
      "\"ws2812\":{\"color\":\"%s\"},"
      "\"rs485\":{\"mode\":\"%s\",\"total\":%u,\"ok\":%s,"
      "\"crc_errors\":%lu,\"sequence_errors\":%lu},"
      "\"can\":{\"mode\":\"%s\",\"total\":%u,\"ok\":%s,\"state\":\"%s\","
      "\"bus_errors\":%lu}"
      "}",
      g_time_text, static_cast<unsigned long>(g_time_age_seconds),
      g_wifi_sta_connected ? "true" : "false", g_sta_ssid,
      g_sta_ip, static_cast<int>(g_sta_rssi), g_ap_ssid, g_ap_ip,
      g_ethernet.started ? "true" : "false",
      g_ethernet.link_up ? "true" : "false",
      g_ethernet.has_ip ? "true" : "false", g_ethernet.mac, g_ethernet.ip,
      g_ethernet.netmask, g_ethernet.gateway, g_ethernet.dns,
      g_sd_mounted ? "true" : "false", g_sd_name, g_sd_type,
      static_cast<unsigned long long>(g_sd_size_mb),
      static_cast<unsigned long>(g_sd_sector_size),
      static_cast<unsigned long long>(g_sd_sector_count), g_sd_error,
      g_adc_pin_mv, g_adc_input_mv,
      g_digital_input_active ? "true" : "false",
      g_boot_button_pressed ? "true" : "false",
      t_can485_c5::gpio::button::kEsp32c5Boot,
      Ws2812ColorName(), ModeName(rs485_mode),
      static_cast<unsigned>(g_rs485_total_size),
      g_rs485_data_ok ? "true" : "false",
      static_cast<unsigned long>(g_rs485_crc_error_count),
      static_cast<unsigned long>(g_rs485_sequence_error_count),
      ModeName(can_mode),
      static_cast<unsigned>(g_can_total_size), g_can_data_ok ? "true" : "false",
      g_can_state_text, static_cast<unsigned long>(g_can_bus_error_count));
  return g_status_json;
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
.input-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:16px;margin-top:12px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
button{background:#263746;color:#e8eef2;border:1px solid #41566a;border-radius:6px;padding:10px 14px}
button.active{background:#1f8f5f;border-color:#32b878}
.ok{color:#63d68a}.bad{color:#ff7c7c}.small{font-size:14px;color:#a9bac7}
</style>
</head>
<body>
<header><h1>T-CAN485-C5 Control Center</h1><div class="small">W5500 Ethernet and Wi-Fi AP/STA are enabled</div></header>
<main>
<section class="grid">
<div class="card"><div class="label">AP</div><div class="value" id="ap">-</div><div class="small" id="apip"></div></div>
<div class="card"><div class="label">STA</div><div class="value" id="sta">-</div><div class="small" id="staip"></div></div>
<div class="card"><div class="label">Time</div><div class="value" id="time">-</div><div class="small" id="timeage"></div></div>
<div class="card"><div class="label">W5500 Ethernet (DHCP)</div><div class="value" id="eth">-</div><div class="small" id="ethinfo"></div></div>
</section>
<section class="grid">
<div class="card"><div class="label">SD Card</div><div class="value" id="sd">-</div><div class="small" id="sdinfo"></div></div>
<div class="card"><div class="label">0-24V Inputs</div><div class="input-grid"><div><div class="small">GPIO4 measured voltage</div><div class="value" id="adcmeasured">-</div></div><div><div class="small">P2 theoretical input voltage</div><div class="value" id="adctheoretical">-</div></div><div><div class="small">PC817 digital input on GPIO24, active low</div><div class="value" id="digitalinput">-</div></div></div></div>
<div class="card"><div class="label">BOOT Button</div><div class="value" id="bootbutton">-</div><div class="small" id="bootbuttoninfo"></div></div>
<div class="card"><div class="label">WS2812</div><div class="value" id="ws2812">-</div><div class="small">GPIO3, red / green / blue / white / black</div><div class="row"><button onclick="setMode('ws2812','next')">Next color</button></div></div>
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
 const eth=document.getElementById('eth'); const ethinfo=document.getElementById('ethinfo'); const adcmeasured=document.getElementById('adcmeasured'); const adctheoretical=document.getElementById('adctheoretical'); const digitalinput=document.getElementById('digitalinput'); const bootbutton=document.getElementById('bootbutton'); const bootbuttoninfo=document.getElementById('bootbuttoninfo'); const ws2812=document.getElementById('ws2812');
 eth.textContent=s.ethernet.has_ip?'Ready':(s.ethernet.link?'DHCP':'Link down'); eth.className='value '+(s.ethernet.has_ip?'ok':'bad');
 ethinfo.innerHTML=`MAC ${s.ethernet.mac}<br>IP ${s.ethernet.ip} | mask ${s.ethernet.netmask} | gateway ${s.ethernet.gateway} | DNS ${s.ethernet.dns}`;
 time.textContent=s.time; timeage.textContent=`updated ${s.time_age_seconds} s ago`; sta.textContent=s.wifi.sta?s.wifi.sta_ssid:'Connecting'; sta.className='value '+(s.wifi.sta?'ok':'bad');
 staip.textContent=s.wifi.sta?`${s.wifi.sta_ip} | RSSI ${s.wifi.sta_rssi} dBm`:s.wifi.sta_ip; ap.textContent=s.wifi.ap_ssid; apip.textContent=s.wifi.ap_ip;
 sd.textContent=s.sd.mounted?'Detected':'Not detected'; sd.className='value '+(s.sd.mounted?'ok':'bad'); sdinfo.textContent=`name ${s.sd.name} | ${s.sd.type} | ${s.sd.size_mb} MB | sector ${s.sd.sector_size} B | count ${s.sd.sector_count} | ${s.sd.error}`;
 adcmeasured.textContent=s.adc.measured_mv>=0?`${(s.adc.measured_mv/1000).toFixed(3)} V`:'Unavailable'; adcmeasured.className='value '+(s.adc.measured_mv>=0?'ok':'bad');
 adctheoretical.textContent=s.adc.theoretical_mv>=0?`${(s.adc.theoretical_mv/1000).toFixed(2)} V`:'Unavailable'; adctheoretical.className='value '+(s.adc.theoretical_mv>=0?'ok':'bad');
 digitalinput.textContent=s.digital_input.active?'Active':'Inactive'; digitalinput.className='value '+(s.digital_input.active?'ok':'');
 bootbutton.textContent=s.button.pressed?'Pressed':'Released'; bootbutton.className='value '+(s.button.pressed?'ok':'bad'); bootbuttoninfo.textContent=`GPIO ${s.button.gpio} | active low`;
 ws2812.textContent=s.ws2812.color.charAt(0).toUpperCase()+s.ws2812.color.slice(1);
 const rs485Mode=s.rs485.mode.charAt(0).toUpperCase()+s.rs485.mode.slice(1);
 const canMode=s.can.mode.charAt(0).toUpperCase()+s.can.mode.slice(1);
 rs485.textContent=rs485Mode; rs485.className='value '+(s.rs485.ok?'ok':'bad'); rs485t.textContent=s.rs485.mode==='send'?`total ${s.rs485.total} B`:`total ${s.rs485.total} B | crc error ${s.rs485.crc_errors} | seq error ${s.rs485.sequence_errors}`;
 can.textContent=canMode; can.className='value '+(s.can.ok?'ok':'bad'); cant.textContent=`total ${s.can.total} B | bus error ${s.can.bus_errors}`;
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
  const char* json = StatusJson();
  httpd_resp_set_type(request, "application/json");
  return httpd_resp_send(request, json, HTTPD_RESP_USE_STRLEN);
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
  } else if (std::strcmp(target, "ws2812") == 0 &&
             std::strcmp(mode, "next") == 0) {
    const esp_err_t err = NextWs2812Color();
    if (err != ESP_OK) {
      return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                 esp_err_to_name(err));
    }
  }

  httpd_resp_set_type(request, "application/json");
  return httpd_resp_sendstr(request, "{\"ok\":true}");
}

void StartHttpServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 8192;

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
  InitDigitalInput();
  InitBootButton();
  ESP_ERROR_CHECK(InitWs2812());
  InitWifi();
  UpdateTimeText();
  const esp_err_t ethernet_err = InitEthernet();
  if (ethernet_err != ESP_OK) {
    printf("[ethernet] initialization failed: %s (0x%x)\n",
           esp_err_to_name(ethernet_err),
           static_cast<unsigned>(ethernet_err));
  }
  StartHttpServer();
  printf("[sd] automatic detection started\n");

  xTaskCreate(TimeTask, "time_task", kTaskStackSize, nullptr, kTaskPriority,
              nullptr);
  xTaskCreate(WifiInfoTask, "wifi_info_task", kTaskStackSize, nullptr,
              kTaskPriority, nullptr);
  xTaskCreate(AdcTask, "adc_task", kTaskStackSize, nullptr, kTaskPriority,
              nullptr);
  xTaskCreate(DigitalInputTask, "digital_input_task", kTaskStackSize, nullptr,
              kTaskPriority, nullptr);
  xTaskCreate(ButtonTask, "button_task", kTaskStackSize, nullptr,
              kTaskPriority, nullptr);
  xTaskCreate(SdTask, "sd_task", kTaskStackSize, nullptr, kTaskPriority,
              nullptr);
  xTaskCreate(Rs485Task, "rs485_task", kTaskStackSize, nullptr,
              kTaskPriority, nullptr);
  xTaskCreate(CanTask, "can_task", kTaskStackSize, nullptr, kTaskPriority,
              nullptr);

  printf("HTTP control center: connect AP %s password %s, open http://%s\n",
         g_ap_ssid, kWifiApPassword, g_ap_ip);
  if (ethernet_err == ESP_OK) {
    printf("HTTP control center will also use the W5500 DHCP address\n");
  }
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
