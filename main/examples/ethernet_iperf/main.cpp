/*
 * @Description: W5500 Ethernet iperf test
 * @Author: LILYGO_L
 * @Date: 2026-08-26 12:00:00
 * @LastEditTime: 2026-08-26 12:00:00
 * @License: GPL 3.0
 */

#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iperf_cmd.h"
#include "lilygo_device_driver_library.h"

namespace {

namespace t_can485_c5 = lilygo_device_driver::t_can485_c5;

constexpr spi_host_device_t kW5500SpiHost = SPI2_HOST;
constexpr int kW5500SpiClockMhz = 10;
constexpr int kW5500RxTaskStackSize = 4096;
constexpr int kW5500ResetAssertMs = 10;
constexpr int kW5500ResetRecoveryMs = 100;

esp_eth_handle_t g_eth_handle = nullptr;
esp_netif_t* g_eth_netif = nullptr;
volatile bool g_eth_link_up = false;
char g_eth_mac[18] = "00:00:00:00:00:00";
char g_eth_ip[16] = "0.0.0.0";
char g_eth_netmask[16] = "0.0.0.0";
char g_eth_gateway[16] = "0.0.0.0";
char g_eth_dns[16] = "0.0.0.0";

void UpdateEthernetIpInfo()
{
  if (g_eth_netif == nullptr) {
    return;
  }

  esp_netif_ip_info_t ip_info = {};
  if (esp_netif_get_ip_info(g_eth_netif, &ip_info) == ESP_OK) {
    std::snprintf(g_eth_ip, sizeof(g_eth_ip), IPSTR, IP2STR(&ip_info.ip));
    std::snprintf(g_eth_netmask, sizeof(g_eth_netmask), IPSTR,
                  IP2STR(&ip_info.netmask));
    std::snprintf(g_eth_gateway, sizeof(g_eth_gateway), IPSTR,
                  IP2STR(&ip_info.gw));
  }

  esp_netif_dns_info_t dns_info = {};
  if (esp_netif_get_dns_info(g_eth_netif, ESP_NETIF_DNS_MAIN, &dns_info) ==
          ESP_OK &&
      dns_info.ip.type == ESP_IPADDR_TYPE_V4) {
    std::snprintf(g_eth_dns, sizeof(g_eth_dns), IPSTR,
                  IP2STR(&dns_info.ip.u_addr.ip4));
  }
}

void ClearEthernetIpInfo()
{
  std::snprintf(g_eth_ip, sizeof(g_eth_ip), "%s", "0.0.0.0");
  std::snprintf(g_eth_netmask, sizeof(g_eth_netmask), "%s", "0.0.0.0");
  std::snprintf(g_eth_gateway, sizeof(g_eth_gateway), "%s", "0.0.0.0");
  std::snprintf(g_eth_dns, sizeof(g_eth_dns), "%s", "0.0.0.0");
}

void EthernetEventHandler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
  (void)arg;
  (void)event_base;
  (void)event_data;

  switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
      g_eth_link_up = true;
      printf("[ethernet] link up\n");
      break;
    case ETHERNET_EVENT_DISCONNECTED:
      g_eth_link_up = false;
      ClearEthernetIpInfo();
      printf("[ethernet] link down\n");
      break;
    case ETHERNET_EVENT_START:
      printf("[ethernet] started\n");
      break;
    case ETHERNET_EVENT_STOP:
      g_eth_link_up = false;
      ClearEthernetIpInfo();
      printf("[ethernet] stopped\n");
      break;
    default:
      break;
  }
}

void IpEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id,
                    void* event_data)
{
  (void)arg;
  (void)event_base;

  if (event_id == IP_EVENT_ETH_GOT_IP) {
    const ip_event_got_ip_t* event =
        static_cast<const ip_event_got_ip_t*>(event_data);
    std::snprintf(g_eth_ip, sizeof(g_eth_ip), IPSTR,
                  IP2STR(&event->ip_info.ip));
    std::snprintf(g_eth_netmask, sizeof(g_eth_netmask), IPSTR,
                  IP2STR(&event->ip_info.netmask));
    std::snprintf(g_eth_gateway, sizeof(g_eth_gateway), IPSTR,
                  IP2STR(&event->ip_info.gw));
    UpdateEthernetIpInfo();
    printf("[ethernet] DHCP IP %s\n", g_eth_ip);
  } else if (event_id == IP_EVENT_ETH_LOST_IP) {
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

  spi_bus_config_t bus_config = {};
  bus_config.mosi_io_num = t_can485_c5::gpio::w5500::kMosi;
  bus_config.miso_io_num = t_can485_c5::gpio::w5500::kMiso;
  bus_config.sclk_io_num = t_can485_c5::gpio::w5500::kSclk;
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.data4_io_num = -1;
  bus_config.data5_io_num = -1;
  bus_config.data6_io_num = -1;
  bus_config.data7_io_num = -1;
  err = spi_bus_initialize(kW5500SpiHost, &bus_config, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    return err;
  }

  spi_device_interface_config_t device_config = {};
  device_config.mode = 0;
  device_config.clock_speed_hz = kW5500SpiClockMhz * 1000 * 1000;
  device_config.spics_io_num = t_can485_c5::gpio::w5500::kCs;
  device_config.queue_size = 20;

  eth_w5500_config_t w5500_config =
      ETH_W5500_DEFAULT_CONFIG(kW5500SpiHost, &device_config);
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
    spi_bus_free(kW5500SpiHost);
    return ESP_ERR_NO_MEM;
  }

  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
  err = esp_eth_driver_install(&eth_config, &g_eth_handle);
  if (err != ESP_OK) {
    mac->del(mac);
    phy->del(phy);
    spi_bus_free(kW5500SpiHost);
    return err;
  }

  uint8_t base_mac[ETH_ADDR_LEN] = {};
  uint8_t local_mac[ETH_ADDR_LEN] = {};
  ESP_ERROR_CHECK(esp_read_mac(base_mac, ESP_MAC_ETH));
  ESP_ERROR_CHECK(esp_derive_local_mac(local_mac, base_mac));
  ESP_ERROR_CHECK(esp_eth_ioctl(g_eth_handle, ETH_CMD_S_MAC_ADDR, local_mac));
  std::snprintf(g_eth_mac, sizeof(g_eth_mac),
                "%02X:%02X:%02X:%02X:%02X:%02X", local_mac[0],
                local_mac[1], local_mac[2], local_mac[3], local_mac[4],
                local_mac[5]);

  esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
  g_eth_netif = esp_netif_new(&netif_config);
  if (g_eth_netif == nullptr) {
    return ESP_ERR_NO_MEM;
  }
  ESP_ERROR_CHECK(
      esp_netif_set_hostname(g_eth_netif, "t-can485-c5-iperf"));
  auto eth_glue = esp_eth_new_netif_glue(g_eth_handle);
  if (eth_glue == nullptr) {
    return ESP_ERR_NO_MEM;
  }
  ESP_ERROR_CHECK(esp_netif_attach(g_eth_netif, eth_glue));
  ESP_ERROR_CHECK(esp_event_handler_register(
      ETH_EVENT, ESP_EVENT_ANY_ID, &EthernetEventHandler, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_register(
      IP_EVENT, ESP_EVENT_ANY_ID, &IpEventHandler, nullptr));
  return esp_eth_start(g_eth_handle);
}

int EthernetInfoCommand(int argc, char** argv)
{
  if (argc != 2 || std::strcmp(argv[1], "info") != 0) {
    printf("usage: ethernet info\n");
    return 1;
  }

  UpdateEthernetIpInfo();
  eth_speed_t speed = ETH_SPEED_10M;
  eth_duplex_t duplex = ETH_DUPLEX_HALF;
  int speed_mbps = 0;
  const char* duplex_name = "-";
  if (g_eth_link_up &&
      esp_eth_ioctl(g_eth_handle, ETH_CMD_G_SPEED, &speed) == ESP_OK) {
    speed_mbps = speed == ETH_SPEED_100M ? 100 : 10;
  }
  if (g_eth_link_up &&
      esp_eth_ioctl(g_eth_handle, ETH_CMD_G_DUPLEX_MODE, &duplex) == ESP_OK) {
    duplex_name = duplex == ETH_DUPLEX_FULL ? "full" : "half";
  }

  printf("W5500:\n");
  printf("  Link: %s\n", g_eth_link_up ? "up" : "down");
  printf("  Speed: %d Mbps %s duplex\n", speed_mbps, duplex_name);
  printf("  HW ADDR: %s\n", g_eth_mac);
  printf("  ETHIP: %s\n", g_eth_ip);
  printf("  ETHMASK: %s\n", g_eth_netmask);
  printf("  ETHGW: %s\n", g_eth_gateway);
  printf("  DNS: %s\n", g_eth_dns);
  return 0;
}

void RegisterEthernetCommand()
{
  esp_console_cmd_t command = {};
  command.command = "ethernet";
  command.help = "Ethernet interface information";
  command.hint = "<info>";
  command.func = EthernetInfoCommand;
  ESP_ERROR_CHECK(esp_console_cmd_register(&command));
}

}  // namespace

extern "C" void app_main(void)
{
  printf("T-CAN485-C5 W5500 iperf test\n");
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
  repl_config.prompt = "iperf>";
  esp_console_repl_t* repl = nullptr;
#if CONFIG_ESP_CONSOLE_UART
  esp_console_dev_uart_config_t uart_config =
      ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(
      esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
  esp_console_dev_usb_cdc_config_t usb_config =
      ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(
      esp_console_new_repl_usb_cdc(&usb_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  esp_console_dev_usb_serial_jtag_config_t usb_config =
      ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(
      esp_console_new_repl_usb_serial_jtag(&usb_config, &repl_config, &repl));
#else
#error Unsupported console output mode
#endif

  const esp_err_t ethernet_err = InitEthernet();
  if (ethernet_err != ESP_OK) {
    printf("[ethernet] initialization failed: %s (0x%x)\n",
           esp_err_to_name(ethernet_err),
           static_cast<unsigned>(ethernet_err));
  }
  ESP_ERROR_CHECK(iperf_cmd_register_iperf());
  RegisterEthernetCommand();

  printf("\n");
  printf("=======================================================\n");
  printf("|     T-CAN485-C5 W5500 Ethernet Bandwidth Test      |\n");
  printf("|                                                     |\n");
  printf("|  1. Wait for the DHCP address                       |\n");
  printf("|  2. Enter: ethernet info                            |\n");
  printf("|  3. Server: iperf -s -i 3                           |\n");
  printf("|  4. Client: iperf -c SERVER_IP -t 60 -i 3           |\n");
  printf("|  5. Add -u to run a UDP test                        |\n");
  printf("|                                                     |\n");
  printf("=======================================================\n\n");
  if (ethernet_err != ESP_OK) {
    printf("W5500 is not available; fix the SPI/reset hardware before "
           "running iperf.\n\n");
  }

  ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
