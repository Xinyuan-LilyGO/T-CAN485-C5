/*
 * @Description: SD card monitor example
 * @Author: LILYGO_L
 * @Date: 2026-06-12 12:00:00
 * @LastEditTime: 2026-06-12 12:00:00
 * @License: GPL 3.0
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <string>
#include <sys/stat.h>

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lilygo_device_driver_library.h"
#include "sd_protocol_defs.h"
#include "sdmmc_cmd.h"

namespace {

namespace t_can485_c5 = lilygo_device_driver::t_can485_c5;

constexpr char kMountPoint[] = "/sdcard";
constexpr int kSdPollPeriodMs = 1000;
constexpr int kSdSpiMaxFreqKhz = 10000;
constexpr int kMaxTreeDepth = 16;

bool g_spi_bus_ready = false;
bool g_sd_mounted = false;
sdmmc_card_t* g_sd_card = nullptr;

bool InitSdSpiBus()
{
  if (g_spi_bus_ready) {
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
    printf("Init SD SPI bus failed: %s\n", esp_err_to_name(err));
    return false;
  }

  g_spi_bus_ready = true;
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

void PrintDirectoryTree(const std::string& path, int depth)
{
  if (depth > kMaxTreeDepth) {
    printf("%*s...\n", depth * 2, "");
    return;
  }

  DIR* dir = opendir(path.c_str());
  if (dir == nullptr) {
    printf("%*s[open failed] %s\n", depth * 2, "", path.c_str());
    return;
  }

  while (true) {
    dirent* entry = readdir(dir);
    if (entry == nullptr) {
      break;
    }
    if (std::strcmp(entry->d_name, ".") == 0 ||
        std::strcmp(entry->d_name, "..") == 0) {
      continue;
    }

    std::string child_path = path;
    if (child_path.back() != '/') {
      child_path += "/";
    }
    child_path += entry->d_name;

    struct stat stat_buffer = {};
    const bool stat_ok = stat(child_path.c_str(), &stat_buffer) == 0;
    const bool is_dir = stat_ok && S_ISDIR(stat_buffer.st_mode);
    printf("%*s%s %s", depth * 2, "", is_dir ? "[D]" : "[F]",
           entry->d_name);
    if (!is_dir && stat_ok) {
      printf(" (%ld B)", static_cast<long>(stat_buffer.st_size));
    }
    printf("\n");

    if (is_dir) {
      PrintDirectoryTree(child_path, depth + 1);
    }
  }

  closedir(dir);
}

void PrintCardInfo()
{
  const uint64_t card_size_mb =
      static_cast<uint64_t>(g_sd_card->csd.capacity) *
      g_sd_card->csd.sector_size / (1024ULL * 1024ULL);
  printf("SD card inserted\n");
  printf("Type: %s\n", CardTypeName(g_sd_card));
  printf("Size: %llu MB\n", static_cast<unsigned long long>(card_size_mb));
  printf("Mount: %s\n", kMountPoint);
  sdmmc_card_print_info(stdout, g_sd_card);
  printf("Files:\n");
  PrintDirectoryTree(kMountPoint, 0);
  printf("End of file list\n\n");
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
  mount_config.max_files = 8;
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
  PrintCardInfo();
  return true;
}

void UnmountSdCard(const char* reason)
{
  if (g_sd_mounted && g_sd_card != nullptr) {
    esp_vfs_fat_sdcard_unmount(kMountPoint, g_sd_card);
  }
  g_sd_card = nullptr;
  g_sd_mounted = false;
  printf("SD card removed or unavailable: %s\n\n", reason);
}

bool MountedCardStillReadable()
{
  if (!g_sd_mounted || g_sd_card == nullptr) {
    return false;
  }
  if (sdmmc_get_status(g_sd_card) != ESP_OK) {
    return false;
  }

  DIR* dir = opendir(kMountPoint);
  if (dir == nullptr) {
    return false;
  }
  closedir(dir);
  return true;
}

}  // namespace

extern "C" void app_main(void)
{
  printf("Ciallo\n");
  printf("SD card monitor start. Insert a card to print card info and files.\n");

  while (true) {
    if (!g_sd_mounted) {
      if (!MountSdCard()) {
        printf("SD card not detected\n");
      }
    } else if (!MountedCardStillReadable()) {
      UnmountSdCard("card removed or read failed");
    }
    vTaskDelay(pdMS_TO_TICKS(kSdPollPeriodMs));
  }
}
