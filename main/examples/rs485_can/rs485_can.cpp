/*
 * @Description: CAN and RS485 transmit/receive test
 * @Author: LILYGO_L
 * @Date: 2026-06-12 12:00:00
 * @LastEditTime: 2026-06-12 16:15:32
 * @License: GPL 3.0
 */

#include <cstdint>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lilygo_device_driver_library.h"

namespace {

namespace t_can485_c5 = lilygo_device_driver::t_can485_c5;

constexpr uart_port_t kRs485UartPort = UART_NUM_1;
constexpr int kRs485BaudRate = 115200;
constexpr int kRs485RxBufferSize = 4096;
constexpr int kRs485TxBufferSize = 4096;
constexpr int kRs485MaxTransmitSize = 1024;
constexpr int kRs485TxDoneWaitMs = 20;
constexpr char kRs485TestChar = 'R';

constexpr int kCanDataLength = 8;
constexpr int kCanTxWaitMs = 0;
constexpr int kCanTxIntervalMs = 20;
constexpr int kCanBusOffDelayMs = 500;
constexpr int kCanRecoverRetryIntervalMs = 1000;
constexpr int kCanStatusPrintIntervalMs = 5000;
constexpr int kCanPollMs = 100;
constexpr int kCanMaxReceiveFramesPerLoop = 32;
constexpr uint32_t kCanTestId = 0x0F1;
constexpr char kCanTestChar = 'C';
constexpr int kCanBitrate = 1000000;
constexpr int kCanTxQueueDepth = 16;
constexpr int kCanRxQueueDepth = 32;

constexpr int kPrintIntervalUs = 3 * 1000 * 1000;
constexpr int kTaskStackSize = 6 * 1024;
constexpr UBaseType_t kTaskPriority = 5;

TaskHandle_t g_rs485_task_handle = nullptr;
TaskHandle_t g_can_task_handle = nullptr;
volatile bool g_rs485_exit_test = false;
volatile bool g_can_exit_test = false;
std::string g_console_buffer;
twai_node_handle_t g_can_node = nullptr;
QueueHandle_t g_can_rx_queue = nullptr;
volatile uint32_t g_can_error_flags = 0;
volatile twai_error_state_t g_can_state = TWAI_ERROR_ACTIVE;
volatile bool g_can_recovering = false;
int64_t g_can_last_status_print_us = 0;
int64_t g_can_last_recover_us = 0;

struct CanRxFrame {
  uint32_t id;
  uint8_t data[kCanDataLength];
  uint8_t data_length;
  bool is_remote;
  bool is_extended;
};

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

bool ValidateData(const char* tag, const uint8_t* data, size_t len,
                  uint8_t expected_char)
{
  for (size_t i = 0; i < len; ++i) {
    if (data[i] != expected_char) {
      printf("\n[%s] data corruption error\n", tag);
      printf("[%s] offset: %zu, expected: 0x%02X ('%c'), got: 0x%02X\n",
             tag, i, expected_char, expected_char, data[i]);
      printf("[%s] data snippet:", tag);
      const size_t start = (i > 5) ? i - 5 : 0;
      const size_t end = (i + 5 > len) ? len : i + 5;
      for (size_t j = start; j < end; ++j) {
        printf(" 0x%02X", data[j]);
      }
      printf("\n");
      g_rs485_exit_test = true;
      g_can_exit_test = true;
      return false;
    }
  }

  return true;
}

void PrintCanFrame(const CanRxFrame& frame)
{
  printf("[can receive] raw id=0x%lX dlc=%u ext=%d rtr=%d data:",
         static_cast<unsigned long>(frame.id), frame.data_length,
         frame.is_extended ? 1 : 0, frame.is_remote ? 1 : 0);
  for (uint8_t i = 0; i < frame.data_length; ++i) {
    printf(" 0x%02X", frame.data[i]);
  }
  printf("\n");
}

void PrintProgress(const char* tag, size_t bytes_this_time, size_t total_size,
                   int64_t now, int64_t last_print_time)
{
  const float elapsed_s =
      static_cast<float>(now - last_print_time) / 1000000.0f;
  const float speed_kbps =
      (elapsed_s > 0.0f)
          ? static_cast<float>(bytes_this_time) / 1024.0f / elapsed_s
          : 0.0f;
  printf("[%s] size: %.2f KB | speed: %.2f KB/s | total size: %.2f KB\n",
         tag, static_cast<float>(bytes_this_time) / 1024.0f, speed_kbps,
         static_cast<float>(total_size) / 1024.0f);
}

void PrintResult(const char* tag, size_t total_size, int64_t start_time)
{
  const float total_time_s =
      static_cast<float>(esp_timer_get_time() - start_time) / 1000000.0f;
  const float avg_speed =
      (total_time_s > 0.0f)
          ? static_cast<float>(total_size) / 1024.0f / total_time_s
          : 0.0f;
  printf("[%s] === result ===\n", tag);
  printf("[%s] total size: %.2f KB\n", tag,
         static_cast<float>(total_size) / 1024.0f);
  printf("[%s] total time %.3f s\n", tag, total_time_s);
  printf("[%s] avg speed: %.2f KB/s\n", tag, avg_speed);
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
    printf("Install RS485 UART driver failed: %s\n", esp_err_to_name(err));
    return false;
  }
  err = uart_param_config(kRs485UartPort, &uart_config);
  if (err != ESP_OK) {
    printf("Config RS485 UART failed: %s\n", esp_err_to_name(err));
    uart_driver_delete(kRs485UartPort);
    return false;
  }
  err = uart_set_pin(kRs485UartPort, t_can485_c5::gpio::rs485::kTx,
                     t_can485_c5::gpio::rs485::kRx, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    printf("Set RS485 UART pins failed: %s\n", esp_err_to_name(err));
    uart_driver_delete(kRs485UartPort);
    return false;
  }

  return true;
}

void Rs485Task(void* param)
{
  const bool is_send = (param != nullptr);
  printf("rs485_test_task (%s) start\n", is_send ? "send" : "receive");

  if (!InitRs485()) {
    g_rs485_task_handle = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  size_t total_size = 0;
  size_t bytes_this_time = 0;
  const int64_t start_time = esp_timer_get_time();
  int64_t last_print_time = start_time;

  if (is_send) {
    std::string payload(kRs485MaxTransmitSize, kRs485TestChar);
    while (!g_rs485_exit_test) {
      const int len =
          uart_write_bytes(kRs485UartPort, payload.data(), payload.size());
      if (len > 0) {
        uart_wait_tx_done(kRs485UartPort, pdMS_TO_TICKS(kRs485TxDoneWaitMs));
        bytes_this_time += len;
        total_size += len;
      }

      const int64_t now = esp_timer_get_time();
      if (now - last_print_time >= kPrintIntervalUs) {
        PrintProgress("rs485 send", bytes_this_time, total_size, now,
                      last_print_time);
        bytes_this_time = 0;
        last_print_time = now;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    PrintResult("rs485 send", total_size, start_time);
  } else {
    while (!g_rs485_exit_test) {
      size_t rx_len = 0;
      uart_get_buffered_data_len(kRs485UartPort, &rx_len);
      if (rx_len > 0) {
        std::unique_ptr<uint8_t[]> buffer(new uint8_t[rx_len]);
        const int read_len = uart_read_bytes(kRs485UartPort, buffer.get(),
                                             rx_len, pdMS_TO_TICKS(20));
        if (read_len > 0 &&
            ValidateData("rs485 receive", buffer.get(), read_len,
                         kRs485TestChar)) {
          bytes_this_time += read_len;
          total_size += read_len;
        }
      }

      const int64_t now = esp_timer_get_time();
      if (now - last_print_time >= kPrintIntervalUs) {
        PrintProgress("rs485 receive", bytes_this_time, total_size, now,
                      last_print_time);
        bytes_this_time = 0;
        last_print_time = now;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    PrintResult("rs485 receive", total_size, start_time);
  }

  uart_driver_delete(kRs485UartPort);
  printf("[rs485] task completed\n");
  g_rs485_task_handle = nullptr;
  vTaskDelete(nullptr);
}

bool InitCan()
{
  g_can_error_flags = 0;
  g_can_state = TWAI_ERROR_ACTIVE;
  g_can_recovering = false;
  g_can_last_status_print_us = 0;
  g_can_last_recover_us = 0;

  g_can_rx_queue = xQueueCreate(kCanRxQueueDepth, sizeof(CanRxFrame));
  if (g_can_rx_queue == nullptr) {
    printf("Create CAN RX queue failed\n");
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
    printf("Create TWAI node failed: %s\n", esp_err_to_name(err));
    vQueueDelete(g_can_rx_queue);
    g_can_rx_queue = nullptr;
    return false;
  }

  twai_mask_filter_config_t filter_config = {};
  filter_config.id = 0;
  filter_config.mask = 0;
  err = twai_node_config_mask_filter(g_can_node, 0, &filter_config);
  if (err != ESP_OK) {
    printf("Configure TWAI accept-all filter failed: %s\n",
           esp_err_to_name(err));
    twai_node_delete(g_can_node);
    g_can_node = nullptr;
    vQueueDelete(g_can_rx_queue);
    g_can_rx_queue = nullptr;
    return false;
  }

  twai_event_callbacks_t callbacks = {};
  callbacks.on_rx_done = OnCanRxDone;
  callbacks.on_error = OnCanError;
  callbacks.on_state_change = OnCanStateChange;
  err = twai_node_register_event_callbacks(g_can_node, &callbacks, nullptr);
  if (err != ESP_OK) {
    printf("Register TWAI callbacks failed: %s\n", esp_err_to_name(err));
    twai_node_delete(g_can_node);
    g_can_node = nullptr;
    vQueueDelete(g_can_rx_queue);
    g_can_rx_queue = nullptr;
    return false;
  }

  err = twai_node_enable(g_can_node);
  if (err != ESP_OK) {
    printf("Enable TWAI node failed: %s\n", esp_err_to_name(err));
    twai_node_delete(g_can_node);
    g_can_node = nullptr;
    vQueueDelete(g_can_rx_queue);
    g_can_rx_queue = nullptr;
    return false;
  }

  return true;
}

void PrintCanStatus(bool force_print = false)
{
  if (g_can_node == nullptr) {
    return;
  }

  const int64_t now = esp_timer_get_time();
  twai_node_status_t status = {};
  twai_node_record_t record = {};
  const esp_err_t err = twai_node_get_info(g_can_node, &status, &record);
  if (err != ESP_OK) {
    return;
  }

  const bool can_retry_recover =
      now - g_can_last_recover_us >= kCanRecoverRetryIntervalMs * 1000;
  if (status.state == TWAI_ERROR_BUS_OFF && !g_can_recovering &&
      can_retry_recover) {
    const esp_err_t recover_err = twai_node_recover(g_can_node);
    g_can_last_recover_us = now;
    if (recover_err == ESP_OK) {
      g_can_recovering = true;
      printf("[can] bus off, start recovery\n");
    }
  }

  if (!force_print &&
      now - g_can_last_status_print_us <
          kCanStatusPrintIntervalMs * 1000) {
    return;
  }

  const uint32_t error_flags = g_can_error_flags;
  g_can_error_flags = 0;
  const bool need_print = force_print || error_flags != 0 ||
                          status.state != TWAI_ERROR_ACTIVE ||
                          g_can_state != status.state;
  if (need_print) {
    printf("[can] state=%s tx_err=%u rx_err=%u tx_queue=%lu bus_err=%lu "
           "flags=0x%08lX\n",
           CanStateName(status.state), status.tx_error_count,
           status.rx_error_count,
           static_cast<unsigned long>(status.tx_queue_remaining),
           static_cast<unsigned long>(record.bus_err_num),
           static_cast<unsigned long>(error_flags));
    g_can_last_status_print_us = now;
  }
}

void DeinitCan()
{
  if (g_can_node != nullptr) {
    twai_node_transmit_wait_all_done(g_can_node, 100);
    esp_err_t err = twai_node_disable(g_can_node);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      printf("Disable TWAI node failed: %s\n", esp_err_to_name(err));
    }
    err = twai_node_delete(g_can_node);
    if (err != ESP_OK) {
      printf("Delete TWAI node failed: %s\n", esp_err_to_name(err));
    }
    g_can_node = nullptr;
  }
  if (g_can_rx_queue != nullptr) {
    vQueueDelete(g_can_rx_queue);
    g_can_rx_queue = nullptr;
  }
}

void CanTask(void* param)
{
  const bool is_send = (param != nullptr);
  printf("can_test_task (%s) start\n", is_send ? "send" : "receive");

  if (!InitCan()) {
    g_can_task_handle = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  size_t total_size = 0;
  size_t bytes_this_time = 0;
  const int64_t start_time = esp_timer_get_time();
  int64_t last_print_time = start_time;

  if (is_send) {
    uint8_t data[kCanDataLength] = {};
    for (int i = 0; i < kCanDataLength; ++i) {
      data[i] = kCanTestChar;
    }
    twai_frame_t frame = {};
    frame.header.id = kCanTestId;
    frame.header.dlc = kCanDataLength;
    frame.buffer = data;
    frame.buffer_len = sizeof(data);

    while (!g_can_exit_test) {
      PrintCanStatus();
      twai_node_status_t status = {};
      twai_node_record_t record = {};
      if (twai_node_get_info(g_can_node, &status, &record) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(kCanTxIntervalMs));
        continue;
      }
      if (status.state == TWAI_ERROR_BUS_OFF) {
        vTaskDelay(pdMS_TO_TICKS(kCanBusOffDelayMs));
        continue;
      }
      if (status.state != TWAI_ERROR_ACTIVE) {
        vTaskDelay(pdMS_TO_TICKS(kCanTxIntervalMs));
      }

      if (status.tx_queue_remaining > 0) {
        const esp_err_t err =
            twai_node_transmit(g_can_node, &frame, kCanTxWaitMs);
        if (err == ESP_OK) {
          bytes_this_time += kCanDataLength;
          total_size += kCanDataLength;
        }
      }

      const int64_t now = esp_timer_get_time();
      if (now - last_print_time >= kPrintIntervalUs) {
        PrintProgress("can send", bytes_this_time, total_size, now,
                      last_print_time);
        bytes_this_time = 0;
        last_print_time = now;
      }
      vTaskDelay(pdMS_TO_TICKS(kCanTxIntervalMs));
    }
    PrintResult("can send", total_size, start_time);
  } else {
    while (!g_can_exit_test) {
      PrintCanStatus();

      CanRxFrame rx_frame = {};
      int receive_count = 0;
      while (receive_count < kCanMaxReceiveFramesPerLoop &&
             xQueueReceive(g_can_rx_queue, &rx_frame,
                           pdMS_TO_TICKS(kCanPollMs)) == pdTRUE) {
        ++receive_count;
        if (!rx_frame.is_remote && !rx_frame.is_extended &&
            rx_frame.id == kCanTestId &&
            rx_frame.data_length == kCanDataLength &&
            ValidateData("can receive", rx_frame.data,
                         rx_frame.data_length, kCanTestChar)) {
          bytes_this_time += rx_frame.data_length;
          total_size += rx_frame.data_length;
        } else {
          PrintCanFrame(rx_frame);
        }
      }

      const int64_t now = esp_timer_get_time();
      if (now - last_print_time >= kPrintIntervalUs) {
        PrintProgress("can receive", bytes_this_time, total_size, now,
                      last_print_time);
        bytes_this_time = 0;
        last_print_time = now;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    PrintResult("can receive", total_size, start_time);
  }

  DeinitCan();
  printf("[can] task completed\n");
  g_can_task_handle = nullptr;
  vTaskDelete(nullptr);
}

std::string TrimCommandToken(std::string token)
{
  while (!token.empty() &&
         (token.back() == '\r' || token.back() == '\n' || token.back() == ' ')) {
    token.pop_back();
  }
  while (!token.empty() && token.front() == ' ') {
    token.erase(token.begin());
  }
  return token;
}

void StopRs485Task()
{
  if (g_rs485_task_handle == nullptr) {
    g_rs485_exit_test = false;
    return;
  }

  g_rs485_exit_test = true;
  for (int i = 0; i < 10; ++i) {
    if (g_rs485_task_handle == nullptr) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (g_rs485_task_handle != nullptr) {
    printf("Stop RS485 task timeout\n");
  }
  g_rs485_exit_test = false;
}

void StopCanTask()
{
  if (g_can_task_handle == nullptr) {
    g_can_exit_test = false;
    return;
  }

  g_can_exit_test = true;
  for (int i = 0; i < 10; ++i) {
    if (g_can_task_handle == nullptr) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (g_can_task_handle != nullptr) {
    printf("Stop CAN task timeout\n");
  }
  g_can_exit_test = false;
}

void StopAllTasks()
{
  StopRs485Task();
  StopCanTask();
}

bool StartRs485Task(bool is_send)
{
  printf("Start RS485 %s command\n", is_send ? "send" : "receive");
  StopRs485Task();
  if (g_rs485_task_handle != nullptr) {
    printf("Start RS485 failed: previous RS485 task is still running\n");
    return false;
  }

  g_rs485_exit_test = false;
  if (xTaskCreate(Rs485Task, "rs485_task", kTaskStackSize,
                  is_send ? reinterpret_cast<void*>(1) : nullptr,
                  kTaskPriority, &g_rs485_task_handle) != pdPASS) {
    g_rs485_task_handle = nullptr;
    printf("Create RS485 task failed\n");
    return false;
  }
  return true;
}

bool StartCanTask(bool is_send)
{
  printf("Start CAN %s command\n", is_send ? "send" : "receive");
  StopCanTask();
  if (g_can_task_handle != nullptr) {
    printf("Start CAN failed: previous CAN task is still running\n");
    return false;
  }

  g_can_exit_test = false;
  if (xTaskCreate(CanTask, "can_task", kTaskStackSize,
                  is_send ? reinterpret_cast<void*>(1) : nullptr,
                  kTaskPriority, &g_can_task_handle) != pdPASS) {
    g_can_task_handle = nullptr;
    printf("Create CAN task failed\n");
    return false;
  }
  return true;
}

void PrintCommandHelp()
{
  printf("operation command:\n");
  printf("[rs485:send:]\n");
  printf("[rs485:receive:]\n");
  printf("[can:send:]\n");
  printf("[can:receive:]\n");
  printf("[all_exit:]\n");
}

bool ParseCommand(const std::vector<std::string>& cmd)
{
  if (cmd.empty()) {
    printf("parse_cmd fail (cmd is empty)\n");
    return false;
  }
  if (cmd[0] == "all_exit") {
    StopAllTasks();
    printf("all exit test\n");
    return true;
  }
  if (cmd.size() <= 1) {
    printf("parse_cmd fail (missing mode)\n");
    PrintCommandHelp();
    return false;
  }

  if (cmd[0] == "rs485") {
    if (cmd[1] == "send") {
      return StartRs485Task(true);
    }
    if (cmd[1] == "receive") {
      return StartRs485Task(false);
    }
  } else if (cmd[0] == "can") {
    if (cmd[1] == "send") {
      return StartCanTask(true);
    }
    if (cmd[1] == "receive") {
      return StartCanTask(false);
    }
  }

  printf("parse_cmd fail (unknown command)\n");
  PrintCommandHelp();
  return false;
}

void ParseCommandLine(const std::string& content)
{
  std::stringstream stream(content);
  std::string token;
  std::vector<std::string> parts;
  while (std::getline(stream, token, ':')) {
    token = TrimCommandToken(token);
    if (!token.empty()) {
      parts.push_back(token);
    }
  }
  ParseCommand(parts);
}

bool IsCompleteCommand(const std::string& content)
{
  const std::string command = TrimCommandToken(content);
  return command == "all_exit:" || command == "rs485:send:" ||
         command == "rs485:receive:" || command == "can:send:" ||
         command == "can:receive:";
}

void AppendConsoleInput(const std::string& content)
{
  g_console_buffer += content;

  size_t line_end = std::string::npos;
  while ((line_end = g_console_buffer.find_first_of("\r\n")) !=
         std::string::npos) {
    const std::string line = g_console_buffer.substr(0, line_end);
    g_console_buffer.erase(0, line_end + 1);
    if (!TrimCommandToken(line).empty()) {
      ParseCommandLine(line);
    }
  }

  if (IsCompleteCommand(g_console_buffer)) {
    ParseCommandLine(g_console_buffer);
    g_console_buffer.clear();
  }
}

}  // namespace

extern "C" void app_main(void)
{
  printf("Ciallo\n");

  usb_serial_jtag_driver_config_t usb_serial_jtag_config =
      USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_serial_jtag_config.rx_buffer_size = 1024 * 2;
  usb_serial_jtag_config.tx_buffer_size = 1024 * 2;
  esp_err_t err = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    printf("Install USB Serial/JTAG driver failed: %s\n",
           esp_err_to_name(err));
    return;
  }

  PrintCommandHelp();

  while (true) {
    uint8_t buffer[128] = {};
    const int read_len = usb_serial_jtag_read_bytes(buffer, sizeof(buffer),
                                                    pdMS_TO_TICKS(20));
    if (read_len > 0) {
      AppendConsoleInput(
          std::string(reinterpret_cast<char*>(buffer), read_len));
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
