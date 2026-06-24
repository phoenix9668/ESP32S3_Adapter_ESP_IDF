#include "serial_router.h"

#include "app_config.h"
#include "app_protocol.h"
#include "board.h"
#include "ch9434.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "radio_service.h"
#include <stdbool.h>
#include <string.h>

typedef struct {
  uint8_t uart_idx;
  size_t length;
  uint8_t data[APP_PAYLOAD_MAX_LEN];
} serial_command_t;

typedef struct {
  size_t length;
  TickType_t last_rx_tick;
  uint8_t data[APP_PAYLOAD_MAX_LEN];
} serial_accumulator_t;

static const char *TAG = "SERIAL";

static QueueHandle_t s_command_queue;
static serial_accumulator_t s_weight_frame;
static serial_accumulator_t s_channel1_response;
static uint32_t s_uart_rx_total[APP_CH9434_UART_COUNT];

static void serial_router_task(void *arg);
static void ch9434_init_uarts(void);
static void process_pending_commands(void);
static void write_serial_command(const serial_command_t *command);
static void wait_for_tx_drain(uint8_t uart_idx, size_t bytes);
static void process_ch9434_interrupts(void);
static void poll_uart_fifos(void);
static void read_uart_fifo(uint8_t uart_idx);
static void handle_rx_chunk(uint8_t uart_idx, const uint8_t *data,
                            size_t length);
static void handle_weight_data(const uint8_t *data, size_t length);
static void drop_stale_weight_frame_if_idle(void);
static void handle_channel1_data(const uint8_t *data, size_t length);
static void flush_channel1_response_if_idle(void);
static void forward_channel1_response(void);

esp_err_t serial_router_init(void) {
  if (s_command_queue != NULL) {
    return ESP_OK;
  }

  s_command_queue =
      xQueueCreate(APP_SERIAL_COMMAND_QUEUE_LEN, sizeof(serial_command_t));
  if (s_command_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

esp_err_t serial_router_start(void) {
  esp_err_t ret = serial_router_init();
  if (ret != ESP_OK) {
    return ret;
  }

  BaseType_t ok =
      xTaskCreate(serial_router_task, "serial_router", APP_TASK_STACK_LARGE,
                  NULL, tskIDLE_PRIORITY + 6, NULL);
  if (ok != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "serial router started");
  return ESP_OK;
}

esp_err_t serial_router_submit_command(uint8_t uart_idx, const uint8_t *data,
                                       size_t length) {
  if (s_command_queue == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (uart_idx >= APP_CH9434_UART_COUNT || data == NULL || length == 0U ||
      length > APP_PAYLOAD_MAX_LEN) {
    return ESP_ERR_INVALID_ARG;
  }

  serial_command_t command = {
      .uart_idx = uart_idx,
      .length = length,
  };
  memcpy(command.data, data, length);

  if (xQueueSend(s_command_queue, &command,
                 pdMS_TO_TICKS(APP_QUEUE_SEND_TIMEOUT_MS)) != pdPASS) {
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

static void serial_router_task(void *arg) {
  (void)arg;

  ch9434_spi2_init();
  ch9434_init_uarts();

  while (true) {
    process_pending_commands();

    if (gpio_get_level(CH9434_INT) == 0) {
      process_ch9434_interrupts();
    }

    poll_uart_fifos();
    drop_stale_weight_frame_if_idle();
    flush_channel1_response_if_idle();
    vTaskDelay(pdMS_TO_TICKS(APP_CH9434_POLL_INTERVAL_MS));
  }
}

static void ch9434_init_uarts(void) {
  CH9434InitClkMode(CH9434_ENABLE, CH9434_ENABLE, 13);
  vTaskDelay(pdMS_TO_TICKS(50));

  CH9434UARTxInit(CH9434_UART_IDX_0, UART_BPS_ID0);
  CH9434UARTxInit(CH9434_UART_IDX_1, UART_BPS);
  CH9434UARTxInit(CH9434_UART_IDX_2, UART_BPS);
  CH9434UARTxInit(CH9434_UART_IDX_3, UART_BPS);

  ESP_LOGI(TAG, "CH9434 initialized");
}

static void process_pending_commands(void) {
  serial_command_t command;
  while (xQueueReceive(s_command_queue, &command, 0) == pdPASS) {
    write_serial_command(&command);
  }
}

static void write_serial_command(const serial_command_t *command) {
  ESP_LOGD(TAG, "write uart%u command len=%u", command->uart_idx,
           (unsigned)command->length);
  ESP_LOG_BUFFER_HEXDUMP(TAG, command->data, command->length, ESP_LOG_DEBUG);

  if (command->uart_idx == CH9434_UART_IDX_1) {
    s_channel1_response.length = 0U;
  }

  board_rs485_set_direction(command->uart_idx, BOARD_RS485_TX);
  vTaskDelay(pdMS_TO_TICKS(APP_RS485_DIR_SETTLE_MS));

  CH9434UARTxSetTxFIFOData(command->uart_idx, command->data,
                           (uint16_t)command->length);
  wait_for_tx_drain(command->uart_idx, command->length);

  board_rs485_set_direction(command->uart_idx, BOARD_RS485_RX);
}

static void wait_for_tx_drain(uint8_t uart_idx, size_t bytes) {
  const TickType_t start = xTaskGetTickCount();
  const TickType_t timeout = pdMS_TO_TICKS(APP_RS485_TX_DONE_TIMEOUT_MS);
  uint16_t tx_fifo_len = 0;
  bool drained = false;

  while ((xTaskGetTickCount() - start) < timeout) {
    tx_fifo_len = CH9434UARTxGetTxFIFOLen(uart_idx);
    if (tx_fifo_len == 0U) {
      drained = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (!drained) {
    ESP_LOGW(TAG, "uart%u tx fifo not empty after %ums, remaining=%u", uart_idx,
             (unsigned)APP_RS485_TX_DONE_TIMEOUT_MS, (unsigned)tx_fifo_len);
  }

  const uint32_t bps = uart_idx == CH9434_UART_IDX_0 ? UART_BPS_ID0 : UART_BPS;
  uint32_t hold_ms = (uint32_t)((bytes * 11U * 1000U + bps - 1U) / bps) + 1U;
  if (hold_ms < APP_RS485_TX_HOLD_MIN_MS) {
    hold_ms = APP_RS485_TX_HOLD_MIN_MS;
  }
  vTaskDelay(pdMS_TO_TICKS(hold_ms));
}

static void process_ch9434_interrupts(void) {
  for (uint8_t pass = 0U; pass < APP_CH9434_IRQ_DRAIN_PASSES; ++pass) {
    if (gpio_get_level(CH9434_INT) != 0) {
      return;
    }

    for (uint8_t uart_idx = 0; uart_idx < APP_CH9434_UART_COUNT; ++uart_idx) {
      const uint8_t iir = CH9434UARTxReadIIR(uart_idx);
      ESP_LOGD(TAG, "uart%u iir=0x%02x", uart_idx, iir);

      switch (iir & 0x0F) {
      case 0x01:
        break;
      case 0x06:
        ESP_LOGD(TAG, "uart%u lsr=0x%02x", uart_idx,
                 CH9434UARTxReadLSR(uart_idx));
        read_uart_fifo(uart_idx);
        break;
      case 0x04:
      case 0x0C:
        read_uart_fifo(uart_idx);
        break;
      case 0x02:
        break;
      case 0x00:
        ESP_LOGD(TAG, "uart%u msr=0x%02x", uart_idx,
                 CH9434UARTxReadMSR(uart_idx));
        break;
      default:
        ESP_LOGD(TAG, "uart%u unhandled iir=0x%02x", uart_idx, iir);
        break;
      }
    }
  }
}

static void poll_uart_fifos(void) {
  for (uint8_t uart_idx = 0; uart_idx < APP_CH9434_UART_COUNT; ++uart_idx) {
    read_uart_fifo(uart_idx);
  }
}

static void read_uart_fifo(uint8_t uart_idx) {
  uint8_t rx_buf[APP_CH9434_RX_BUFFER_LEN];

  while (true) {
    uint16_t pending = CH9434UARTxGetRxFIFOLen(uart_idx);
    if (pending == 0U) {
      return;
    }

    const uint16_t read_len =
        pending > APP_CH9434_RX_BUFFER_LEN ? APP_CH9434_RX_BUFFER_LEN : pending;
    CH9434UARTxGetRxFIFOData(uart_idx, rx_buf, read_len);

    s_uart_rx_total[uart_idx] += read_len;
    ESP_LOGD(TAG, "uart%u rx=%u total=%lu", uart_idx, (unsigned)read_len,
             (unsigned long)s_uart_rx_total[uart_idx]);
    ESP_LOG_BUFFER_HEXDUMP(TAG, rx_buf, read_len, ESP_LOG_DEBUG);

    handle_rx_chunk(uart_idx, rx_buf, read_len);
  }
}

static void handle_rx_chunk(uint8_t uart_idx, const uint8_t *data,
                            size_t length) {
  if (uart_idx == CH9434_UART_IDX_0) {
    handle_weight_data(data, length);
    return;
  }

  if (uart_idx == CH9434_UART_IDX_1) {
    handle_channel1_data(data, length);
  }
}

static void handle_weight_data(const uint8_t *data, size_t length) {
  drop_stale_weight_frame_if_idle();

  for (size_t i = 0U; i < length; ++i) {
    if (s_weight_frame.length >= APP_PAYLOAD_MAX_LEN) {
      const size_t dump_len =
          s_weight_frame.length > 64U ? 64U : s_weight_frame.length;
      ESP_LOGW(TAG, "drop oversized channel0 frame len=%u tail_dump=%u",
               (unsigned)s_weight_frame.length, (unsigned)dump_len);
      ESP_LOG_BUFFER_HEXDUMP(
          TAG, &s_weight_frame.data[s_weight_frame.length - dump_len], dump_len,
          ESP_LOG_WARN);
      s_weight_frame.length = 0U;
    }

    s_weight_frame.data[s_weight_frame.length++] = data[i];
    s_weight_frame.last_rx_tick = xTaskGetTickCount();

    if (!app_protocol_is_weight_frame_complete(s_weight_frame.data,
                                               s_weight_frame.length)) {
      continue;
    }

    esp_err_t ret = radio_service_send_frame(
        APP_FRAME_TYPE_WEIGHT, s_weight_frame.data, s_weight_frame.length);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "failed to enqueue channel0 frame: %s",
               esp_err_to_name(ret));
    } else {
      board_led_pulse(BOARD_LED_GREEN, APP_WEIGHT_LED_PULSE_MS);
    }

    s_weight_frame.length = 0U;
  }
}

static void drop_stale_weight_frame_if_idle(void) {
  if (s_weight_frame.length == 0U) {
    return;
  }

  const TickType_t now = xTaskGetTickCount();
  if (now - s_weight_frame.last_rx_tick <
      pdMS_TO_TICKS(APP_WEIGHT_FRAME_STALE_MS)) {
    return;
  }

  ESP_LOGW(TAG, "discard stale channel0 frame len=%u",
           (unsigned)s_weight_frame.length);
  s_weight_frame.length = 0U;
}

static void handle_channel1_data(const uint8_t *data, size_t length) {
  const TickType_t now = xTaskGetTickCount();

  if (s_channel1_response.length > 0U &&
      now - s_channel1_response.last_rx_tick >=
          pdMS_TO_TICKS(APP_SERIAL_RESPONSE_TIMEOUT_MS)) {
    ESP_LOGW(TAG, "discard stale channel1 response len=%u",
             (unsigned)s_channel1_response.length);
    s_channel1_response.length = 0U;
  }

  for (size_t i = 0U; i < length; ++i) {
    if (s_channel1_response.length >= APP_PAYLOAD_MAX_LEN) {
      forward_channel1_response();
    }

    s_channel1_response.data[s_channel1_response.length++] = data[i];
    s_channel1_response.last_rx_tick = now;
  }
}

static void flush_channel1_response_if_idle(void) {
  if (s_channel1_response.length == 0U) {
    return;
  }

  const TickType_t now = xTaskGetTickCount();
  if (now - s_channel1_response.last_rx_tick <
      pdMS_TO_TICKS(APP_SERIAL_RESPONSE_IDLE_MS)) {
    return;
  }

  forward_channel1_response();
}

static void forward_channel1_response(void) {
  if (s_channel1_response.length == 0U) {
    return;
  }

  esp_err_t ret = radio_service_send_frame(APP_FRAME_TYPE_CHANNEL1,
                                           s_channel1_response.data,
                                           s_channel1_response.length);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "failed to enqueue channel1 response: %s",
             esp_err_to_name(ret));
  } else {
    board_led_pulse(BOARD_LED_BLUE, APP_CHANNEL1_LED_PULSE_MS);
  }

  s_channel1_response.length = 0U;
}
