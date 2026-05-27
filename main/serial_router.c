#include "serial_router.h"

#include "app_config.h"
#include "app_protocol.h"
#include "board.h"
#include "cellular_4g.h"
#include "ch9434.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "radio_service.h"
#include <string.h>

typedef struct {
  uint8_t uart_idx;
  size_t length;
  uint8_t data[APP_PAYLOAD_MAX_LEN];
} serial_command_t;

typedef struct {
  size_t length;
  uint8_t data[APP_PAYLOAD_MAX_LEN];
} frame_accumulator_t;

typedef enum {
  SERIAL_RESPONSE_ROUTE_RADIO = 0,
  SERIAL_RESPONSE_ROUTE_CELLULAR_4G,
} serial_response_route_t;

static const char *TAG = "SERIAL";

static const uint8_t s_rfid_poll_command[] = {0x04, 0xFF, 0x01, 0x1B, 0xB4};

static QueueHandle_t s_command_queue;
static frame_accumulator_t s_weight_frame;
static uint32_t s_uart_rx_total[APP_CH9434_UART_COUNT];
static serial_response_route_t s_response_route[APP_CH9434_UART_COUNT];
static TickType_t s_next_rfid_poll_tick;

static void serial_router_task(void *arg);
static void ch9434_init_uarts(void);
static bool process_pending_commands(void);
static void poll_rfid_if_due(void);
static void write_serial_command(const serial_command_t *command,
                                 serial_response_route_t response_route);
static void wait_for_tx_drain(uint8_t uart_idx, size_t bytes);
static void process_ch9434_interrupts(void);
static void read_uart_fifo(uint8_t uart_idx);
static void handle_rx_chunk(uint8_t uart_idx, const uint8_t *data,
                            size_t length);
static void handle_weight_data(const uint8_t *data, size_t length);

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

  if (uart_idx >= APP_CH9434_UART_COUNT || data == NULL || length == 0 ||
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
  ch9434_spi2_init();
  ch9434_init_uarts();
  s_next_rfid_poll_tick = xTaskGetTickCount();

  while (true) {
    if (process_pending_commands()) {
      s_next_rfid_poll_tick =
          xTaskGetTickCount() + pdMS_TO_TICKS(APP_RFID_POLL_INTERVAL_MS);
    } else {
      poll_rfid_if_due();
    }

    if (gpio_get_level(CH9434_INT) == 0) {
      process_ch9434_interrupts();
    }

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

static bool process_pending_commands(void) {
  serial_command_t command;
  bool processed = false;
  while (xQueueReceive(s_command_queue, &command, 0) == pdPASS) {
    write_serial_command(&command, SERIAL_RESPONSE_ROUTE_RADIO);
    processed = true;
  }

  return processed;
}

static void poll_rfid_if_due(void) {
  const TickType_t now = xTaskGetTickCount();
  if ((int32_t)(now - s_next_rfid_poll_tick) < 0) {
    return;
  }

  serial_command_t poll_command = {
      .uart_idx = CH9434_UART_IDX_1,
      .length = sizeof(s_rfid_poll_command),
  };
  memcpy(poll_command.data, s_rfid_poll_command, sizeof(s_rfid_poll_command));
  write_serial_command(&poll_command, SERIAL_RESPONSE_ROUTE_CELLULAR_4G);
  s_next_rfid_poll_tick = now + pdMS_TO_TICKS(APP_RFID_POLL_INTERVAL_MS);
}

static void write_serial_command(const serial_command_t *command,
                                 serial_response_route_t response_route) {
  ESP_LOGD(TAG, "write uart%u command len=%u", command->uart_idx,
           (unsigned)command->length);
  ESP_LOG_BUFFER_HEXDUMP(TAG, command->data, command->length, ESP_LOG_DEBUG);

  board_rs485_set_direction(command->uart_idx, BOARD_RS485_TX);
  vTaskDelay(pdMS_TO_TICKS(APP_RS485_DIR_SETTLE_MS));

  CH9434UARTxSetTxFIFOData(command->uart_idx, command->data,
                           (uint16_t)command->length);
  s_response_route[command->uart_idx] = response_route;
  wait_for_tx_drain(command->uart_idx, command->length);

  board_rs485_set_direction(command->uart_idx, BOARD_RS485_RX);
}

static void wait_for_tx_drain(uint8_t uart_idx, size_t bytes) {
  const TickType_t start = xTaskGetTickCount();
  const TickType_t timeout = pdMS_TO_TICKS(APP_RS485_TX_DONE_TIMEOUT_MS);

  while ((xTaskGetTickCount() - start) < timeout) {
    if (CH9434UARTxGetTxFIFOLen(uart_idx) == 0) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  const uint32_t bps = uart_idx == CH9434_UART_IDX_0 ? UART_BPS_ID0 : UART_BPS;
  uint32_t hold_ms = (uint32_t)((bytes * 11U * 1000U + bps - 1U) / bps) + 1U;
  if (hold_ms < APP_RS485_TX_HOLD_MIN_MS) {
    hold_ms = APP_RS485_TX_HOLD_MIN_MS;
  }
  vTaskDelay(pdMS_TO_TICKS(hold_ms));
}

static void process_ch9434_interrupts(void) {
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

static void read_uart_fifo(uint8_t uart_idx) {
  uint8_t rx_buf[APP_CH9434_RX_BUFFER_LEN];

  while (true) {
    uint16_t pending = CH9434UARTxGetRxFIFOLen(uart_idx);
    if (pending == 0) {
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
    board_led_set(BOARD_LED_GREEN, true);
    handle_weight_data(data, length);
    return;
  }

  if (uart_idx == CH9434_UART_IDX_1) {
    if (s_response_route[uart_idx] == SERIAL_RESPONSE_ROUTE_CELLULAR_4G) {
      esp_err_t ret = cellular_4g_send(data, length);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to send RFID response to 4G: %s",
                 esp_err_to_name(ret));
      }
      return;
    }

    esp_err_t ret =
        radio_service_send_frame(APP_FRAME_TYPE_RFID, data, length);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "failed to enqueue RFID response: %s",
               esp_err_to_name(ret));
    }
  }
}

static void handle_weight_data(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    if (s_weight_frame.length >= APP_PAYLOAD_MAX_LEN) {
      ESP_LOGW(TAG, "drop oversized weight frame");
      s_weight_frame.length = 0;
    }

    s_weight_frame.data[s_weight_frame.length++] = data[i];

    if (!app_protocol_is_weight_frame_complete(s_weight_frame.data,
                                               s_weight_frame.length)) {
      continue;
    }

    esp_err_t ret = radio_service_send_frame(APP_FRAME_TYPE_WEIGHT,
                                             s_weight_frame.data,
                                             s_weight_frame.length);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "failed to enqueue weight frame: %s",
               esp_err_to_name(ret));
    }
    s_weight_frame.length = 0;
  }
}
