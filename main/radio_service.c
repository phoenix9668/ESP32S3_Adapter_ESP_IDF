#include "radio_service.h"

#include "app_config.h"
#include "ch9434.h"
#include "driver/uart.h"
#include "e34_2g4d20d.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "serial_router.h"
#include <stdbool.h>
#include <string.h>

typedef struct {
  app_frame_type_t type;
  size_t length;
  uint8_t payload[APP_PAYLOAD_MAX_LEN];
} radio_tx_item_t;

typedef struct {
  uint8_t data[APP_PACKET_MAX_LEN];
  size_t length;
} radio_rx_stream_t;

static const char *TAG = "RADIO";

static QueueHandle_t s_tx_queue;
static SemaphoreHandle_t s_uart_mutex;
static uint8_t s_board_address;
static volatile bool s_rx_enabled;

static void radio_tx_task(void *arg);
static void radio_rx_task(void *arg);
static void radio_reset_task(void *arg);
static void radio_rx_append(radio_rx_stream_t *stream, const uint8_t *data,
                            size_t length);
static void radio_rx_process_stream(radio_rx_stream_t *stream);
static void radio_rx_handle_packet(const app_packet_view_t *packet);
static void radio_rx_drop_prefix(radio_rx_stream_t *stream, size_t count);
static size_t radio_rx_find_header(const radio_rx_stream_t *stream);

esp_err_t radio_service_start(uint8_t board_address) {
  if (s_tx_queue != NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  s_board_address = board_address;
  s_uart_mutex = xSemaphoreCreateMutex();
  if (s_uart_mutex == NULL) {
    return ESP_ERR_NO_MEM;
  }

  s_tx_queue = xQueueCreate(APP_RADIO_TX_QUEUE_LEN, sizeof(radio_tx_item_t));
  if (s_tx_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t ret = e34_2g4d20d_uart1_init();
  if (ret != ESP_OK) {
    return ret;
  }

  ret = e34_2g4d20d_gpio_init();
  if (ret != ESP_OK) {
    return ret;
  }

  if (xSemaphoreTake(s_uart_mutex, portMAX_DELAY) == pdTRUE) {
    ret = e34_2g4d20d_parameter_set(0xC0, 0x00, 0x00, 0x18, 0x00, 0x40);
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_flush_input(UART_NUM_1));
    xSemaphoreGive(s_uart_mutex);
    if (ret != ESP_OK) {
      return ret;
    }
  }

  s_rx_enabled = true;

  BaseType_t ok = xTaskCreate(radio_rx_task, "radio_rx", APP_TASK_STACK_LARGE,
                              NULL, tskIDLE_PRIORITY + 8, NULL);
  if (ok != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  ok = xTaskCreate(radio_tx_task, "radio_tx", APP_TASK_STACK_DEFAULT, NULL,
                   tskIDLE_PRIORITY + 7, NULL);
  if (ok != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  ok = xTaskCreate(radio_reset_task, "radio_reset", APP_TASK_STACK_DEFAULT,
                   NULL, tskIDLE_PRIORITY + 4, NULL);
  if (ok != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "radio service started");
  return ESP_OK;
}

esp_err_t radio_service_send_frame(app_frame_type_t type,
                                   const uint8_t *payload, size_t payload_len) {
  if (s_tx_queue == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (payload_len > APP_PAYLOAD_MAX_LEN ||
      (payload_len > 0U && payload == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }

  radio_tx_item_t item = {
      .type = type,
      .length = payload_len,
  };

  if (payload_len > 0U) {
    memcpy(item.payload, payload, payload_len);
  }

  if (xQueueSend(s_tx_queue, &item, pdMS_TO_TICKS(APP_QUEUE_SEND_TIMEOUT_MS)) !=
      pdPASS) {
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

static void radio_tx_task(void *arg) {
  (void)arg;
  uint8_t packet[APP_PACKET_MAX_LEN];

  while (true) {
    radio_tx_item_t item;
    if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) != pdPASS) {
      continue;
    }

    size_t packet_len = 0;
    if (!app_protocol_build_packet(s_board_address, item.type, item.payload,
                                   item.length, packet, sizeof(packet),
                                   &packet_len)) {
      ESP_LOGE(TAG, "failed to build radio packet");
      continue;
    }

    ESP_LOGD(TAG, "send type=0x%02x payload=%u packet=%u", (unsigned)item.type,
             (unsigned)item.length, (unsigned)packet_len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, packet, packet_len, ESP_LOG_DEBUG);

    if (xSemaphoreTake(s_uart_mutex, portMAX_DELAY) == pdTRUE) {
      const int tx_bytes = e34_2g4d20d_sendData(TAG, packet, packet_len);
      if (tx_bytes != (int)packet_len) {
        ESP_LOGW(TAG, "radio write short: %d/%u", tx_bytes,
                 (unsigned)packet_len);
      }
      xSemaphoreGive(s_uart_mutex);
    }
  }
}

static void radio_rx_task(void *arg) {
  (void)arg;
  uint8_t data[APP_PACKET_MAX_LEN];
  radio_rx_stream_t stream = {0};

  while (true) {
    if (!s_rx_enabled) {
      stream.length = 0U;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    const int rx_bytes = uart_read_bytes(
        UART_NUM_1, data, sizeof(data), pdMS_TO_TICKS(APP_RADIO_RX_TIMEOUT_MS));
    if (rx_bytes <= 0) {
      continue;
    }

    if (!s_rx_enabled) {
      stream.length = 0U;
      continue;
    }

    ESP_LOGD(TAG, "received %d bytes from radio", rx_bytes);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, rx_bytes, ESP_LOG_DEBUG);
    radio_rx_append(&stream, data, (size_t)rx_bytes);
  }
}

static void radio_reset_task(void *arg) {
  (void)arg;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(APP_RADIO_RESET_PERIOD_MS));
    s_rx_enabled = false;
    if (xSemaphoreTake(s_uart_mutex, portMAX_DELAY) == pdTRUE) {
      ESP_LOGI(TAG, "reset E34 module");
      ESP_ERROR_CHECK_WITHOUT_ABORT(uart_flush_input(UART_NUM_1));
      ESP_ERROR_CHECK_WITHOUT_ABORT(e34_2g4d20d_reset());
      ESP_ERROR_CHECK_WITHOUT_ABORT(uart_flush_input(UART_NUM_1));
      xSemaphoreGive(s_uart_mutex);
    }
    s_rx_enabled = true;
  }
}

static void radio_rx_append(radio_rx_stream_t *stream, const uint8_t *data,
                            size_t length) {
  for (size_t i = 0; i < length; ++i) {
    if (stream->length >= sizeof(stream->data)) {
      ESP_LOGW(TAG, "radio rx stream full, drop one byte");
      radio_rx_drop_prefix(stream, 1U);
    }

    stream->data[stream->length++] = data[i];
    radio_rx_process_stream(stream);
  }
}

static void radio_rx_process_stream(radio_rx_stream_t *stream) {
  while (stream->length >= APP_PACKET_OVERHEAD) {
    const size_t header_pos = radio_rx_find_header(stream);
    if (header_pos == stream->length) {
      radio_rx_drop_prefix(stream, stream->length - 1U);
      return;
    }

    if (header_pos > 0U) {
      radio_rx_drop_prefix(stream, header_pos);
    }

    if (stream->length >= APP_PACKET_HEADER_LEN &&
        !app_protocol_header_matches(s_board_address, stream->data,
                                     stream->length)) {
      ESP_LOGD(TAG, "skip radio frame header for another address or type");
      radio_rx_drop_prefix(stream, 1U);
      continue;
    }

    app_packet_view_t packet;
    size_t packet_len = 0;
    if (app_protocol_find_valid_packet(s_board_address, stream->data,
                                       stream->length, &packet_len, &packet)) {
      radio_rx_handle_packet(&packet);
      radio_rx_drop_prefix(stream, packet_len);
      continue;
    }

    if (stream->length >= APP_PACKET_MAX_LEN) {
      ESP_LOGW(TAG, "drop invalid radio frame candidate");
      radio_rx_drop_prefix(stream, 1U);
      continue;
    }

    return;
  }
}

static void radio_rx_handle_packet(const app_packet_view_t *packet) {
  if (packet->type != APP_FRAME_TYPE_CHANNEL1) {
    ESP_LOGW(TAG, "drop unsupported command type=0x%02x",
             (unsigned)packet->type);
    return;
  }

  esp_err_t ret = serial_router_submit_command(
      CH9434_UART_IDX_1, packet->payload, packet->payload_len);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "failed to enqueue serial command: %s", esp_err_to_name(ret));
  }
}

static void radio_rx_drop_prefix(radio_rx_stream_t *stream, size_t count) {
  if (count == 0U) {
    return;
  }

  if (count >= stream->length) {
    stream->length = 0U;
    return;
  }

  memmove(stream->data, &stream->data[count], stream->length - count);
  stream->length -= count;
}

static size_t radio_rx_find_header(const radio_rx_stream_t *stream) {
  for (size_t i = 0U; i + 1U < stream->length; ++i) {
    if (stream->data[i] == ((APP_PACKET_HEADER >> 8) & 0xFFU) &&
        stream->data[i + 1U] == (APP_PACKET_HEADER & 0xFFU)) {
      return i;
    }
  }

  return stream->length;
}
