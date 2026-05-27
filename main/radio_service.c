#include "radio_service.h"

#include "app_config.h"
#include "board.h"
#include "ch9434.h"
#include "driver/uart.h"
#include "e34_2g4d20d.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "serial_router.h"
#include <string.h>

typedef struct {
  app_frame_type_t type;
  size_t length;
  uint8_t payload[APP_PAYLOAD_MAX_LEN];
} radio_tx_item_t;

static const char *TAG = "RADIO";

static QueueHandle_t s_tx_queue;
static SemaphoreHandle_t s_uart_mutex;
static uint8_t s_board_address;

static void radio_tx_task(void *arg);
static void radio_rx_task(void *arg);
static void radio_reset_task(void *arg);

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

  e34_2g4d20d_uart1_init();
  e34_2g4d20d_gpio_init();

  xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
  e34_2g4d20d_parameter_set(0xC0, 0x00, 0x00, 0x18, 0x00, 0x40);
  xSemaphoreGive(s_uart_mutex);

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
                                   const uint8_t *payload,
                                   size_t payload_len) {
  if (s_tx_queue == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (payload_len > APP_PAYLOAD_MAX_LEN ||
      (payload_len > 0 && payload == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }

  radio_tx_item_t item = {
      .type = type,
      .length = payload_len,
  };

  if (payload_len > 0) {
    memcpy(item.payload, payload, payload_len);
  }

  if (xQueueSend(s_tx_queue, &item, pdMS_TO_TICKS(APP_QUEUE_SEND_TIMEOUT_MS)) !=
      pdPASS) {
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

static void radio_tx_task(void *arg) {
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

    ESP_LOGD(TAG, "send type=0x%02x payload=%u packet=%u", item.type,
             (unsigned)item.length, (unsigned)packet_len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, packet, packet_len, ESP_LOG_DEBUG);

    board_led_set(BOARD_LED_BLUE, true);
    if (xSemaphoreTake(s_uart_mutex, portMAX_DELAY) == pdTRUE) {
      e34_2g4d20d_sendData(TAG, packet, packet_len);
      xSemaphoreGive(s_uart_mutex);
    }
    board_led_set(BOARD_LED_BLUE, false);
  }
}

static void radio_rx_task(void *arg) {
  uint8_t data[APP_PACKET_MAX_LEN];

  while (true) {
    const int rx_bytes = uart_read_bytes(
        UART_NUM_1, data, sizeof(data), pdMS_TO_TICKS(APP_RADIO_RX_TIMEOUT_MS));
    if (rx_bytes <= 0) {
      continue;
    }

    ESP_LOGD(TAG, "received %d bytes from radio", rx_bytes);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, rx_bytes, ESP_LOG_DEBUG);

    app_packet_view_t packet;
    if (!app_protocol_parse_packet(s_board_address, data, (size_t)rx_bytes,
                                   &packet)) {
      ESP_LOGW(TAG, "drop invalid radio packet, len=%d", rx_bytes);
      continue;
    }

    if (packet.type != APP_FRAME_TYPE_RFID) {
      ESP_LOGW(TAG, "drop unsupported command type=0x%02x", packet.type);
      continue;
    }

    esp_err_t ret = serial_router_submit_command(CH9434_UART_IDX_1,
                                                 packet.payload,
                                                 packet.payload_len);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "failed to enqueue serial command: %s",
               esp_err_to_name(ret));
    }
  }
}

static void radio_reset_task(void *arg) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(APP_RADIO_RESET_PERIOD_MS));
    if (xSemaphoreTake(s_uart_mutex, portMAX_DELAY) == pdTRUE) {
      ESP_LOGI(TAG, "reset E34 module");
      e34_2g4d20d_reset();
      xSemaphoreGive(s_uart_mutex);
    }
  }
}
