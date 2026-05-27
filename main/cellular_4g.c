#include "cellular_4g.h"

#include "app_config.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#define CELLULAR_4G_UART_NUM UART_NUM_2
#define CELLULAR_4G_TXD_PIN GPIO_NUM_2
#define CELLULAR_4G_RXD_PIN GPIO_NUM_1
#define CELLULAR_4G_EN_PIN GPIO_NUM_44
#define CELLULAR_4G_BAUD_RATE 115200
#define CELLULAR_4G_RX_BUF_SIZE 1024
#define CELLULAR_4G_TX_BUF_SIZE 1024
#define CELLULAR_4G_RX_FRAME_BUF_SIZE 128

static const char *TAG = "CELLULAR_4G";
static const uint8_t s_heartbeat_pattern[] = "EG800K_HEARTBEAT\r\n";

static bool s_initialized;
static TickType_t s_last_heartbeat_tick;
static size_t s_heartbeat_match_len;

static bool cellular_4g_match_heartbeat_byte(uint8_t byte) {
  const size_t heartbeat_len = sizeof(s_heartbeat_pattern) - 1U;

  if (byte == s_heartbeat_pattern[s_heartbeat_match_len]) {
    s_heartbeat_match_len++;
    if (s_heartbeat_match_len == heartbeat_len) {
      s_heartbeat_match_len = 0;
      return true;
    }
    return false;
  }

  s_heartbeat_match_len = (byte == s_heartbeat_pattern[0]) ? 1U : 0U;
  return false;
}

static bool cellular_4g_process_rx_data(const uint8_t *data, size_t length) {
  bool heartbeat_seen = false;

  for (size_t i = 0; i < length; ++i) {
    if (cellular_4g_match_heartbeat_byte(data[i])) {
      heartbeat_seen = true;
    }
  }

  return heartbeat_seen;
}

static esp_err_t cellular_4g_enable_gpio_init(void) {
  const gpio_config_t config = {
      .pin_bit_mask = BIT64(CELLULAR_4G_EN_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&config);
  if (ret != ESP_OK) {
    return ret;
  }

  return gpio_set_level(CELLULAR_4G_EN_PIN, 1);
}

static void cellular_4g_restart(void) {
  ESP_LOGW(TAG, "4G heartbeat timeout, toggling EN gpio=%d",
           CELLULAR_4G_EN_PIN);

  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CELLULAR_4G_EN_PIN, 0));
  vTaskDelay(pdMS_TO_TICKS(APP_CELLULAR_RESTART_LOW_MS));
  uart_flush_input(CELLULAR_4G_UART_NUM);
  s_heartbeat_match_len = 0;
  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CELLULAR_4G_EN_PIN, 1));
}

static void cellular_4g_rx_watchdog_task(void *arg) {
  (void)arg;

  uint8_t rx_buf[CELLULAR_4G_RX_FRAME_BUF_SIZE];
  const TickType_t poll_timeout =
      pdMS_TO_TICKS(APP_CELLULAR_RX_POLL_TIMEOUT_MS);
  const TickType_t heartbeat_timeout =
      pdMS_TO_TICKS(APP_CELLULAR_HEARTBEAT_TIMEOUT_MS);

  while (true) {
    const int rx_bytes = uart_read_bytes(CELLULAR_4G_UART_NUM, rx_buf,
                                         sizeof(rx_buf), poll_timeout);
    const TickType_t now = xTaskGetTickCount();

    if (rx_bytes > 0) {
      if (cellular_4g_process_rx_data(rx_buf, (size_t)rx_bytes)) {
        s_last_heartbeat_tick = now;
        ESP_LOGD(TAG, "received 4G heartbeat");
      } else {
        ESP_LOGD(TAG, "ignored %d non-heartbeat bytes from 4G module",
                 rx_bytes);
        ESP_LOG_BUFFER_HEXDUMP(TAG, rx_buf, rx_bytes, ESP_LOG_DEBUG);
      }
    }

    if ((now - s_last_heartbeat_tick) >= heartbeat_timeout) {
      cellular_4g_restart();
      s_last_heartbeat_tick = xTaskGetTickCount();
    }
  }
}

/*
 * ESP32 UART2 TX connects to the 4G module RXD, and ESP32 UART2 RX connects
 * to the module TXD. The schematic routes the module EN signal to ESP32-S3
 * GPIO44, where low disables the module power and high enables it.
 */
esp_err_t cellular_4g_init(void) {
  if (s_initialized) {
    return ESP_OK;
  }

  esp_err_t ret = cellular_4g_enable_gpio_init();
  if (ret != ESP_OK) {
    return ret;
  }

  const uart_config_t uart_config = {
      .baud_rate = CELLULAR_4G_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  ret = uart_driver_install(CELLULAR_4G_UART_NUM, CELLULAR_4G_RX_BUF_SIZE,
                            CELLULAR_4G_TX_BUF_SIZE, 0, NULL, 0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    return ret;
  }

  ret = uart_param_config(CELLULAR_4G_UART_NUM, &uart_config);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = uart_set_pin(CELLULAR_4G_UART_NUM, CELLULAR_4G_TXD_PIN,
                     CELLULAR_4G_RXD_PIN, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    return ret;
  }

  uart_flush(CELLULAR_4G_UART_NUM);

  s_last_heartbeat_tick = xTaskGetTickCount();
  s_heartbeat_match_len = 0;
  const BaseType_t task_created =
      xTaskCreate(cellular_4g_rx_watchdog_task, "cellular_4g_rx",
                  APP_TASK_STACK_DEFAULT, NULL, tskIDLE_PRIORITY + 5, NULL);
  if (task_created != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  s_initialized = true;
  ESP_LOGI(TAG, "4G UART%d initialized tx=%d rx=%d en=%d baud=%d",
           CELLULAR_4G_UART_NUM, CELLULAR_4G_TXD_PIN, CELLULAR_4G_RXD_PIN,
           CELLULAR_4G_EN_PIN, CELLULAR_4G_BAUD_RATE);
  return ESP_OK;
}

esp_err_t cellular_4g_send(const uint8_t *data, size_t length) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (data == NULL || length == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  const int tx_bytes = uart_write_bytes(CELLULAR_4G_UART_NUM, data, length);
  if (tx_bytes < 0 || (size_t)tx_bytes != length) {
    ESP_LOGW(TAG, "4G UART write incomplete: %d/%u", tx_bytes,
             (unsigned)length);
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "sent %u bytes to 4G module", (unsigned)length);
  ESP_LOG_BUFFER_HEXDUMP(TAG, data, length, ESP_LOG_DEBUG);
  return ESP_OK;
}
