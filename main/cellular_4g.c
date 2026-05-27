#include "cellular_4g.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <stdbool.h>

#define CELLULAR_4G_UART_NUM UART_NUM_2
#define CELLULAR_4G_TXD_PIN GPIO_NUM_2
#define CELLULAR_4G_RXD_PIN GPIO_NUM_1
#define CELLULAR_4G_BAUD_RATE 115200
#define CELLULAR_4G_RX_BUF_SIZE 1024
#define CELLULAR_4G_TX_BUF_SIZE 1024

static const char *TAG = "CELLULAR_4G";

static bool s_initialized;

/*
 * ESP32 UART2 TX connects to the 4G module RXD, and ESP32 UART2 RX connects
 * to the module TXD. EN/DTR are not driven here because the current schematic
 * exposes the 4G data link through U2TXD/U2RXD only.
 */
esp_err_t cellular_4g_init(void) {
  if (s_initialized) {
    return ESP_OK;
  }

  const uart_config_t uart_config = {
      .baud_rate = CELLULAR_4G_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret = uart_driver_install(CELLULAR_4G_UART_NUM,
                                      CELLULAR_4G_RX_BUF_SIZE,
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
  s_initialized = true;
  ESP_LOGI(TAG, "4G UART%d initialized tx=%d rx=%d baud=%d",
           CELLULAR_4G_UART_NUM, CELLULAR_4G_TXD_PIN, CELLULAR_4G_RXD_PIN,
           CELLULAR_4G_BAUD_RATE);
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
