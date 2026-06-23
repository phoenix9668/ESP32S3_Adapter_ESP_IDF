/********************************** (C) COPYRIGHT
 ******************************** File Name          : e34_2g4d20d.c Author :
 *pheonix Version            : V1.0 Date               : 2023/11/22 Description
 *: e34_2g4d20d driver file
 *******************************************************************************/

#include "e34_2g4d20d.h"

#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * e34_2g4d20d parameters definition
 */
#define RX_BUF_SIZE 512
static const char *E34_2G4D20D_TAG = "E34_2G4D20D";
static const uint8_t read_parameter[] = {0xC1, 0xC1, 0xC1};
static const uint8_t reset_device[] = {0xC4, 0xC4, 0xC4};

/* -----------------------------------------------------------------------------
 *                      define E34_2G4D20D interface functions
 * -----------------------------------------------------------------------------
 */
esp_err_t e34_2g4d20d_uart1_init(void) {
  ESP_LOGD(E34_2G4D20D_TAG, "Initializing bus UART%d...", UART_NUM_1);
  const uart_config_t uart1_config = {
      .baud_rate = E34_2G4D20D_Baudrate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret =
      uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    return ret;
  }

  ret = uart_param_config(UART_NUM_1, &uart1_config);
  if (ret != ESP_OK) {
    return ret;
  }

  return uart_set_pin(UART_NUM_1, UART1_TXD_PIN, UART1_RXD_PIN,
                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

esp_err_t e34_2g4d20d_gpio_init(void) {
  const gpio_config_t mode_gpio_conf = {
      .pin_bit_mask = (1ULL << M0) | (1ULL << M1),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t ret = gpio_config(&mode_gpio_conf);
  if (ret != ESP_OK) {
    return ret;
  }

  const gpio_config_t aux_gpio_conf = {
      .pin_bit_mask = (1ULL << AUX),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ret = gpio_config(&aux_gpio_conf);
  if (ret != ESP_OK) {
    return ret;
  }

  gpio_set_level(M0, 1);
  gpio_set_level(M1, 0);
  return ESP_OK;
}

esp_err_t e34_2g4d20d_wait_aux(uint32_t timeout_ms) {
  const TickType_t start = xTaskGetTickCount();
  const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

  while ((xTaskGetTickCount() - start) <= timeout) {
    if (gpio_get_level(AUX) != 0) {
      return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  return ESP_ERR_TIMEOUT;
}

int e34_2g4d20d_sendData(const char *logName, const uint8_t *data, size_t len) {
  if (data == NULL || len == 0U) {
    return 0;
  }

  const esp_err_t ready = e34_2g4d20d_wait_aux(APP_E34_AUX_TIMEOUT_MS);
  if (ready != ESP_OK) {
    ESP_LOGW(logName, "E34 AUX not ready before TX: %s",
             esp_err_to_name(ready));
  }

  const int tx_bytes = uart_write_bytes(UART_NUM_1, data, len);
  ESP_LOGD(logName, "Wrote %d bytes", tx_bytes);
  ESP_LOG_BUFFER_HEXDUMP(logName, data, len, ESP_LOG_DEBUG);

  if (tx_bytes > 0) {
    const uint32_t tx_time_ms = (uint32_t)(((size_t)tx_bytes * 11U * 1000U +
                                            E34_2G4D20D_Baudrate - 1U) /
                                           E34_2G4D20D_Baudrate) +
                                20U;
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(tx_time_ms)));

    const esp_err_t done = e34_2g4d20d_wait_aux(APP_E34_AUX_TIMEOUT_MS);
    if (done != ESP_OK) {
      ESP_LOGW(logName, "E34 AUX not ready after TX: %s",
               esp_err_to_name(done));
    }
  }

  return tx_bytes;
}

/* -----------------------------------------------------------------------------
 *                                  api function
 * -----------------------------------------------------------------------------
 */

void e34_2g4d20d_model_sel(modes_t mode) {
  if (mode == HALF_DUPLEX) {
    gpio_set_level(M0, 0);
    gpio_set_level(M1, 0);
  } else if (mode == FULL_DUPLEX) {
    gpio_set_level(M0, 1);
    gpio_set_level(M1, 0);
  } else if (mode == RESERVE) {
    gpio_set_level(M0, 0);
    gpio_set_level(M1, 1);
  } else if (mode == SET) {
    gpio_set_level(M0, 1);
    gpio_set_level(M1, 1);
  }
}

esp_err_t e34_2g4d20d_parameter_set(uint8_t dev_head, uint8_t dev_addh,
                                    uint8_t dev_addl, uint8_t dev_sped,
                                    uint8_t dev_chan, uint8_t dev_option) {
  uint8_t set_para[] = {dev_head, dev_addh, dev_addl,
                        dev_sped, dev_chan, dev_option};
  esp_err_t ret = ESP_OK;

  e34_2g4d20d_model_sel(SET);
  vTaskDelay(pdMS_TO_TICKS(100));

  esp_err_t ready = e34_2g4d20d_wait_aux(APP_E34_AUX_TIMEOUT_MS);
  if (ready != ESP_OK) {
    ESP_LOGW(E34_2G4D20D_TAG, "E34 not ready in SET mode: %s",
             esp_err_to_name(ready));
  }

  if (e34_2g4d20d_sendData(E34_2G4D20D_TAG, set_para, sizeof(set_para)) !=
      (int)sizeof(set_para)) {
    ret = ESP_FAIL;
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
  if (e34_2g4d20d_sendData(E34_2G4D20D_TAG, read_parameter,
                           sizeof(read_parameter)) !=
      (int)sizeof(read_parameter)) {
    ret = ESP_FAIL;
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
  e34_2g4d20d_model_sel(FULL_DUPLEX);
  vTaskDelay(pdMS_TO_TICKS(100));
  return ret;
}

esp_err_t e34_2g4d20d_reset(void) {
  esp_err_t ret = ESP_OK;

  e34_2g4d20d_model_sel(SET);
  vTaskDelay(pdMS_TO_TICKS(100));

  esp_err_t ready = e34_2g4d20d_wait_aux(APP_E34_AUX_TIMEOUT_MS);
  if (ready != ESP_OK) {
    ESP_LOGW(E34_2G4D20D_TAG, "E34 not ready before reset: %s",
             esp_err_to_name(ready));
  }

  if (e34_2g4d20d_sendData(E34_2G4D20D_TAG, reset_device,
                           sizeof(reset_device)) != (int)sizeof(reset_device)) {
    ret = ESP_FAIL;
  }

  vTaskDelay(pdMS_TO_TICKS(100));
  e34_2g4d20d_model_sel(FULL_DUPLEX);
  return ret;
}
