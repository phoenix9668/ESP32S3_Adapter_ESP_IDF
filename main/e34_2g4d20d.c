/********************************** (C) COPYRIGHT
 ******************************** File Name          : e34_2g4d20d.h Author :
 *pheonix Version            : V1.0 Date               : 2023/11/22 Description
 *: e34_2g4d20d driver file
 *******************************************************************************/

#include "e34_2g4d20d.h"
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
void e34_2g4d20d_uart1_init(void) {
  esp_log_level_set(E34_2G4D20D_TAG, ESP_LOG_DEBUG);
  ESP_LOGD(E34_2G4D20D_TAG, "Initializing bus UART%d...", UART_NUM_1);
  const uart_config_t uart1_config = {
      .baud_rate = E34_2G4D20D_Baudrate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  // We won't use a buffer for sending data.
  uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart1_config);
  uart_set_pin(UART_NUM_1, UART1_TXD_PIN, UART1_RXD_PIN, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
}

void e34_2g4d20d_gpio_init(void) {
  gpio_config_t gpio_conf;
  gpio_conf.mode = GPIO_MODE_OUTPUT;
  gpio_conf.pin_bit_mask = (1ULL << M0) | (1ULL << M1);
  gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&gpio_conf);

  gpio_set_level(M0, 1);
  gpio_set_level(M1, 0);

  gpio_conf.mode = GPIO_MODE_INPUT;
  gpio_conf.pin_bit_mask = (1ULL << AUX);
  gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&gpio_conf);
}

int e34_2g4d20d_sendData(const char *logName, const uint8_t *data,
                         size_t len) {
  const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
  ESP_LOGD(logName, "Wrote %d bytes", txBytes);
  ESP_LOG_BUFFER_HEXDUMP(logName, data, len, ESP_LOG_DEBUG);
  return txBytes;
}

/* -----------------------------------------------------------------------------
 *                                  api function
 * -----------------------------------------------------------------------------
 */

void e34_2g4d20d_model_sel(modes_t mode) {
  switch (mode) {
  case HALF_DUPLEX:
    gpio_set_level(M0, 0);
    gpio_set_level(M1, 0);
    break;
  case FULL_DUPLEX:
    gpio_set_level(M0, 1);
    gpio_set_level(M1, 0);
    break;
  case RESERVE:
    gpio_set_level(M0, 0);
    gpio_set_level(M1, 1);
    break;
  case SET:
    gpio_set_level(M0, 1);
    gpio_set_level(M1, 1);
    break;
  }
}

void e34_2g4d20d_parameter_set(char dev_head, char dev_addh, char dev_addl,
                               char dev_sped, char dev_chan, char dev_option) {
  // char dev_head = 0xc0;
  // char dev_addh = 0x00;
  // char dev_addl = 0x00;
  // char dev_sped = 0x18;
  // char dev_chan = 0x00;
  // char dev_option = 0x40;
  uint8_t set_para[] = {(uint8_t)dev_head, (uint8_t)dev_addh,
                        (uint8_t)dev_addl, (uint8_t)dev_sped,
                        (uint8_t)dev_chan, (uint8_t)dev_option};
  e34_2g4d20d_model_sel(SET);
  vTaskDelay(pdMS_TO_TICKS(100));
  e34_2g4d20d_sendData(E34_2G4D20D_TAG, set_para, sizeof(set_para));
  vTaskDelay(pdMS_TO_TICKS(1000));
  e34_2g4d20d_sendData(E34_2G4D20D_TAG, read_parameter,
                       sizeof(read_parameter));
  vTaskDelay(pdMS_TO_TICKS(1000));
  e34_2g4d20d_model_sel(FULL_DUPLEX);
}

void e34_2g4d20d_reset(void) {
  e34_2g4d20d_model_sel(SET);
  vTaskDelay(pdMS_TO_TICKS(100));
  e34_2g4d20d_sendData(E34_2G4D20D_TAG, reset_device, sizeof(reset_device));
  vTaskDelay(pdMS_TO_TICKS(100));
  e34_2g4d20d_model_sel(FULL_DUPLEX);
}
