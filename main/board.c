#include "board.h"

#include "app_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GREEN_PIN GPIO_NUM_17
#define LED_BLUE_PIN GPIO_NUM_16

#define ADDR1 GPIO_NUM_9
#define ADDR2 GPIO_NUM_3
#define ADDR3 GPIO_NUM_8
#define ADDR4 GPIO_NUM_18

static const char *TAG = "BOARD";

static const gpio_num_t s_rs485_dir_pins[APP_CH9434_UART_COUNT] = {
    GPIO_NUM_47,
    GPIO_NUM_21,
    GPIO_NUM_14,
    GPIO_NUM_13,
};

static uint8_t s_board_address;
static esp_timer_handle_t s_blue_led_timer;

static void blue_led_timer_callback(void *arg) {
  (void)arg;
  board_led_set(BOARD_LED_BLUE, false);
}

static esp_err_t configure_leds(void) {
  const gpio_config_t gpio_conf = {
      .pin_bit_mask = (1ULL << LED_GREEN_PIN) | (1ULL << LED_BLUE_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&gpio_conf);
  if (ret != ESP_OK) {
    return ret;
  }

  board_led_set(BOARD_LED_GREEN, false);
  board_led_set(BOARD_LED_BLUE, false);

  const esp_timer_create_args_t timer_args = {
      .callback = blue_led_timer_callback,
      .name = "blue_led",
  };
  return esp_timer_create(&timer_args, &s_blue_led_timer);
}

static esp_err_t configure_rs485_dir_pins(void) {
  uint64_t pin_mask = 0;
  for (size_t i = 0; i < APP_CH9434_UART_COUNT; ++i) {
    pin_mask |= (1ULL << s_rs485_dir_pins[i]);
  }

  const gpio_config_t gpio_conf = {
      .pin_bit_mask = pin_mask,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&gpio_conf);
  if (ret != ESP_OK) {
    return ret;
  }

  for (uint8_t channel = 0; channel < APP_CH9434_UART_COUNT; ++channel) {
    board_rs485_set_direction(channel, BOARD_RS485_RX);
  }

  return ESP_OK;
}

static esp_err_t read_address_switch(void) {
  const gpio_config_t gpio_conf = {
      .pin_bit_mask =
          (1ULL << ADDR1) | (1ULL << ADDR2) | (1ULL << ADDR3) | (1ULL << ADDR4),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&gpio_conf);
  if (ret != ESP_OK) {
    return ret;
  }

  s_board_address =
      (uint8_t)(~((gpio_get_level(ADDR4) << 3) | (gpio_get_level(ADDR3) << 2) |
                  (gpio_get_level(ADDR2) << 1) | gpio_get_level(ADDR1)) &
                0x0F);

  ESP_LOGI(TAG, "board address: 0x%02x", s_board_address);
  return ESP_OK;
}

static void blink_address(void) {
  for (uint8_t i = 0; i < s_board_address; ++i) {
    board_led_set(BOARD_LED_GREEN, true);
    board_led_set(BOARD_LED_BLUE, true);
    vTaskDelay(pdMS_TO_TICKS(1000));
    board_led_set(BOARD_LED_GREEN, false);
    board_led_set(BOARD_LED_BLUE, false);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

esp_err_t board_init(void) {
  esp_err_t ret = configure_leds();
  if (ret != ESP_OK) {
    return ret;
  }

  ret = configure_rs485_dir_pins();
  if (ret != ESP_OK) {
    return ret;
  }

  ret = read_address_switch();
  if (ret != ESP_OK) {
    return ret;
  }

  blink_address();
  return ESP_OK;
}

uint8_t board_get_address(void) { return s_board_address; }

void board_led_set(board_led_t led, bool on) {
  gpio_num_t pin = led == BOARD_LED_BLUE ? LED_BLUE_PIN : LED_GREEN_PIN;
  gpio_set_level(pin, on ? 1 : 0);
}

void board_blue_led_pulse(uint32_t duration_ms) {
  if (s_blue_led_timer == NULL || duration_ms == 0U) {
    return;
  }

  board_led_set(BOARD_LED_BLUE, true);
  if (esp_timer_is_active(s_blue_led_timer)) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(s_blue_led_timer));
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_timer_start_once(s_blue_led_timer, (uint64_t)duration_ms * 1000U));
}

void board_rs485_set_direction(uint8_t channel, board_rs485_dir_t direction) {
  if (channel >= APP_CH9434_UART_COUNT) {
    ESP_LOGW(TAG, "ignore invalid RS485 channel %u", channel);
    return;
  }

  gpio_set_level(s_rs485_dir_pins[channel],
                 direction == BOARD_RS485_TX ? 1 : 0);
}
