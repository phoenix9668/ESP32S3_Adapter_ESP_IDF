#ifndef BOARD_H
#define BOARD_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  BOARD_LED_GREEN = 0,
  BOARD_LED_BLUE,
} board_led_t;

typedef enum {
  BOARD_RS485_RX = 0,
  BOARD_RS485_TX,
} board_rs485_dir_t;

esp_err_t board_init(void);
uint8_t board_get_address(void);
void board_led_set(board_led_t led, bool on);
void board_blue_led_pulse(uint32_t duration_ms);
void board_rs485_set_direction(uint8_t channel, board_rs485_dir_t direction);

#endif // BOARD_H
