#ifndef SERIAL_ROUTER_H
#define SERIAL_ROUTER_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

esp_err_t serial_router_init(void);
esp_err_t serial_router_start(void);
esp_err_t serial_router_submit_command(uint8_t uart_idx, const uint8_t *data,
                                       size_t length);

#endif // SERIAL_ROUTER_H
