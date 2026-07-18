#ifndef SERIAL_ROUTER_H
#define SERIAL_ROUTER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t serial_router_init(void);
esp_err_t serial_router_start(void);
esp_err_t serial_router_submit_command(uint8_t uart_idx, const uint8_t *data,
                                       size_t length);
esp_err_t serial_router_set_paused(bool paused);
bool serial_router_is_paused(void);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_ROUTER_H
