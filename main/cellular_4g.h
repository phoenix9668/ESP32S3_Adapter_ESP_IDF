#ifndef CELLULAR_4G_H
#define CELLULAR_4G_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

esp_err_t cellular_4g_init(void);
esp_err_t cellular_4g_send(const uint8_t *data, size_t length);

#endif // CELLULAR_4G_H
