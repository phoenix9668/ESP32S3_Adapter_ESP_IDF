#ifndef CELLULAR_4G_H
#define CELLULAR_4G_H

#include "cellular_4g_protocol.h"

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  bool has_valid_frame;
  bool has_complete_status;
  bool communication_stale;
  cellular_4g_protocol_frame_t latest;
} cellular_4g_status_t;

esp_err_t cellular_4g_init(void);
esp_err_t cellular_4g_send(const uint8_t *data, size_t length);
esp_err_t cellular_4g_request_status(void);
bool cellular_4g_get_status(cellular_4g_status_t *status);

#endif // CELLULAR_4G_H
