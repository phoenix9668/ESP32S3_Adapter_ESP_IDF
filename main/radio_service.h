#ifndef RADIO_SERVICE_H
#define RADIO_SERVICE_H

#include "app_protocol.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

esp_err_t radio_service_start(uint8_t board_address);
esp_err_t radio_service_send_frame(app_frame_type_t type,
                                   const uint8_t *payload,
                                   size_t payload_len);

#endif // RADIO_SERVICE_H
