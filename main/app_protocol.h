#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include "app_config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  APP_FRAME_TYPE_WEIGHT = 0x01,
  APP_FRAME_TYPE_CHANNEL1 = 0x02,
} app_frame_type_t;

typedef struct {
  app_frame_type_t type;
  const uint8_t *payload;
  size_t payload_len;
} app_packet_view_t;

void app_protocol_init(void);
uint32_t app_protocol_crc32(const uint8_t *data, size_t length);

bool app_protocol_build_packet(uint8_t address, app_frame_type_t type,
                               const uint8_t *payload, size_t payload_len,
                               uint8_t *out, size_t out_cap, size_t *out_len);

bool app_protocol_parse_packet(uint8_t address, const uint8_t *data,
                               size_t length, app_packet_view_t *packet);

bool app_protocol_find_valid_packet(uint8_t address, const uint8_t *data,
                                    size_t length, size_t *packet_len,
                                    app_packet_view_t *packet);

bool app_protocol_header_matches(uint8_t address, const uint8_t *data,
                                 size_t length);

bool app_protocol_is_weight_frame_complete(const uint8_t *data, size_t length);

#endif // APP_PROTOCOL_H
