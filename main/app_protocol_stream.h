#ifndef APP_PROTOCOL_STREAM_H
#define APP_PROTOCOL_STREAM_H

#include "app_protocol.h"
#include <stddef.h>
#include <stdint.h>

typedef void (*app_protocol_stream_callback_t)(
    const app_packet_view_t *packet, void *context);

typedef struct {
  uint8_t address;
  uint8_t buffer[APP_PACKET_MAX_LEN * 2U];
  size_t length;
} app_protocol_stream_t;

void app_protocol_stream_init(app_protocol_stream_t *stream, uint8_t address);
size_t app_protocol_stream_feed(app_protocol_stream_t *stream,
                                const uint8_t *data, size_t length,
                                app_protocol_stream_callback_t callback,
                                void *context);

#endif
