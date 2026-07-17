#include "app_protocol_stream.h"

#include <string.h>

static void discard_prefix(app_protocol_stream_t *stream, size_t count) {
  if (count >= stream->length) {
    stream->length = 0U;
    return;
  }
  memmove(stream->buffer, stream->buffer + count, stream->length - count);
  stream->length -= count;
}

void app_protocol_stream_init(app_protocol_stream_t *stream, uint8_t address) {
  if (stream == NULL) {
    return;
  }
  memset(stream, 0, sizeof(*stream));
  stream->address = address;
}

size_t app_protocol_stream_feed(app_protocol_stream_t *stream,
                                const uint8_t *data, size_t length,
                                app_protocol_stream_callback_t callback,
                                void *context) {
  if (stream == NULL || (length > 0U && data == NULL)) {
    return 0U;
  }
  const size_t capacity = sizeof(stream->buffer);
  if (length > capacity) {
    data += length - capacity;
    length = capacity;
  }
  if (stream->length + length > capacity) {
    discard_prefix(stream, stream->length + length - capacity);
  }
  if (length > 0U) {
    memcpy(stream->buffer + stream->length, data, length);
    stream->length += length;
  }

  const uint8_t header_high = (uint8_t)(APP_PACKET_HEADER >> 8U);
  const uint8_t header_low = (uint8_t)APP_PACKET_HEADER;
  size_t frames = 0U;
  while (stream->length >= 2U) {
    size_t header_offset = 0U;
    while (header_offset + 1U < stream->length &&
           (stream->buffer[header_offset] != header_high ||
            stream->buffer[header_offset + 1U] != header_low)) {
      ++header_offset;
    }
    if (header_offset > 0U) {
      discard_prefix(stream, header_offset);
    }
    if (stream->length < APP_PACKET_OVERHEAD) {
      break;
    }
    if (stream->buffer[2] != 0x00U ||
        stream->buffer[3] != stream->address ||
        stream->buffer[4] != 0x00U ||
        (stream->buffer[5] != APP_FRAME_TYPE_WEIGHT &&
         stream->buffer[5] != APP_FRAME_TYPE_RFID)) {
      discard_prefix(stream, 1U);
      continue;
    }

    const size_t maximum = stream->length < APP_PACKET_MAX_LEN
                               ? stream->length
                               : APP_PACKET_MAX_LEN;
    bool complete = false;
    for (size_t candidate = APP_PACKET_OVERHEAD; candidate <= maximum;
         ++candidate) {
      app_packet_view_t packet;
      if (!app_protocol_parse_packet(stream->address, stream->buffer,
                                     candidate, &packet)) {
        continue;
      }
      if (callback != NULL) {
        callback(&packet, context);
      }
      ++frames;
      discard_prefix(stream, candidate);
      complete = true;
      break;
    }
    if (complete) {
      continue;
    }
    bool resynchronized = false;
    for (size_t offset = 1U;
         offset + APP_PACKET_OVERHEAD <= stream->length; ++offset) {
      if (stream->buffer[offset] != header_high ||
          stream->buffer[offset + 1U] != header_low) {
        continue;
      }
      const size_t available = stream->length - offset;
      const size_t nested_maximum =
          available < APP_PACKET_MAX_LEN ? available : APP_PACKET_MAX_LEN;
      for (size_t candidate = APP_PACKET_OVERHEAD;
           candidate <= nested_maximum; ++candidate) {
        app_packet_view_t nested_packet;
        if (app_protocol_parse_packet(stream->address,
                                      stream->buffer + offset, candidate,
                                      &nested_packet)) {
          discard_prefix(stream, offset);
          resynchronized = true;
          break;
        }
      }
      if (resynchronized) {
        break;
      }
    }
    if (resynchronized) {
      continue;
    }
    if (stream->length >= APP_PACKET_MAX_LEN) {
      discard_prefix(stream, 1U);
      continue;
    }
    break;
  }
  return frames;
}
