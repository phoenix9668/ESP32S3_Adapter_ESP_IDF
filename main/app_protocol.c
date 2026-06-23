#include "app_protocol.h"

#include <string.h>

#define CRC32_POLYNOMIAL 0xEDB88320UL

static uint32_t s_crc32_table[256];
static bool s_crc32_ready;

void app_protocol_init(void) {
  if (s_crc32_ready) {
    return;
  }

  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t crc = i;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1U) ? CRC32_POLYNOMIAL : 0U);
    }
    s_crc32_table[i] = crc;
  }

  s_crc32_ready = true;
}

uint32_t app_protocol_crc32(const uint8_t *data, size_t length) {
  app_protocol_init();

  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc = (crc >> 8) ^ s_crc32_table[(crc & 0xFFU) ^ data[i]];
  }

  return ~crc;
}

bool app_protocol_build_packet(uint8_t address, app_frame_type_t type,
                               const uint8_t *payload, size_t payload_len,
                               uint8_t *out, size_t out_cap, size_t *out_len) {
  if (out == NULL || out_len == NULL || payload_len > APP_PAYLOAD_MAX_LEN ||
      (type != APP_FRAME_TYPE_WEIGHT && type != APP_FRAME_TYPE_CHANNEL1) ||
      (payload_len > 0U && payload == NULL)) {
    return false;
  }

  const size_t packet_len = APP_PACKET_OVERHEAD + payload_len;
  if (out_cap < packet_len) {
    return false;
  }

  out[0] = (uint8_t)((APP_PACKET_HEADER >> 8) & 0xFFU);
  out[1] = (uint8_t)(APP_PACKET_HEADER & 0xFFU);
  out[2] = 0x00;
  out[3] = address;
  out[4] = 0x00;
  out[5] = (uint8_t)type;

  if (payload_len > 0U) {
    memcpy(&out[APP_PACKET_HEADER_LEN], payload, payload_len);
  }

  const uint32_t crc =
      app_protocol_crc32(out, APP_PACKET_HEADER_LEN + payload_len);
  const size_t crc_pos = APP_PACKET_HEADER_LEN + payload_len;
  out[crc_pos] = (uint8_t)((crc >> 24) & 0xFFU);
  out[crc_pos + 1] = (uint8_t)((crc >> 16) & 0xFFU);
  out[crc_pos + 2] = (uint8_t)((crc >> 8) & 0xFFU);
  out[crc_pos + 3] = (uint8_t)(crc & 0xFFU);

  *out_len = packet_len;
  return true;
}

bool app_protocol_header_matches(uint8_t address, const uint8_t *data,
                                 size_t length) {
  return data != NULL && length >= APP_PACKET_HEADER_LEN &&
         data[0] == ((APP_PACKET_HEADER >> 8) & 0xFFU) &&
         data[1] == (APP_PACKET_HEADER & 0xFFU) && data[2] == 0x00 &&
         data[3] == address && data[4] == 0x00 &&
         (data[5] == APP_FRAME_TYPE_WEIGHT ||
          data[5] == APP_FRAME_TYPE_CHANNEL1);
}

bool app_protocol_parse_packet(uint8_t address, const uint8_t *data,
                               size_t length, app_packet_view_t *packet) {
  if (packet == NULL || length < APP_PACKET_OVERHEAD ||
      length > APP_PACKET_MAX_LEN ||
      !app_protocol_header_matches(address, data, length)) {
    return false;
  }

  const size_t payload_len = length - APP_PACKET_OVERHEAD;
  const size_t crc_pos = APP_PACKET_HEADER_LEN + payload_len;
  const uint32_t expected_crc =
      app_protocol_crc32(data, APP_PACKET_HEADER_LEN + payload_len);
  const uint32_t actual_crc =
      ((uint32_t)data[crc_pos] << 24) | ((uint32_t)data[crc_pos + 1] << 16) |
      ((uint32_t)data[crc_pos + 2] << 8) | data[crc_pos + 3];

  if (expected_crc != actual_crc) {
    return false;
  }

  packet->type = (app_frame_type_t)data[5];
  packet->payload = &data[APP_PACKET_HEADER_LEN];
  packet->payload_len = payload_len;
  return true;
}

bool app_protocol_find_valid_packet(uint8_t address, const uint8_t *data,
                                    size_t length, size_t *packet_len,
                                    app_packet_view_t *packet) {
  if (packet_len == NULL || packet == NULL ||
      !app_protocol_header_matches(address, data, length)) {
    return false;
  }

  const size_t max_len =
      length < APP_PACKET_MAX_LEN ? length : APP_PACKET_MAX_LEN;
  for (size_t candidate_len = APP_PACKET_OVERHEAD; candidate_len <= max_len;
       ++candidate_len) {
    if (app_protocol_parse_packet(address, data, candidate_len, packet)) {
      *packet_len = candidate_len;
      return true;
    }
  }

  return false;
}

bool app_protocol_is_weight_frame_complete(const uint8_t *data, size_t length) {
  static const uint8_t ez3610_frame_tail[] = {
      0x04, 0x1B, 0x4F, 0x61, 0x40, 0x40, 0x41, 0x40, 0x40, 0x45,
      0x04, 0x1B, 0x4F, 0x75, 0x30, 0x36, 0x04, 0x1A, 0x04,
  };

  if (data == NULL || length == 0U) {
    return false;
  }

  const uint8_t last = data[length - 1];
  if (last == 0x0D) {
    return true;
  }

  if (length >= 2U) {
    const uint8_t prev = data[length - 2];
    if ((last == 0x0A && prev == 0x0D) || (last == 0x23 && prev == 0x2E)) {
      return true;
    }
  }

  return length >= sizeof(ez3610_frame_tail) &&
         memcmp(&data[length - sizeof(ez3610_frame_tail)], ez3610_frame_tail,
                sizeof(ez3610_frame_tail)) == 0;
}
