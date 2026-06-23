#include "app_protocol.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static void test_crc32_reference(void) {
  static const uint8_t input[] = "123456789";
  assert(app_protocol_crc32(input, strlen((const char *)input)) == 0xCBF43926U);
}

static void test_build_and_parse_packet(void) {
  static const uint8_t payload[] = {0x04, 0xFF, 0x01, 0x1B, 0xB4};
  uint8_t packet[APP_PACKET_MAX_LEN];
  size_t packet_len = 0;

  assert(app_protocol_build_packet(0x03, APP_FRAME_TYPE_CHANNEL1, payload,
                                   sizeof(payload), packet, sizeof(packet),
                                   &packet_len));
  assert(packet_len == APP_PACKET_OVERHEAD + sizeof(payload));
  assert(packet[0] == 0xE5);
  assert(packet[1] == 0x5E);
  assert(packet[3] == 0x03);
  assert(packet[5] == APP_FRAME_TYPE_CHANNEL1);

  app_packet_view_t parsed;
  assert(app_protocol_parse_packet(0x03, packet, packet_len, &parsed));
  assert(parsed.type == APP_FRAME_TYPE_CHANNEL1);
  assert(parsed.payload_len == sizeof(payload));
  assert(memcmp(parsed.payload, payload, sizeof(payload)) == 0);

  packet[packet_len - 1] ^= 0x01U;
  assert(!app_protocol_parse_packet(0x03, packet, packet_len, &parsed));
}

static void test_find_first_packet_in_stream(void) {
  static const uint8_t payload_a[] = "abc\r";
  static const uint8_t payload_b[] = {0x01, 0x02, 0x03};
  uint8_t stream[APP_PACKET_MAX_LEN * 2];
  size_t packet_a_len = 0;
  size_t packet_b_len = 0;

  assert(app_protocol_build_packet(0x0A, APP_FRAME_TYPE_WEIGHT, payload_a,
                                   sizeof(payload_a) - 1U, stream,
                                   sizeof(stream), &packet_a_len));
  assert(app_protocol_build_packet(
      0x0A, APP_FRAME_TYPE_CHANNEL1, payload_b, sizeof(payload_b),
      &stream[packet_a_len], sizeof(stream) - packet_a_len, &packet_b_len));

  app_packet_view_t parsed;
  size_t found_len = 0;
  assert(app_protocol_find_valid_packet(
      0x0A, stream, packet_a_len + packet_b_len, &found_len, &parsed));
  assert(found_len == packet_a_len);
  assert(parsed.type == APP_FRAME_TYPE_WEIGHT);
  assert(parsed.payload_len == sizeof(payload_a) - 1U);
}

static void test_reject_wrong_address_or_type(void) {
  static const uint8_t payload[] = {0x01};
  uint8_t packet[APP_PACKET_MAX_LEN];
  size_t packet_len = 0;

  assert(app_protocol_build_packet(0x01, APP_FRAME_TYPE_WEIGHT, payload,
                                   sizeof(payload), packet, sizeof(packet),
                                   &packet_len));
  assert(!app_protocol_header_matches(0x02, packet, packet_len));
  packet[5] = 0x7F;
  assert(!app_protocol_header_matches(0x01, packet, packet_len));
  assert(!app_protocol_build_packet(0x01, (app_frame_type_t)0x7F, payload,
                                    sizeof(payload), packet, sizeof(packet),
                                    &packet_len));
}

static void test_weight_frame_tails(void) {
  static const uint8_t cr_tail[] = {'1', '2', '3', '\r'};
  static const uint8_t crlf_tail[] = {'1', '2', '3', '\r', '\n'};
  static const uint8_t dot_hash_tail[] = {'1', '2', '.', '#'};
  static const uint8_t incomplete[] = {'1', '2', '3'};
  static const uint8_t ez3610_tail[] = {
      0x55, 0x04, 0x1B, 0x4F, 0x61, 0x40, 0x40, 0x41, 0x40, 0x40,
      0x45, 0x04, 0x1B, 0x4F, 0x75, 0x30, 0x36, 0x04, 0x1A, 0x04,
  };

  assert(app_protocol_is_weight_frame_complete(cr_tail, sizeof(cr_tail)));
  assert(app_protocol_is_weight_frame_complete(crlf_tail, sizeof(crlf_tail)));
  assert(app_protocol_is_weight_frame_complete(dot_hash_tail,
                                               sizeof(dot_hash_tail)));
  assert(
      app_protocol_is_weight_frame_complete(ez3610_tail, sizeof(ez3610_tail)));
  assert(
      !app_protocol_is_weight_frame_complete(incomplete, sizeof(incomplete)));
}

int main(void) {
  app_protocol_init();

  test_crc32_reference();
  test_build_and_parse_packet();
  test_find_first_packet_in_stream();
  test_reject_wrong_address_or_type();
  test_weight_frame_tails();

  return 0;
}
