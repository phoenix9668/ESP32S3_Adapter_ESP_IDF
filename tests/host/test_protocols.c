#include "app_protocol.h"
#include "app_protocol_stream.h"
#include "gnss_ml307c.h"
#include "onenet_config.h"
#include "onenet_ota_protocol.h"
#include "onenet_reply.h"
#include "onenet_time.h"
#include "rfid_response.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  size_t count;
  app_frame_type_t type[4];
  size_t length[4];
  uint8_t payload[4][32];
} capture_t;

static void capture_packet(const app_packet_view_t *packet, void *context) {
  capture_t *capture = context;
  assert(capture->count < 4U);
  const size_t index = capture->count++;
  capture->type[index] = packet->type;
  capture->length[index] = packet->payload_len;
  assert(packet->payload_len <= sizeof(capture->payload[index]));
  memcpy(capture->payload[index], packet->payload, packet->payload_len);
}

static size_t make_packet(uint8_t address, app_frame_type_t type,
                          const char *payload, uint8_t *packet) {
  size_t length = 0U;
  assert(app_protocol_build_packet(address, type, (const uint8_t *)payload,
                                   strlen(payload), packet,
                                   APP_PACKET_MAX_LEN, &length));
  return length;
}

static void test_stream_split_and_joined(void) {
  uint8_t first[APP_PACKET_MAX_LEN];
  uint8_t second[APP_PACKET_MAX_LEN];
  const size_t first_len =
      make_packet(3U, APP_FRAME_TYPE_RFID, "READ", first);
  const size_t second_len =
      make_packet(3U, APP_FRAME_TYPE_RFID, "STOP", second);
  uint8_t joined[APP_PACKET_MAX_LEN * 2U];
  const uint8_t noise[] = {0x00, 0x11, 0xE5};
  memcpy(joined, noise, sizeof(noise));
  memcpy(joined + sizeof(noise), first, first_len);
  memcpy(joined + sizeof(noise) + first_len, second, second_len);

  app_protocol_stream_t stream;
  app_protocol_stream_init(&stream, 3U);
  capture_t capture = {};
  const size_t split = sizeof(noise) + 5U;
  assert(app_protocol_stream_feed(&stream, joined, split, capture_packet,
                                  &capture) == 0U);
  assert(app_protocol_stream_feed(
             &stream, joined + split,
             sizeof(noise) + first_len + second_len - split, capture_packet,
             &capture) == 2U);
  assert(capture.count == 2U);
  assert(capture.length[0] == 4U);
  assert(memcmp(capture.payload[0], "READ", 4U) == 0);
  assert(memcmp(capture.payload[1], "STOP", 4U) == 0);
}

static void test_stream_bad_crc_recovery(void) {
  uint8_t bad[APP_PACKET_MAX_LEN];
  uint8_t good[APP_PACKET_MAX_LEN];
  const size_t bad_len = make_packet(7U, APP_FRAME_TYPE_RFID, "BAD", bad);
  const size_t good_len = make_packet(7U, APP_FRAME_TYPE_RFID, "GOOD", good);
  bad[bad_len - 1U] ^= 0x5AU;
  uint8_t input[APP_PACKET_MAX_LEN * 2U];
  memcpy(input, bad, bad_len);
  memcpy(input + bad_len, good, good_len);

  app_protocol_stream_t stream;
  app_protocol_stream_init(&stream, 7U);
  capture_t capture = {};
  assert(app_protocol_stream_feed(&stream, input, bad_len + good_len,
                                  capture_packet, &capture) == 1U);
  assert(capture.count == 1U);
  assert(capture.length[0] == 4U);
  assert(memcmp(capture.payload[0], "GOOD", 4U) == 0);
}

static void test_gnss_coordinates(void) {
  gnss_fix_t fix;
  assert(gnss_ml307c_parse_line(
      "+MGNSSLOC: 123519.00,2234.5000N,11356.2500E,0.8,12.3,3,"
      "90.0,4.5,2.4,160726,10,0",
      &fix));
  assert(fix.valid);
  assert(fix.fix_type == 3U);
  assert(fabs(fix.latitude - 22.575) < 0.000001);
  assert(fabs(fix.longitude - 113.9375) < 0.000001);
  assert(strcmp(fix.utc_time, "2026-07-16T12:35:19Z") == 0);

  assert(gnss_ml307c_parse_line(
      "+MGNSSLOC: 010203.00,3456.0000S,05822.0000W,1.2,5.0,2,"
      "180.0,0.0,0.0,020126,7,0",
      &fix));
  assert(fix.valid);
  assert(fix.latitude < 0.0);
  assert(fix.longitude < 0.0);
}

static void test_gnss_empty_no_fix(void) {
  gnss_fix_t fix;
  assert(gnss_ml307c_parse_line(
      "+MGNSSLOC: 123519.00,,,99.9,,1,,,,160726,0,0", &fix));
  assert(!fix.valid);
  assert(fix.fix_type == 1U);
  assert(fix.satellites == 0U);
}

static void test_gnss_coordinate_bounds(void) {
  gnss_fix_t fix;
  assert(gnss_ml307c_parse_line(
      "+MGNSSLOC: 123519.00,9000.0000N,18000.0000E,0.8,0.0,3,"
      "0.0,0.0,0.0,160726,8,0",
      &fix));
  assert(fix.valid);

  assert(gnss_ml307c_parse_line(
      "+MGNSSLOC: 123519.00,9000.0001N,18000.0000E,0.8,0.0,3,"
      "0.0,0.0,0.0,160726,8,0",
      &fix));
  assert(!fix.valid);

  assert(gnss_ml307c_parse_line(
      "+MGNSSLOC: 123519.00,0000.0000N,18000.0001E,0.8,0.0,3,"
      "0.0,0.0,0.0,160726,8,0",
      &fix));
  assert(!fix.valid);
}

static void test_rfid_multiple_tags(void) {
  uint8_t frame[RFID_RESPONSE_MAX_FRAME_LEN] = {};
  const uint8_t first[RFID_RESPONSE_TAG_RAW_LEN] = {
      0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x10, 0x20, 0x30, 0x40};
  const uint8_t second[RFID_RESPONSE_TAG_RAW_LEN] = {
      0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10, 0xAA, 0xBB, 0xCC, 0xDD};
  const size_t length = 5U + 2U * (1U + RFID_RESPONSE_TAG_RAW_LEN) + 2U;
  frame[0] = (uint8_t)(length - 1U);
  frame[4] = 2U;
  frame[5] = RFID_RESPONSE_TAG_RAW_LEN;
  memcpy(&frame[6], first, sizeof(first));
  frame[18] = RFID_RESPONSE_TAG_RAW_LEN;
  memcpy(&frame[19], second, sizeof(second));

  rfid_response_t response;
  assert(rfid_response_parse(frame, length, &response));
  assert(response.count == 2U);
  assert(memcmp(response.tags[0], first, RFID_RESPONSE_TAG_UPLOAD_LEN) == 0);
  assert(memcmp(response.tags[1], second, RFID_RESPONSE_TAG_UPLOAD_LEN) == 0);

  frame[18] = RFID_RESPONSE_TAG_RAW_LEN - 1U;
  assert(!rfid_response_parse(frame, length, &response));
}

static void test_onenet_reply_matching(void) {
  char id[16];
  int code = -1;
  const char success[] = "{\"id\":\"42\",\"code\":200,\"msg\":\"success\"}";
  assert(onenet_reply_parse(success, strlen(success), id, sizeof(id), &code));
  assert(strcmp(id, "42") == 0);
  assert(code == 200);

  const char rejected[] = "{\"id\":\"42\",\"code\":500}";
  assert(onenet_reply_parse(rejected, strlen(rejected), id, sizeof(id), &code));
  assert(strcmp(id, "42") == 0);
  assert(code != 200);

  const char stale[] = "{\"id\":\"41\",\"code\":200}";
  assert(onenet_reply_parse(stale, strlen(stale), id, sizeof(id), &code));
  assert(strcmp(id, "42") != 0);
  assert(!onenet_reply_parse("{}", 2U, id, sizeof(id), &code));
}

static void test_onenet_token_vector(void) {
  onenet_config_t config = {};
  strcpy(config.product_id, "test-product");
  strcpy(config.device_name, "test-device");
  strcpy(config.device_key, "dGVzdC1rZXk=");
  char token[ONENET_TOKEN_MAX_LEN];
  assert(onenet_generate_mqtt_token(&config, 1893456000ULL, token,
                                    sizeof(token)) == ESP_OK);
  assert(strcmp(
             token,
             "version=2018-10-31&res=products%2Ftest-product%2Fdevices%2F"
             "test-device&et=1893456000&method=sha256&sign="
             "Lupe049xpJG17%2B7KQHCIHxYy2CTX%2BHYi%2BFt9LoIYyfk%3D") == 0);

  assert(onenet_generate_ota_token(&config, 1893456000ULL, token,
                                   sizeof(token)) == ESP_OK);
  assert(strcmp(
             token,
             "version=2018-10-31&res=products%2Ftest-product%2Fdevices%2F"
             "test-device&et=1893456000&method=sha1&sign="
             "SMnYQ%2F3hxwrnfgWAihw6qNFIrEY%3D") == 0);
}

static void test_onenet_network_clock(void) {
  uint64_t epoch = 0U;
  assert(onenet_time_parse_cclk("26/07/17,12:34:56+32", &epoch));
  /* ML307C's clock digits are UTC; +32 is retained NITZ metadata. */
  assert(epoch == 1784291696ULL);
  assert(onenet_time_parse_cclk("\"26/01/02,03:04:05+00\"", &epoch));
  assert(epoch == 1767323045ULL);
  assert(!onenet_time_parse_cclk("23/01/02,03:04:05+00", &epoch));
  assert(!onenet_time_parse_cclk("26/02/30,03:04:05+00", &epoch));
}

static void test_onenet_ota_protocol(void) {
  assert(onenet_ota_version_is_newer("1.0.1", "1.0.0"));
  assert(onenet_ota_version_is_newer("2.0.0", "1.99.99"));
  assert(!onenet_ota_version_is_newer("1.0.0", "1.0.0"));
  assert(!onenet_ota_version_is_newer("0.9.9", "1.0.0"));
  assert(onenet_ota_version_is_newer("1.0.0", "1.0.0-rc1"));

  assert(onenet_ota_content_range_matches("bytes 65536-131071/659456",
                                          65536U, 131071U, 659456U));
  assert(!onenet_ota_content_range_matches("bytes 0-65535/659456", 65536U,
                                           131071U, 659456U));
  assert(!onenet_ota_content_range_matches("bytes 0-65535/*", 0U, 65535U,
                                           659456U));
  assert(onenet_ota_resume_offset(588364U, 659456U, 4096U) == 585728U);
  assert(onenet_ota_resume_offset(659456U, 659456U, 4096U) == 0U);
  assert(onenet_ota_retry_delay_seconds(0U) == 5U);
  assert(onenet_ota_retry_delay_seconds(1U) == 15U);
  assert(onenet_ota_retry_delay_seconds(4U) == 300U);
  assert(onenet_ota_retry_delay_seconds(20U) == 300U);

  char id[16];
  const char inform[] =
      "{\"id\":\"ota-42\",\"version\":\"1.0\",\"params\":[]}";
  assert(onenet_ota_parse_inform_id(inform, strlen(inform), id,
                                    sizeof(id)));
  assert(strcmp(id, "ota-42") == 0);

  const char task_json[] =
      "{\"code\":0,\"msg\":\"succ\",\"data\":{"
      "\"target\":\"1.1.0\",\"tid\":12,\"size\":1048576,"
      "\"md5\":\"0123456789ABCDEF0123456789ABCDEF\",\"type\":1}}";
  onenet_ota_task_t task = {};
  assert(onenet_ota_parse_task(task_json, strlen(task_json), "1.0.0",
                               4U * 1024U * 1024U, &task));
  assert(strcmp(task.task_id, "12") == 0);
  assert(strcmp(task.target_version, "1.1.0") == 0);
  assert(strcmp(task.md5, "0123456789abcdef0123456789abcdef") == 0);
  assert(task.size == 1048576U);

  assert(!onenet_ota_parse_task(task_json, strlen(task_json), "1.1.0",
                                4U * 1024U * 1024U, &task));
  assert(!onenet_ota_parse_task(task_json, strlen(task_json), "1.0.0",
                                512U * 1024U, &task));
}

int main(void) {
  app_protocol_init();
  test_stream_split_and_joined();
  test_stream_bad_crc_recovery();
  test_gnss_coordinates();
  test_gnss_empty_no_fix();
  test_gnss_coordinate_bounds();
  test_rfid_multiple_tags();
  test_onenet_reply_matching();
  test_onenet_token_vector();
  test_onenet_network_clock();
  test_onenet_ota_protocol();
  puts("protocol_tests: all tests passed");
  return 0;
}
