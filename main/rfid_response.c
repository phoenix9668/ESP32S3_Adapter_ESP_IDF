#include "rfid_response.h"

#include <string.h>

#define RFID_RESPONSE_TAG_COUNT_OFFSET 4U
#define RFID_RESPONSE_TAG_RECORDS_OFFSET 5U
#define RFID_RESPONSE_CRC_LEN 2U
#define RFID_RESPONSE_TAG_RECORD_LEN (1U + RFID_RESPONSE_TAG_RAW_LEN)
#define RFID_RESPONSE_MIN_FRAME_LEN                                            \
  (RFID_RESPONSE_TAG_RECORDS_OFFSET + RFID_RESPONSE_CRC_LEN)

bool rfid_response_parse(const uint8_t *data, size_t length,
                         rfid_response_t *response) {
  if (data == NULL || response == NULL ||
      length < RFID_RESPONSE_MIN_FRAME_LEN ||
      length > RFID_RESPONSE_MAX_FRAME_LEN || (size_t)data[0] + 1U != length) {
    return false;
  }

  const uint8_t tag_count = data[RFID_RESPONSE_TAG_COUNT_OFFSET];
  if (tag_count > RFID_RESPONSE_MAX_TAGS) {
    return false;
  }

  const size_t expected_length =
      RFID_RESPONSE_TAG_RECORDS_OFFSET +
      (size_t)tag_count * RFID_RESPONSE_TAG_RECORD_LEN + RFID_RESPONSE_CRC_LEN;
  if (length != expected_length) {
    return false;
  }

  memset(response, 0, sizeof(*response));
  response->count = tag_count;

  size_t cursor = RFID_RESPONSE_TAG_RECORDS_OFFSET;
  for (uint8_t tag_index = 0U; tag_index < tag_count; ++tag_index) {
    if (data[cursor++] != RFID_RESPONSE_TAG_RAW_LEN) {
      return false;
    }

    memcpy(response->tags[tag_index], &data[cursor],
           RFID_RESPONSE_TAG_UPLOAD_LEN);
    cursor += RFID_RESPONSE_TAG_RAW_LEN;
  }

  return cursor + RFID_RESPONSE_CRC_LEN == length;
}
