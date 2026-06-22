#ifndef RFID_RESPONSE_H
#define RFID_RESPONSE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RFID_RESPONSE_MAX_FRAME_LEN 256U
#define RFID_RESPONSE_TAG_RAW_LEN 12U
#define RFID_RESPONSE_TAG_UPLOAD_LEN 8U
#define RFID_RESPONSE_MAX_TAGS 19U

typedef struct {
  uint8_t count;
  uint8_t tags[RFID_RESPONSE_MAX_TAGS][RFID_RESPONSE_TAG_UPLOAD_LEN];
} rfid_response_t;

bool rfid_response_parse(const uint8_t *data, size_t length,
                         rfid_response_t *response);

#endif // RFID_RESPONSE_H
