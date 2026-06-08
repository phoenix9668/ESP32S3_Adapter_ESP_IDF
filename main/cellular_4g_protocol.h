#ifndef CELLULAR_4G_PROTOCOL_H
#define CELLULAR_4G_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CELLULAR_4G_PROTOCOL_VERSION 1U
#define CELLULAR_4G_PROTOCOL_MAX_FRAME_LEN 192U
#define CELLULAR_4G_PROTOCOL_MAX_LINE_LEN                                      \
  (CELLULAR_4G_PROTOCOL_MAX_FRAME_LEN - 2U)
#define CELLULAR_4G_PROTOCOL_RAT_LEN 8U
#define CELLULAR_4G_PROTOCOL_EVENT_LEN 24U

typedef enum {
  CELLULAR_4G_MESSAGE_STAT = 0,
  CELLULAR_4G_MESSAGE_EVENT,
  CELLULAR_4G_MESSAGE_ACK,
} cellular_4g_message_type_t;

typedef struct {
  cellular_4g_message_type_t type;
  uint32_t seq;
  uint32_t uptime_sec;
  uint8_t state;
  uint8_t sim;
  uint8_t registration;
  uint8_t pdp;
  uint8_t mqtt;
  uint8_t csq;
  char rat[CELLULAR_4G_PROTOCOL_RAT_LEN];
  int32_t last_tx_sec;
  uint8_t cause;
  char event[CELLULAR_4G_PROTOCOL_EVENT_LEN];
  uint32_t ref_seq;
} cellular_4g_protocol_frame_t;

uint16_t cellular_4g_protocol_crc16(const char *data, size_t length);
bool cellular_4g_protocol_parse(const char *line, size_t length,
                                cellular_4g_protocol_frame_t *frame);

#endif // CELLULAR_4G_PROTOCOL_H
