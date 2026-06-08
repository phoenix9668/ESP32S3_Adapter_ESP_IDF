#include "cellular_4g_protocol.h"

#include <limits.h>
#include <string.h>

enum {
  FIELD_V = 1U << 0,
  FIELD_T = 1U << 1,
  FIELD_SEQ = 1U << 2,
  FIELD_UP = 1U << 3,
  FIELD_STATE = 1U << 4,
  FIELD_SIM = 1U << 5,
  FIELD_REG = 1U << 6,
  FIELD_PDP = 1U << 7,
  FIELD_MQTT = 1U << 8,
  FIELD_CSQ = 1U << 9,
  FIELD_RAT = 1U << 10,
  FIELD_LASTTX = 1U << 11,
  FIELD_CAUSE = 1U << 12,
  FIELD_EV = 1U << 13,
  FIELD_REF = 1U << 14,
  FIELD_RESULT = 1U << 15,
};

static bool parse_u32(const char *text, uint32_t *value) {
  if (text == NULL || *text == '\0' || value == NULL) {
    return false;
  }

  uint32_t result = 0;
  for (const char *cursor = text; *cursor != '\0'; ++cursor) {
    if (*cursor < '0' || *cursor > '9') {
      return false;
    }
    const uint32_t digit = (uint32_t)(*cursor - '0');
    if (result > (UINT32_MAX - digit) / 10U) {
      return false;
    }
    result = result * 10U + digit;
  }

  *value = result;
  return true;
}

static bool parse_i32(const char *text, int32_t *value) {
  if (text == NULL || value == NULL) {
    return false;
  }
  if (strcmp(text, "-1") == 0) {
    *value = -1;
    return true;
  }

  uint32_t parsed = 0;
  if (!parse_u32(text, &parsed) || parsed > INT32_MAX) {
    return false;
  }
  *value = (int32_t)parsed;
  return true;
}

static int hex_nibble(char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'A' && value <= 'F') {
    return value - 'A' + 10;
  }
  return -1;
}

static bool parse_crc(const char *text, uint16_t *crc) {
  if (text == NULL || crc == NULL) {
    return false;
  }

  uint16_t result = 0;
  for (size_t i = 0; i < 4U; ++i) {
    const int nibble = hex_nibble(text[i]);
    if (nibble < 0) {
      return false;
    }
    result = (uint16_t)((result << 4U) | (uint16_t)nibble);
  }
  *crc = result;
  return true;
}

static bool valid_cause(uint32_t cause) { return cause <= 10U || cause == 99U; }

static bool valid_event_name(const char *event) {
  if (event == NULL || *event == '\0') {
    return false;
  }
  for (const char *cursor = event; *cursor != '\0'; ++cursor) {
    if ((*cursor < 'A' || *cursor > 'Z') && *cursor != '_' &&
        (*cursor < '0' || *cursor > '9')) {
      return false;
    }
  }
  return true;
}

static bool copy_text(char *destination, size_t destination_size,
                      const char *source) {
  const size_t length = strlen(source);
  if (length == 0U || length >= destination_size) {
    return false;
  }
  memcpy(destination, source, length + 1U);
  return true;
}

uint16_t cellular_4g_protocol_crc16(const char *data, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)((uint8_t)data[i]) << 8U;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1U) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1U);
      }
    }
  }
  return crc;
}

bool cellular_4g_protocol_parse(const char *line, size_t length,
                                cellular_4g_protocol_frame_t *frame) {
  if (line == NULL || frame == NULL || length == 0U ||
      length > CELLULAR_4G_PROTOCOL_MAX_LINE_LEN) {
    return false;
  }

  size_t star_index = length;
  for (size_t i = 0; i < length; ++i) {
    if (line[i] < 32 || line[i] > 126) {
      return false;
    }
    if (line[i] == '*') {
      if (star_index != length) {
        return false;
      }
      star_index = i;
    }
  }
  if (star_index == 0U || star_index + 5U != length) {
    return false;
  }

  uint16_t expected_crc = 0;
  if (!parse_crc(&line[star_index + 1U], &expected_crc) ||
      cellular_4g_protocol_crc16(line, star_index) != expected_crc) {
    return false;
  }

  if (line[star_index - 1U] == ',') {
    return false;
  }
  for (size_t i = 0; i + 1U < star_index; ++i) {
    if (line[i] == ',' && line[i + 1U] == ',') {
      return false;
    }
  }

  char body[CELLULAR_4G_PROTOCOL_MAX_LINE_LEN + 1U];
  memcpy(body, line, star_index);
  body[star_index] = '\0';

  memset(frame, 0, sizeof(*frame));
  frame->csq = 99U;
  frame->last_tx_sec = -1;
  memcpy(frame->rat, "UNKNOWN", sizeof("UNKNOWN"));

  char *save = NULL;
  char *token = strtok_r(body, ",", &save);
  if (token == NULL || strcmp(token, "$Q4G") != 0) {
    return false;
  }

  uint32_t fields = 0U;
  while ((token = strtok_r(NULL, ",", &save)) != NULL) {
    char *separator = strchr(token, '=');
    if (separator == NULL || separator == token ||
        strchr(separator + 1U, '=') != NULL || separator[1] == '\0') {
      return false;
    }
    *separator = '\0';
    const char *key = token;
    const char *value = separator + 1U;
    uint32_t bit = 0U;
    uint32_t parsed = 0U;

    if (strcmp(key, "V") == 0) {
      bit = FIELD_V;
      if (!parse_u32(value, &parsed) ||
          parsed != CELLULAR_4G_PROTOCOL_VERSION) {
        return false;
      }
    } else if (strcmp(key, "T") == 0) {
      bit = FIELD_T;
      if (strcmp(value, "STAT") == 0) {
        frame->type = CELLULAR_4G_MESSAGE_STAT;
      } else if (strcmp(value, "EVENT") == 0) {
        frame->type = CELLULAR_4G_MESSAGE_EVENT;
      } else if (strcmp(value, "ACK") == 0) {
        frame->type = CELLULAR_4G_MESSAGE_ACK;
      } else {
        return false;
      }
    } else if (strcmp(key, "SEQ") == 0) {
      bit = FIELD_SEQ;
      if (!parse_u32(value, &frame->seq)) {
        return false;
      }
    } else if (strcmp(key, "UP") == 0) {
      bit = FIELD_UP;
      if (!parse_u32(value, &frame->uptime_sec)) {
        return false;
      }
    } else if (strcmp(key, "STATE") == 0) {
      bit = FIELD_STATE;
      if (!parse_u32(value, &parsed) || parsed > 6U) {
        return false;
      }
      frame->state = (uint8_t)parsed;
    } else if (strcmp(key, "SIM") == 0) {
      bit = FIELD_SIM;
      if (!parse_u32(value, &parsed) || parsed > 3U) {
        return false;
      }
      frame->sim = (uint8_t)parsed;
    } else if (strcmp(key, "REG") == 0) {
      bit = FIELD_REG;
      if (!parse_u32(value, &parsed) || parsed > 4U) {
        return false;
      }
      frame->registration = (uint8_t)parsed;
    } else if (strcmp(key, "PDP") == 0) {
      bit = FIELD_PDP;
      if (!parse_u32(value, &parsed) || parsed > 1U) {
        return false;
      }
      frame->pdp = (uint8_t)parsed;
    } else if (strcmp(key, "MQTT") == 0) {
      bit = FIELD_MQTT;
      if (!parse_u32(value, &parsed) || parsed > 2U) {
        return false;
      }
      frame->mqtt = (uint8_t)parsed;
    } else if (strcmp(key, "CSQ") == 0) {
      bit = FIELD_CSQ;
      if (!parse_u32(value, &parsed) || (parsed > 31U && parsed != 99U)) {
        return false;
      }
      frame->csq = (uint8_t)parsed;
    } else if (strcmp(key, "RAT") == 0) {
      bit = FIELD_RAT;
      if ((strcmp(value, "LTE") != 0 && strcmp(value, "GSM") != 0 &&
           strcmp(value, "UNKNOWN") != 0) ||
          !copy_text(frame->rat, sizeof(frame->rat), value)) {
        return false;
      }
    } else if (strcmp(key, "LASTTX") == 0) {
      bit = FIELD_LASTTX;
      if (!parse_i32(value, &frame->last_tx_sec)) {
        return false;
      }
    } else if (strcmp(key, "CAUSE") == 0) {
      bit = FIELD_CAUSE;
      if (!parse_u32(value, &parsed) || !valid_cause(parsed)) {
        return false;
      }
      frame->cause = (uint8_t)parsed;
    } else if (strcmp(key, "EV") == 0) {
      bit = FIELD_EV;
      if (!valid_event_name(value) ||
          !copy_text(frame->event, sizeof(frame->event), value)) {
        return false;
      }
    } else if (strcmp(key, "REF") == 0) {
      bit = FIELD_REF;
      if (!parse_u32(value, &frame->ref_seq)) {
        return false;
      }
    } else if (strcmp(key, "RESULT") == 0) {
      bit = FIELD_RESULT;
      if (strcmp(value, "OK") != 0) {
        return false;
      }
    } else {
      continue;
    }

    if ((fields & bit) != 0U) {
      return false;
    }
    fields |= bit;
  }

  if (frame->type == CELLULAR_4G_MESSAGE_ACK) {
    const uint32_t ack_required = FIELD_V | FIELD_T | FIELD_REF | FIELD_RESULT;
    return (fields & ack_required) == ack_required;
  }

  const uint32_t common_required =
      FIELD_V | FIELD_T | FIELD_SEQ | FIELD_UP | FIELD_STATE | FIELD_CAUSE;
  if (frame->type == CELLULAR_4G_MESSAGE_STAT) {
    const uint32_t stat_required = common_required | FIELD_SIM | FIELD_REG |
                                   FIELD_PDP | FIELD_MQTT | FIELD_CSQ |
                                   FIELD_RAT | FIELD_LASTTX;
    return (fields & stat_required) == stat_required;
  }

  const uint32_t event_required = common_required | FIELD_EV;
  return (fields & event_required) == event_required;
}
