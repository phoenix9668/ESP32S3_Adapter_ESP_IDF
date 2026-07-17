#ifndef RFID_STORE_H
#define RFID_STORE_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t sequence;
  size_t length;
  uint8_t payload[64];
} rfid_store_item_t;

esp_err_t rfid_store_init(void);
esp_err_t rfid_store_enqueue(const uint8_t *tag, size_t length);
esp_err_t rfid_store_peek_oldest(rfid_store_item_t *item);
esp_err_t rfid_store_mark_delivered(uint32_t sequence);
size_t rfid_store_pending_count(void);

#ifdef RFID_STORE_HOST_TEST
void rfid_store_test_reset_runtime(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // RFID_STORE_H
