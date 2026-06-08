#ifndef RFID_STORE_H
#define RFID_STORE_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

esp_err_t rfid_store_init(void);
esp_err_t rfid_store_enqueue(const uint8_t *tag, size_t length);
size_t rfid_store_pending_count(void);

#endif // RFID_STORE_H
