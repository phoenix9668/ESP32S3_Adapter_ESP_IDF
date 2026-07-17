#ifndef ESP_PARTITION_H
#define ESP_PARTITION_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#define ESP_PARTITION_TYPE_DATA 1

typedef struct {
  const char *label;
  size_t size;
} esp_partition_t;

const esp_partition_t *esp_partition_find_first(uint8_t type, uint8_t subtype,
                                                const char *label);
esp_err_t esp_partition_read(const esp_partition_t *partition, size_t offset,
                             void *destination, size_t size);
esp_err_t esp_partition_write(const esp_partition_t *partition, size_t offset,
                              const void *source, size_t size);
esp_err_t esp_partition_erase_range(const esp_partition_t *partition,
                                    size_t offset, size_t size);

#endif
