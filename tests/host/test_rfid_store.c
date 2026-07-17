#include "rfid_store.h"

#include "esp_partition.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define MOCK_PARTITION_SIZE 8192U

static uint8_t s_flash[MOCK_PARTITION_SIZE];
static const esp_partition_t s_partition = {
    .label = "rfid_store",
    .size = MOCK_PARTITION_SIZE,
};

const esp_partition_t *esp_partition_find_first(uint8_t type, uint8_t subtype,
                                                const char *label) {
  return type == ESP_PARTITION_TYPE_DATA && subtype == 0x40U &&
                 strcmp(label, s_partition.label) == 0
             ? &s_partition
             : NULL;
}

esp_err_t esp_partition_read(const esp_partition_t *partition, size_t offset,
                             void *destination, size_t size) {
  if (partition != &s_partition || destination == NULL ||
      offset + size > sizeof(s_flash)) {
    return ESP_ERR_INVALID_ARG;
  }
  memcpy(destination, s_flash + offset, size);
  return ESP_OK;
}

esp_err_t esp_partition_write(const esp_partition_t *partition, size_t offset,
                              const void *source, size_t size) {
  if (partition != &s_partition || source == NULL ||
      offset + size > sizeof(s_flash)) {
    return ESP_ERR_INVALID_ARG;
  }
  const uint8_t *bytes = source;
  for (size_t i = 0U; i < size; ++i) {
    if ((s_flash[offset + i] & bytes[i]) != bytes[i]) {
      return ESP_FAIL;
    }
    s_flash[offset + i] &= bytes[i];
  }
  return ESP_OK;
}

esp_err_t esp_partition_erase_range(const esp_partition_t *partition,
                                    size_t offset, size_t size) {
  if (partition != &s_partition || offset + size > sizeof(s_flash) ||
      offset % 4096U != 0U || size % 4096U != 0U) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(s_flash + offset, 0xFF, size);
  return ESP_OK;
}

static void reboot_store(void) {
  rfid_store_test_reset_runtime();
  assert(rfid_store_init() == ESP_OK);
}

int main(void) {
  memset(s_flash, 0xFF, sizeof(s_flash));
  reboot_store();
  const uint8_t first[] = "0123456789ABCDEF";
  const uint8_t second[] = "FEDCBA9876543210";
  assert(rfid_store_enqueue(first, sizeof(first) - 1U) == ESP_OK);
  assert(rfid_store_enqueue(second, sizeof(second) - 1U) == ESP_OK);
  assert(rfid_store_pending_count() == 2U);

  rfid_store_item_t item;
  assert(rfid_store_peek_oldest(&item) == ESP_OK);
  assert(item.sequence == 1U);
  assert(item.length == 16U);
  assert(memcmp(item.payload, first, 16U) == 0);

  reboot_store();
  assert(rfid_store_pending_count() == 2U);
  assert(rfid_store_peek_oldest(&item) == ESP_OK);
  assert(item.sequence == 1U);
  assert(rfid_store_mark_delivered(2U) == ESP_ERR_INVALID_STATE);
  assert(rfid_store_pending_count() == 2U);
  assert(rfid_store_mark_delivered(1U) == ESP_OK);

  reboot_store();
  assert(rfid_store_pending_count() == 1U);
  assert(rfid_store_peek_oldest(&item) == ESP_OK);
  assert(item.sequence == 2U);
  assert(memcmp(item.payload, second, 16U) == 0);
  assert(rfid_store_mark_delivered(2U) == ESP_OK);
  reboot_store();
  assert(rfid_store_pending_count() == 0U);
  puts("rfid_store_tests: all tests passed");
  return 0;
}
