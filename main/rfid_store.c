#include "rfid_store.h"

#include "app_config.h"
#include "app_protocol.h"
#include "cellular_4g.h"

#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define RFID_STORE_PARTITION_LABEL "rfid_store"
#define RFID_STORE_PARTITION_SUBTYPE 0x40
#define RFID_STORE_RECORD_MAGIC 0x52464944UL
#define RFID_STORE_RECORD_VERSION 1U
#define RFID_STORE_RECORD_SIZE 128U
#define RFID_STORE_RECORD_STATE_PENDING 0xFFFFFFFEUL
#define RFID_STORE_RECORD_STATE_DELIVERED 0xFFFFFFFCUL
#define RFID_STORE_PAYLOAD_CAPACITY 108U
#define RFID_STORE_SECTOR_SIZE 4096U

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t sequence;
  uint32_t state;
  uint16_t version;
  uint16_t length;
  uint32_t crc32;
  uint8_t payload[RFID_STORE_PAYLOAD_CAPACITY];
} rfid_store_record_t;

_Static_assert(sizeof(rfid_store_record_t) == RFID_STORE_RECORD_SIZE,
               "RFID store record must be 128 bytes");
_Static_assert(RFID_STORE_SECTOR_SIZE % RFID_STORE_RECORD_SIZE == 0U,
               "RFID records must not cross flash erase sectors");

static const char *TAG = "RFID_STORE";

static const esp_partition_t *s_partition;
static SemaphoreHandle_t s_lock;
static bool s_initialized;
static size_t s_slot_count;
static size_t s_head_slot;
static size_t s_tail_slot;
static size_t s_pending_count;
static uint32_t s_next_sequence;

static size_t record_offset(size_t slot) {
  return slot * RFID_STORE_RECORD_SIZE;
}

static bool record_is_erased(const rfid_store_record_t *record) {
  const uint8_t *bytes = (const uint8_t *)record;
  for (size_t i = 0; i < sizeof(*record); ++i) {
    if (bytes[i] != 0xFFU) {
      return false;
    }
  }
  return true;
}

static uint32_t record_crc(const rfid_store_record_t *record) {
  uint8_t crc_data[sizeof(record->sequence) + sizeof(record->version) +
                   sizeof(record->length) + RFID_STORE_PAYLOAD_CAPACITY];
  size_t offset = 0U;

  memcpy(&crc_data[offset], &record->sequence, sizeof(record->sequence));
  offset += sizeof(record->sequence);
  memcpy(&crc_data[offset], &record->version, sizeof(record->version));
  offset += sizeof(record->version);
  memcpy(&crc_data[offset], &record->length, sizeof(record->length));
  offset += sizeof(record->length);
  memcpy(&crc_data[offset], record->payload, record->length);
  offset += record->length;

  return app_protocol_crc32(crc_data, offset);
}

static bool record_payload_valid(const rfid_store_record_t *record) {
  return record->sequence != UINT32_MAX &&
         record->version == RFID_STORE_RECORD_VERSION && record->length > 0U &&
         record->length <= RFID_STORE_PAYLOAD_CAPACITY &&
         record->crc32 == record_crc(record);
}

static bool record_is_pending(const rfid_store_record_t *record) {
  return record->magic == RFID_STORE_RECORD_MAGIC &&
         record->state == RFID_STORE_RECORD_STATE_PENDING &&
         record_payload_valid(record);
}

static esp_err_t read_record(size_t slot, rfid_store_record_t *record) {
  if (slot >= s_slot_count || record == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  return esp_partition_read(s_partition, record_offset(slot), record,
                            sizeof(*record));
}

static esp_err_t scan_partition(void) {
  uint8_t *sector = malloc(RFID_STORE_SECTOR_SIZE);
  if (sector == NULL) {
    return ESP_ERR_NO_MEM;
  }

  bool have_sequence = false;
  bool have_pending = false;
  uint32_t highest_sequence = 0U;
  uint32_t lowest_pending_sequence = 0U;
  size_t highest_slot = 0U;
  size_t lowest_pending_slot = 0U;
  size_t pending_count = 0U;

  for (size_t sector_offset = 0U; sector_offset < s_partition->size;
       sector_offset += RFID_STORE_SECTOR_SIZE) {
    esp_err_t ret = esp_partition_read(s_partition, sector_offset, sector,
                                       RFID_STORE_SECTOR_SIZE);
    if (ret != ESP_OK) {
      free(sector);
      return ret;
    }

    for (size_t in_sector = 0U; in_sector < RFID_STORE_SECTOR_SIZE;
         in_sector += RFID_STORE_RECORD_SIZE) {
      const rfid_store_record_t *record =
          (const rfid_store_record_t *)&sector[in_sector];
      const size_t slot = (sector_offset + in_sector) / RFID_STORE_RECORD_SIZE;
      if (record_is_erased(record)) {
        continue;
      }

      if (record->sequence != UINT32_MAX &&
          (!have_sequence || record->sequence > highest_sequence)) {
        have_sequence = true;
        highest_sequence = record->sequence;
        highest_slot = slot;
      }

      if (!record_is_pending(record)) {
        continue;
      }

      pending_count++;
      if (!have_pending || record->sequence < lowest_pending_sequence) {
        have_pending = true;
        lowest_pending_sequence = record->sequence;
        lowest_pending_slot = slot;
      }
    }
  }
  free(sector);

  s_pending_count = pending_count;
  s_head_slot = have_pending ? lowest_pending_slot : 0U;
  s_tail_slot = have_sequence ? (highest_slot + 1U) % s_slot_count : 0U;
  s_next_sequence = have_sequence && highest_sequence < UINT32_MAX - 1U
                        ? highest_sequence + 1U
                        : 1U;

  ESP_LOGI(TAG,
           "partition=%s size=%u slots=%u pending=%u next_seq=%lu head=%u "
           "tail=%u",
           s_partition->label, (unsigned)s_partition->size,
           (unsigned)s_slot_count, (unsigned)s_pending_count,
           (unsigned long)s_next_sequence, (unsigned)s_head_slot,
           (unsigned)s_tail_slot);
  return ESP_OK;
}

static esp_err_t sector_has_pending(size_t sector_offset, bool *has_pending) {
  *has_pending = false;
  for (size_t offset = 0U; offset < RFID_STORE_SECTOR_SIZE;
       offset += RFID_STORE_RECORD_SIZE) {
    rfid_store_record_t record;
    esp_err_t ret = esp_partition_read(s_partition, sector_offset + offset,
                                       &record, sizeof(record));
    if (ret != ESP_OK) {
      return ret;
    }
    if (record_is_pending(&record)) {
      *has_pending = true;
      break;
    }
  }
  return ESP_OK;
}

static esp_err_t find_append_slot(size_t *slot_out) {
  size_t slot = s_tail_slot;
  size_t inspected = 0U;

  while (inspected < s_slot_count) {
    rfid_store_record_t record;
    esp_err_t ret = read_record(slot, &record);
    if (ret != ESP_OK) {
      return ret;
    }
    if (record_is_erased(&record)) {
      *slot_out = slot;
      return ESP_OK;
    }

    const size_t byte_offset = record_offset(slot);
    const size_t sector_offset =
        byte_offset - (byte_offset % RFID_STORE_SECTOR_SIZE);
    bool has_pending = false;
    ret = sector_has_pending(sector_offset, &has_pending);
    if (ret != ESP_OK) {
      return ret;
    }
    if (!has_pending) {
      ret = esp_partition_erase_range(s_partition, sector_offset,
                                      RFID_STORE_SECTOR_SIZE);
      if (ret != ESP_OK) {
        return ret;
      }
      *slot_out = sector_offset / RFID_STORE_RECORD_SIZE;
      return ESP_OK;
    }

    const size_t next_sector = sector_offset + RFID_STORE_SECTOR_SIZE;
    slot = next_sector >= s_partition->size
               ? 0U
               : next_sector / RFID_STORE_RECORD_SIZE;
    inspected += RFID_STORE_SECTOR_SIZE / RFID_STORE_RECORD_SIZE;
  }

  return ESP_ERR_NO_MEM;
}

static esp_err_t write_pending_record(size_t slot, uint32_t sequence,
                                      const uint8_t *tag, size_t length) {
  rfid_store_record_t record;
  memset(&record, 0xFF, sizeof(record));
  record.sequence = sequence;
  record.state = RFID_STORE_RECORD_STATE_PENDING;
  record.version = RFID_STORE_RECORD_VERSION;
  record.length = (uint16_t)length;
  memcpy(record.payload, tag, length);
  record.crc32 = record_crc(&record);

  const size_t offset = record_offset(slot);
  esp_err_t ret = esp_partition_write(
      s_partition, offset + sizeof(record.magic), &record.sequence,
      sizeof(record) - sizeof(record.magic));
  if (ret != ESP_OK) {
    return ret;
  }

  record.magic = RFID_STORE_RECORD_MAGIC;
  return esp_partition_write(s_partition, offset, &record.magic,
                             sizeof(record.magic));
}

static bool find_next_pending(size_t start_slot, size_t *slot_out) {
  for (size_t inspected = 1U; inspected <= s_slot_count; ++inspected) {
    const size_t slot = (start_slot + inspected) % s_slot_count;
    rfid_store_record_t record;
    if (read_record(slot, &record) != ESP_OK) {
      return false;
    }
    if (record_is_pending(&record)) {
      *slot_out = slot;
      return true;
    }
  }
  return false;
}

static esp_err_t peek_record(rfid_store_record_t *record) {
  if (s_pending_count == 0U) {
    return ESP_ERR_NOT_FOUND;
  }

  esp_err_t ret = read_record(s_head_slot, record);
  if (ret != ESP_OK) {
    return ret;
  }
  if (record_is_pending(record)) {
    return ESP_OK;
  }

  size_t next_slot;
  if (!find_next_pending(s_head_slot, &next_slot)) {
    s_pending_count = 0U;
    return ESP_ERR_NOT_FOUND;
  }
  s_head_slot = next_slot;
  return read_record(s_head_slot, record);
}

static esp_err_t mark_delivered(uint32_t sequence) {
  rfid_store_record_t record;
  esp_err_t ret = peek_record(&record);
  if (ret != ESP_OK) {
    return ret;
  }
  if (record.sequence != sequence) {
    return ESP_ERR_INVALID_STATE;
  }

  const uint32_t state = RFID_STORE_RECORD_STATE_DELIVERED;
  ret = esp_partition_write(s_partition,
                            record_offset(s_head_slot) +
                                offsetof(rfid_store_record_t, state),
                            &state, sizeof(state));
  if (ret != ESP_OK) {
    return ret;
  }

  const size_t delivered_slot = s_head_slot;
  s_pending_count--;
  if (s_pending_count > 0U) {
    size_t next_slot;
    if (!find_next_pending(delivered_slot, &next_slot)) {
      s_pending_count = 0U;
    } else {
      s_head_slot = next_slot;
    }
  }
  return ESP_OK;
}

static bool cloud_online(void) {
  cellular_4g_status_t status;
  return cellular_4g_get_status(&status) && status.has_complete_status &&
         !status.communication_stale && status.latest.state == 5U &&
         status.latest.pdp == 1U && status.latest.mqtt == 2U;
}

static void rfid_replay_task(void *arg) {
  (void)arg;

  while (true) {
    rfid_store_record_t record;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const esp_err_t peek_ret = peek_record(&record);
    xSemaphoreGive(s_lock);
    if (peek_ret == ESP_ERR_NOT_FOUND) {
      vTaskDelay(pdMS_TO_TICKS(APP_RFID_REPLAY_IDLE_MS));
      continue;
    }
    if (peek_ret != ESP_OK) {
      ESP_LOGE(TAG, "failed to read pending RFID record: %s",
               esp_err_to_name(peek_ret));
      vTaskDelay(pdMS_TO_TICKS(APP_RFID_REPLAY_RETRY_MS));
      continue;
    }

    uint32_t ack_sequence;
    bool already_acked = false;
    while (cellular_4g_wait_for_rfid_ack(&ack_sequence, 0U)) {
      if (ack_sequence == record.sequence) {
        already_acked = true;
      }
    }

    if (!already_acked && !cloud_online()) {
      vTaskDelay(pdMS_TO_TICKS(APP_RFID_REPLAY_IDLE_MS));
      continue;
    }

    if (!already_acked) {
      const esp_err_t send_ret =
          cellular_4g_send_rfid(record.sequence, record.payload, record.length);
      if (send_ret != ESP_OK) {
        ESP_LOGW(TAG, "RFID seq=%lu send failed: %s",
                 (unsigned long)record.sequence, esp_err_to_name(send_ret));
        vTaskDelay(pdMS_TO_TICKS(APP_RFID_REPLAY_RETRY_MS));
        continue;
      }

      ESP_LOGI(TAG, "RFID seq=%lu sent, awaiting cloud ACK",
               (unsigned long)record.sequence);
      const TickType_t started = xTaskGetTickCount();
      const TickType_t timeout = pdMS_TO_TICKS(APP_RFID_ACK_TIMEOUT_MS);
      while (xTaskGetTickCount() - started < timeout) {
        const TickType_t remaining = timeout - (xTaskGetTickCount() - started);
        if (!cellular_4g_wait_for_rfid_ack(&ack_sequence, remaining)) {
          break;
        }
        if (ack_sequence == record.sequence) {
          already_acked = true;
          break;
        }
      }
    }

    if (!already_acked) {
      ESP_LOGW(TAG, "RFID seq=%lu ACK timeout; record retained",
               (unsigned long)record.sequence);
      vTaskDelay(pdMS_TO_TICKS(APP_RFID_REPLAY_RETRY_MS));
      continue;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    const esp_err_t delivered_ret = mark_delivered(record.sequence);
    const size_t remaining_count = s_pending_count;
    xSemaphoreGive(s_lock);
    if (delivered_ret != ESP_OK) {
      ESP_LOGE(TAG, "failed to mark RFID seq=%lu delivered: %s",
               (unsigned long)record.sequence, esp_err_to_name(delivered_ret));
      vTaskDelay(pdMS_TO_TICKS(APP_RFID_REPLAY_RETRY_MS));
      continue;
    }

    ESP_LOGI(TAG, "RFID seq=%lu cloud ACK received, pending=%u",
             (unsigned long)record.sequence, (unsigned)remaining_count);
    vTaskDelay(pdMS_TO_TICKS(APP_RFID_REPLAY_GAP_MS));
  }
}

esp_err_t rfid_store_init(void) {
  if (s_initialized) {
    return ESP_OK;
  }

  s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                         RFID_STORE_PARTITION_SUBTYPE,
                                         RFID_STORE_PARTITION_LABEL);
  if (s_partition == NULL || s_partition->size < RFID_STORE_SECTOR_SIZE ||
      s_partition->size % RFID_STORE_RECORD_SIZE != 0U) {
    return ESP_ERR_NOT_FOUND;
  }

  s_slot_count = s_partition->size / RFID_STORE_RECORD_SIZE;
  s_lock = xSemaphoreCreateMutex();
  if (s_lock == NULL) {
    return ESP_ERR_NO_MEM;
  }

  esp_err_t ret = scan_partition();
  if (ret != ESP_OK) {
    vSemaphoreDelete(s_lock);
    s_lock = NULL;
    return ret;
  }

  const BaseType_t task_created =
      xTaskCreate(rfid_replay_task, "rfid_replay", APP_TASK_STACK_DEFAULT, NULL,
                  tskIDLE_PRIORITY + 4, NULL);
  if (task_created != pdPASS) {
    vSemaphoreDelete(s_lock);
    s_lock = NULL;
    return ESP_ERR_NO_MEM;
  }

  s_initialized = true;
  return ESP_OK;
}

esp_err_t rfid_store_enqueue(const uint8_t *tag, size_t length) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (tag == NULL || length == 0U || length > APP_RFID_TAG_MAX_LEN) {
    return ESP_ERR_INVALID_ARG;
  }
  for (size_t i = 0U; i < length; ++i) {
    const bool digit = tag[i] >= '0' && tag[i] <= '9';
    const bool upper_hex = tag[i] >= 'A' && tag[i] <= 'F';
    if (!digit && !upper_hex) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  xSemaphoreTake(s_lock, portMAX_DELAY);
  size_t slot;
  esp_err_t ret = find_append_slot(&slot);
  if (ret == ESP_OK) {
    const uint32_t sequence = s_next_sequence;
    ret = write_pending_record(slot, sequence, tag, length);
    if (ret == ESP_OK) {
      if (s_pending_count == 0U) {
        s_head_slot = slot;
      }
      s_pending_count++;
      s_tail_slot = (slot + 1U) % s_slot_count;
      s_next_sequence = sequence < UINT32_MAX - 1U ? sequence + 1U : 1U;
      ESP_LOGI(TAG, "queued RFID seq=%lu len=%u pending=%u",
               (unsigned long)sequence, (unsigned)length,
               (unsigned)s_pending_count);
    }
  }
  xSemaphoreGive(s_lock);

  if (ret == ESP_ERR_NO_MEM) {
    ESP_LOGE(TAG, "RFID flash queue full; newest tag rejected");
  }
  return ret;
}

size_t rfid_store_pending_count(void) {
  if (!s_initialized) {
    return 0U;
  }
  xSemaphoreTake(s_lock, portMAX_DELAY);
  const size_t count = s_pending_count;
  xSemaphoreGive(s_lock);
  return count;
}
