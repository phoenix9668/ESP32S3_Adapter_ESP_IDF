#ifndef ONENET_OTA_PROTOCOL_H
#define ONENET_OTA_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ONENET_OTA_VERSION_MAX_LEN 32
#define ONENET_OTA_TASK_ID_MAX_LEN 32
#define ONENET_OTA_MD5_HEX_LEN 32

typedef struct {
  char task_id[ONENET_OTA_TASK_ID_MAX_LEN];
  char target_version[ONENET_OTA_VERSION_MAX_LEN];
  char md5[ONENET_OTA_MD5_HEX_LEN + 1U];
  uint32_t size;
} onenet_ota_task_t;

bool onenet_ota_version_is_newer(const char *candidate, const char *current);
bool onenet_ota_content_range_matches(const char *header, uint32_t offset,
                                      uint32_t end, uint32_t total);
uint32_t onenet_ota_resume_offset(uint32_t persisted_offset,
                                  uint32_t total_size,
                                  uint32_t erase_size);
uint32_t onenet_ota_retry_delay_seconds(uint8_t attempt);
bool onenet_ota_parse_inform_id(const char *json, size_t json_len, char *id,
                                size_t id_size);
bool onenet_ota_parse_task(const char *json, size_t json_len,
                           const char *current_version,
                           size_t inactive_partition_size,
                           onenet_ota_task_t *task);

#ifdef __cplusplus
}
#endif

#endif
