#include "onenet_ota_protocol.h"

#include "cJSON.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool copy_json_string(const cJSON *item, char *output,
                             size_t output_size) {
  if (!cJSON_IsString(item) || item->valuestring == NULL) {
    return false;
  }
  const size_t length = strlen(item->valuestring);
  if (length == 0U || length >= output_size) {
    return false;
  }
  memcpy(output, item->valuestring, length + 1U);
  return true;
}

static bool parse_version_number(const char **cursor, uint32_t *value) {
  if (!isdigit((unsigned char)**cursor)) {
    return false;
  }
  uint64_t parsed = 0U;
  while (isdigit((unsigned char)**cursor)) {
    parsed = parsed * 10U + (uint64_t)(**cursor - '0');
    if (parsed > UINT32_MAX) {
      return false;
    }
    ++*cursor;
  }
  *value = (uint32_t)parsed;
  return true;
}

bool onenet_ota_version_is_newer(const char *candidate, const char *current) {
  if (candidate == NULL || current == NULL || *candidate == '\0' ||
      *current == '\0' || strlen(candidate) >= ONENET_OTA_VERSION_MAX_LEN ||
      strlen(current) >= ONENET_OTA_VERSION_MAX_LEN) {
    return false;
  }
  const char *left = candidate;
  const char *right = current;
  for (unsigned component = 0U; component < 4U; ++component) {
    uint32_t left_value = 0U;
    uint32_t right_value = 0U;
    if (!parse_version_number(&left, &left_value) ||
        !parse_version_number(&right, &right_value)) {
      return strcmp(candidate, current) > 0;
    }
    if (left_value != right_value) {
      return left_value > right_value;
    }
    if (*left == '.' && *right == '.') {
      ++left;
      ++right;
      continue;
    }
    break;
  }
  if (*left == '\0' && *right == '\0') {
    return false;
  }
  if (*left == '\0') {
    return *right == '-';
  }
  if (*right == '\0') {
    return *left != '-';
  }
  if (*left == '-' && *right != '-') {
    return false;
  }
  if (*right == '-' && *left != '-') {
    return true;
  }
  return strcmp(left, right) > 0;
}

bool onenet_ota_content_range_matches(const char *header, uint32_t offset,
                                      uint32_t end, uint32_t total) {
  if (header == NULL || *header == '\0') {
    return false;
  }
  unsigned long parsed_offset = 0U;
  unsigned long parsed_end = 0U;
  unsigned long parsed_total = 0U;
  char trailing = '\0';
  return sscanf(header, "bytes %lu-%lu/%lu%c", &parsed_offset, &parsed_end,
                &parsed_total, &trailing) == 3 &&
         parsed_offset == offset && parsed_end == end &&
         parsed_total == total;
}

uint32_t onenet_ota_resume_offset(uint32_t persisted_offset,
                                  uint32_t total_size,
                                  uint32_t erase_size) {
  if (erase_size == 0U || persisted_offset >= total_size) {
    return 0U;
  }
  return persisted_offset - persisted_offset % erase_size;
}

uint32_t onenet_ota_retry_delay_seconds(uint8_t attempt) {
  static const uint32_t delays[] = {5U, 15U, 30U, 60U, 300U};
  const size_t count = sizeof(delays) / sizeof(delays[0]);
  const size_t index = attempt < count ? attempt : count - 1U;
  return delays[index];
}

bool onenet_ota_parse_inform_id(const char *json, size_t json_len, char *id,
                                size_t id_size) {
  if (json == NULL || id == NULL || id_size == 0U) {
    return false;
  }
  cJSON *root = cJSON_ParseWithLength(json, json_len);
  if (root == NULL) {
    return false;
  }
  const bool valid = copy_json_string(cJSON_GetObjectItemCaseSensitive(root, "id"),
                                      id, id_size);
  cJSON_Delete(root);
  return valid;
}

static bool copy_task_id(const cJSON *item, char *output,
                         size_t output_size) {
  if (copy_json_string(item, output, output_size)) {
    return true;
  }
  if (!cJSON_IsNumber(item) || item->valuedouble < 0.0 ||
      item->valuedouble > 4294967295.0) {
    return false;
  }
  const int written = snprintf(output, output_size, "%.0f", item->valuedouble);
  return written > 0 && (size_t)written < output_size;
}

bool onenet_ota_parse_task(const char *json, size_t json_len,
                           const char *current_version,
                           size_t inactive_partition_size,
                           onenet_ota_task_t *task) {
  if (json == NULL || current_version == NULL || task == NULL) {
    return false;
  }
  memset(task, 0, sizeof(*task));
  cJSON *root = cJSON_ParseWithLength(json, json_len);
  if (root == NULL) {
    return false;
  }
  const cJSON *code = cJSON_GetObjectItemCaseSensitive(root, "code");
  const cJSON *data = cJSON_GetObjectItemCaseSensitive(root, "data");
  const cJSON *package_type =
      cJSON_IsObject(data) ? cJSON_GetObjectItemCaseSensitive(data, "type") : NULL;
  const cJSON *size =
      cJSON_IsObject(data) ? cJSON_GetObjectItemCaseSensitive(data, "size") : NULL;
  const bool type_valid = package_type == NULL ||
                          (cJSON_IsNumber(package_type) && package_type->valueint == 1);
  bool valid = cJSON_IsNumber(code) && code->valueint == 0 &&
               cJSON_IsObject(data) && type_valid && cJSON_IsNumber(size) &&
               size->valuedouble > 0.0 &&
               size->valuedouble <= (double)inactive_partition_size &&
               size->valuedouble <= (double)UINT32_MAX;
  if (valid) {
    valid = copy_task_id(cJSON_GetObjectItemCaseSensitive(data, "tid"),
                         task->task_id, sizeof(task->task_id)) &&
            copy_json_string(cJSON_GetObjectItemCaseSensitive(data, "target"),
                             task->target_version,
                             sizeof(task->target_version)) &&
            copy_json_string(cJSON_GetObjectItemCaseSensitive(data, "md5"),
                             task->md5, sizeof(task->md5));
  }
  if (valid) {
    for (size_t i = 0U; i < ONENET_OTA_MD5_HEX_LEN; ++i) {
      if (!isxdigit((unsigned char)task->md5[i])) {
        valid = false;
        break;
      }
      task->md5[i] = (char)tolower((unsigned char)task->md5[i]);
    }
    valid = valid && strlen(task->md5) == ONENET_OTA_MD5_HEX_LEN &&
            onenet_ota_version_is_newer(task->target_version,
                                        current_version);
  }
  if (valid) {
    task->size = (uint32_t)size->valuedouble;
  } else {
    memset(task, 0, sizeof(*task));
  }
  cJSON_Delete(root);
  return valid;
}
