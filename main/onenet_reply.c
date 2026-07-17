#include "onenet_reply.h"

#include "cJSON.h"
#include <stdio.h>

bool onenet_reply_parse(const char *payload, size_t payload_length, char *id,
                        size_t id_size, int *code) {
  if (payload == NULL || id == NULL || id_size == 0U || code == NULL) {
    return false;
  }
  cJSON *root = cJSON_ParseWithLength(payload, payload_length);
  if (root == NULL) {
    return false;
  }

  bool valid = false;
  const cJSON *id_item = cJSON_GetObjectItemCaseSensitive(root, "id");
  const cJSON *code_item = cJSON_GetObjectItemCaseSensitive(root, "code");
  int written = -1;
  if (cJSON_IsString(id_item) && id_item->valuestring != NULL &&
      cJSON_IsNumber(code_item)) {
    written = snprintf(id, id_size, "%s", id_item->valuestring);
  } else if (cJSON_IsNumber(id_item) && cJSON_IsNumber(code_item)) {
    written = snprintf(id, id_size, "%.0f", id_item->valuedouble);
  }
  if (written > 0 && (size_t)written < id_size) {
    *code = code_item->valueint;
    valid = true;
  }
  cJSON_Delete(root);
  return valid;
}
