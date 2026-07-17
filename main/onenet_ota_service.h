#ifndef ONENET_OTA_SERVICE_H
#define ONENET_OTA_SERVICE_H

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  OTA_STATE_IDLE = 0,
  OTA_STATE_CHECKING,
  OTA_STATE_DOWNLOADING,
  OTA_STATE_VERIFYING,
  OTA_STATE_PENDING_REBOOT,
  OTA_STATE_TRIAL,
  OTA_STATE_FAILED,
} ota_state_t;

typedef struct {
  ota_state_t state;
  char task_id[32];
  char target_version[32];
  uint32_t downloaded_bytes;
  uint32_t total_bytes;
  int last_error;
} ota_status_t;

esp_err_t onenet_ota_boot_guard_start(void);
bool ota_get_status(ota_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
