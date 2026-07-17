#ifndef CELLULAR_SERVICE_H
#define CELLULAR_SERVICE_H

#include "esp_err.h"
#include "gnss_ml307c.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  CELLULAR_STATE_STOPPED = 0,
  CELLULAR_STATE_MODEM_DETECTING,
  CELLULAR_STATE_NETWORK_ATTACHING,
  CELLULAR_STATE_MQTT_CONNECTING,
  CELLULAR_STATE_ONLINE,
  CELLULAR_STATE_BACKOFF,
} cellular_state_t;

typedef struct {
  cellular_state_t state;
  bool config_ready;
  bool modem_ready;
  bool sim_ready;
  bool registered;
  bool data_online;
  bool mqtt_online;
  int csq;
  int last_error;
} cellular_status_t;

esp_err_t cellular_service_start(void);
bool cellular_service_config_ready(void);
bool cellular_service_get_status(cellular_status_t *status);
bool cellular_service_get_latest_gnss(gnss_fix_t *fix);

#ifdef __cplusplus
}
#endif

#endif
