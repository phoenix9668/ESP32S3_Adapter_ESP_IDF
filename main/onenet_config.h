#ifndef ONENET_CONFIG_H
#define ONENET_CONFIG_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ONENET_BROKER_HOST_MAX_LEN 96
#define ONENET_PRODUCT_ID_MAX_LEN 48
#define ONENET_DEVICE_NAME_MAX_LEN 64
#define ONENET_DEVICE_KEY_MAX_LEN 192
#define ONENET_TOKEN_MAX_LEN 1024
#define ONENET_DEFAULT_TOKEN_TTL_SECONDS 3600U

typedef struct {
  char broker_host[ONENET_BROKER_HOST_MAX_LEN];
  uint16_t broker_port;
  char product_id[ONENET_PRODUCT_ID_MAX_LEN];
  char device_name[ONENET_DEVICE_NAME_MAX_LEN];
  char device_key[ONENET_DEVICE_KEY_MAX_LEN];
  uint32_t token_ttl;
} onenet_config_t;

esp_err_t onenet_config_load(onenet_config_t *config);
esp_err_t onenet_generate_mqtt_token(const onenet_config_t *config,
                                     uint64_t expires_at, char *token,
                                     size_t token_size);
esp_err_t onenet_generate_ota_token(const onenet_config_t *config,
                                    uint64_t expires_at, char *token,
                                    size_t token_size);

#ifdef __cplusplus
}
#endif

#endif
