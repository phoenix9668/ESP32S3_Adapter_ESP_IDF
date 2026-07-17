#include "onenet_config.h"

#include "nvs.h"
#include <string.h>

#define ONENET_NVS_NAMESPACE "onenet"

static esp_err_t read_required_string(nvs_handle_t handle, const char *key,
                                      char *output, size_t output_size) {
  size_t required = output_size;
  esp_err_t ret = nvs_get_str(handle, key, output, &required);
  if (ret != ESP_OK || required <= 1U || required > output_size) {
    return ret == ESP_OK ? ESP_ERR_INVALID_SIZE : ret;
  }
  return ESP_OK;
}

esp_err_t onenet_config_load(onenet_config_t *config) {
  if (config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(config, 0, sizeof(*config));
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(ONENET_NVS_NAMESPACE, NVS_READONLY, &handle);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = read_required_string(handle, "broker_host", config->broker_host,
                             sizeof(config->broker_host));
  if (ret == ESP_OK) {
    ret = nvs_get_u16(handle, "broker_port", &config->broker_port);
  }
  if (ret == ESP_OK) {
    ret = read_required_string(handle, "product_id", config->product_id,
                               sizeof(config->product_id));
  }
  if (ret == ESP_OK) {
    ret = read_required_string(handle, "device_name", config->device_name,
                               sizeof(config->device_name));
  }
  if (ret == ESP_OK) {
    ret = read_required_string(handle, "device_key", config->device_key,
                               sizeof(config->device_key));
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
      ret = read_required_string(handle, "access_key", config->device_key,
                                 sizeof(config->device_key));
    }
  }
  if (ret == ESP_OK) {
    ret = nvs_get_u32(handle, "token_ttl", &config->token_ttl);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
      uint64_t legacy_expiry = 0U;
      const esp_err_t legacy_ret =
          nvs_get_u64(handle, "token_expiry", &legacy_expiry);
      if (legacy_ret == ESP_OK || legacy_ret == ESP_ERR_NVS_NOT_FOUND) {
        config->token_ttl = ONENET_DEFAULT_TOKEN_TTL_SECONDS;
        ret = ESP_OK;
      } else {
        ret = legacy_ret;
      }
    }
  }
  nvs_close(handle);
  if (ret == ESP_OK &&
      (config->broker_port == 0U || config->token_ttl == 0U ||
       config->token_ttl > 7U * 24U * 60U * 60U)) {
    ret = ESP_ERR_INVALID_ARG;
  }
  return ret;
}
