#include "onenet_config.h"

#include "mbedtls/base64.h"
#include "mbedtls/md.h"
#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define ONENET_AUTH_VERSION "2018-10-31"

static bool url_unreserved(unsigned char value) {
  return isalnum(value) || value == '-' || value == '_' || value == '.' ||
         value == '~';
}

static esp_err_t url_encode(const unsigned char *input, size_t input_len,
                            char *output, size_t output_size) {
  static const char hex[] = "0123456789ABCDEF";
  size_t used = 0U;
  for (size_t i = 0U; i < input_len; ++i) {
    if (url_unreserved(input[i])) {
      if (used + 1U >= output_size) {
        return ESP_ERR_INVALID_SIZE;
      }
      output[used++] = (char)input[i];
    } else {
      if (used + 3U >= output_size) {
        return ESP_ERR_INVALID_SIZE;
      }
      output[used++] = '%';
      output[used++] = hex[input[i] >> 4U];
      output[used++] = hex[input[i] & 0x0FU];
    }
  }
  output[used] = '\0';
  return ESP_OK;
}

static esp_err_t generate_token(const onenet_config_t *config,
                                uint64_t expires_at, const char *method,
                                mbedtls_md_type_t digest_type, char *token,
                                size_t token_size) {
  if (config == NULL || token == NULL || token_size == 0U) {
    return ESP_ERR_INVALID_ARG;
  }
  char resource[192];
  int written = snprintf(resource, sizeof(resource), "products/%s/devices/%s",
                         config->product_id, config->device_name);
  if (written < 0 || (size_t)written >= sizeof(resource)) {
    return ESP_ERR_INVALID_SIZE;
  }
  char expiry[24];
  written = snprintf(expiry, sizeof(expiry), "%llu",
                     (unsigned long long)expires_at);
  if (written < 0 || (size_t)written >= sizeof(expiry)) {
    return ESP_ERR_INVALID_SIZE;
  }
  char sign_source[320];
  written = snprintf(sign_source, sizeof(sign_source), "%s\n%s\n%s\n%s",
                     expiry, method, resource, ONENET_AUTH_VERSION);
  if (written < 0 || (size_t)written >= sizeof(sign_source)) {
    return ESP_ERR_INVALID_SIZE;
  }

  unsigned char decoded_key[ONENET_DEVICE_KEY_MAX_LEN];
  size_t decoded_key_len = 0U;
  if (mbedtls_base64_decode(decoded_key, sizeof(decoded_key), &decoded_key_len,
                            (const unsigned char *)config->device_key,
                            strlen(config->device_key)) != 0) {
    return ESP_ERR_INVALID_ARG;
  }
  const mbedtls_md_info_t *info =
      mbedtls_md_info_from_type(digest_type);
  unsigned char digest[MBEDTLS_MD_MAX_SIZE];
  const size_t digest_size = info == NULL ? 0U : mbedtls_md_get_size(info);
  if (info == NULL ||
      mbedtls_md_hmac(info, decoded_key, decoded_key_len,
                      (const unsigned char *)sign_source, strlen(sign_source),
                      digest) != 0) {
    return ESP_FAIL;
  }

  unsigned char signature_base64[64];
  size_t signature_len = 0U;
  if (mbedtls_base64_encode(signature_base64, sizeof(signature_base64),
                            &signature_len, digest, digest_size) != 0) {
    return ESP_FAIL;
  }
  char encoded_resource[576];
  char encoded_signature[192];
  esp_err_t ret = url_encode((const unsigned char *)resource, strlen(resource),
                             encoded_resource, sizeof(encoded_resource));
  if (ret == ESP_OK) {
    ret = url_encode(signature_base64, signature_len, encoded_signature,
                     sizeof(encoded_signature));
  }
  if (ret != ESP_OK) {
    return ret;
  }
  written = snprintf(token, token_size,
                     "version=%s&res=%s&et=%s&method=%s&sign=%s",
                     ONENET_AUTH_VERSION, encoded_resource, expiry,
                     method, encoded_signature);
  return written < 0 || (size_t)written >= token_size ? ESP_ERR_INVALID_SIZE
                                                       : ESP_OK;
}

esp_err_t onenet_generate_mqtt_token(const onenet_config_t *config,
                                     uint64_t expires_at, char *token,
                                     size_t token_size) {
  return generate_token(config, expires_at, "sha256", MBEDTLS_MD_SHA256,
                        token, token_size);
}

esp_err_t onenet_generate_ota_token(const onenet_config_t *config,
                                    uint64_t expires_at, char *token,
                                    size_t token_size) {
  return generate_token(config, expires_at, "sha1", MBEDTLS_MD_SHA1, token,
                        token_size);
}
