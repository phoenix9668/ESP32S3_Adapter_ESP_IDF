#include "onenet_ota_service.hpp"

#include "cellular_service.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "http.h"
#include "mbedtls/md5.h"
#include "nvs.h"
#include "onenet_ota_protocol.h"
#include "onenet_ota_service.h"
#include "cJSON.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <time.h>

namespace {

constexpr const char *kOtaBaseUrl = "https://iot-api.heclouds.com";
constexpr const char *kOtaNamespace = "ota";
constexpr const char *kExpectedProjectName = "ESP32S3_Adapter_ESP_IDF";
constexpr uint32_t kOtaPollSeconds = 6U * 60U * 60U;
constexpr uint32_t kHttpTimeoutMs = 30000U;
constexpr size_t kHttpRangeSize = 64U * 1024U;
constexpr size_t kFlashWriteSize = 4096U;
constexpr uint32_t kBootValidationDelayMs = 60U * 1000U;
constexpr uint8_t kPersistDownloading = 1U;
constexpr uint8_t kPersistRebootPending = 2U;
constexpr uint8_t kPersistResultPending = 3U;

const char *TAG = "ONENET_OTA";
portMUX_TYPE s_ota_status_lock = portMUX_INITIALIZER_UNLOCKED;
ota_status_t s_ota_status = {};
bool s_boot_guard_started;

struct PersistedState {
  uint8_t state = 0U;
  onenet_ota_task_t task = {};
  uint32_t offset = 0U;
  char partition[17] = {};
  int32_t result = 0;
};

void set_ota_status(ota_state_t state, const onenet_ota_task_t *task,
                    uint32_t downloaded, int error) {
  taskENTER_CRITICAL(&s_ota_status_lock);
  s_ota_status.state = state;
  s_ota_status.downloaded_bytes = downloaded;
  s_ota_status.last_error = error;
  if (task != nullptr) {
    snprintf(s_ota_status.task_id, sizeof(s_ota_status.task_id), "%s",
             task->task_id);
    snprintf(s_ota_status.target_version,
             sizeof(s_ota_status.target_version), "%s",
             task->target_version);
    s_ota_status.total_bytes = task->size;
  } else if (state == OTA_STATE_IDLE) {
    s_ota_status.task_id[0] = '\0';
    s_ota_status.target_version[0] = '\0';
    s_ota_status.total_bytes = 0U;
  }
  taskEXIT_CRITICAL(&s_ota_status_lock);
}

esp_err_t read_string(nvs_handle_t handle, const char *key, char *output,
                      size_t output_size) {
  size_t required = output_size;
  const esp_err_t ret = nvs_get_str(handle, key, output, &required);
  if (ret != ESP_OK) {
    return ret;
  }
  return required > 1U && required <= output_size ? ESP_OK
                                                   : ESP_ERR_INVALID_SIZE;
}

esp_err_t load_persisted_state(PersistedState *state) {
  if (state == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  *state = PersistedState{};
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(kOtaNamespace, NVS_READONLY, &handle);
  if (ret != ESP_OK) {
    return ret;
  }
  ret = nvs_get_u8(handle, "state", &state->state);
  if (ret == ESP_OK) {
    ret = read_string(handle, "tid", state->task.task_id,
                      sizeof(state->task.task_id));
  }
  if (ret == ESP_OK) {
    ret = read_string(handle, "target", state->task.target_version,
                      sizeof(state->task.target_version));
  }
  if (ret == ESP_OK) {
    ret = read_string(handle, "md5", state->task.md5,
                      sizeof(state->task.md5));
  }
  if (ret == ESP_OK) {
    ret = nvs_get_u32(handle, "size", &state->task.size);
  }
  if (ret == ESP_OK) {
    ret = nvs_get_u32(handle, "offset", &state->offset);
  }
  if (ret == ESP_OK) {
    ret = read_string(handle, "part", state->partition,
                      sizeof(state->partition));
  }
  if (ret == ESP_OK && state->state == kPersistResultPending) {
    ret = nvs_get_i32(handle, "result", &state->result);
  }
  nvs_close(handle);
  return ret;
}

esp_err_t save_persisted_state(const PersistedState &state) {
  nvs_handle_t handle;
  esp_err_t ret = nvs_open(kOtaNamespace, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    return ret;
  }
  if ((ret = nvs_set_u8(handle, "state", state.state)) == ESP_OK &&
      (ret = nvs_set_str(handle, "tid", state.task.task_id)) == ESP_OK &&
      (ret = nvs_set_str(handle, "target", state.task.target_version)) ==
          ESP_OK &&
      (ret = nvs_set_str(handle, "md5", state.task.md5)) == ESP_OK &&
      (ret = nvs_set_u32(handle, "size", state.task.size)) == ESP_OK &&
      (ret = nvs_set_u32(handle, "offset", state.offset)) == ESP_OK &&
      (ret = nvs_set_str(handle, "part", state.partition)) == ESP_OK) {
    if (state.state == kPersistResultPending) {
      ret = nvs_set_i32(handle, "result", state.result);
    } else {
      nvs_erase_key(handle, "result");
    }
  }
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }
  nvs_close(handle);
  return ret;
}

void clear_persisted_state() {
  nvs_handle_t handle;
  if (nvs_open(kOtaNamespace, NVS_READWRITE, &handle) == ESP_OK) {
    nvs_erase_all(handle);
    nvs_commit(handle);
    nvs_close(handle);
  }
}

std::string read_http_body(Http *http) {
  return http == nullptr ? std::string{} : http->ReadAll();
}

bool make_ota_authorization(const onenet_config_t &config,
                            std::string *authorization) {
  const time_t now = time(nullptr);
  if (authorization == nullptr || now < 1704067200) {
    return false;
  }
  char token[ONENET_TOKEN_MAX_LEN];
  const esp_err_t ret = onenet_generate_ota_token(
      &config, (uint64_t)now + config.token_ttl, token, sizeof(token));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "failed to generate device OTA authorization: %s",
             esp_err_to_name(ret));
    return false;
  }
  *authorization = token;
  return true;
}

bool http_json(AtModem *modem, const onenet_config_t &config,
               const char *method, const std::string &path,
               const std::string *request_body, std::string *response_body) {
  if (modem == nullptr || response_body == nullptr) {
    return false;
  }
  std::string authorization;
  if (!make_ota_authorization(config, &authorization)) {
    return false;
  }
  std::unique_ptr<Http> http = modem->CreateHttp();
  if (!http) {
    return false;
  }
  http->SetTimeout(kHttpTimeoutMs);
  http->SetHeader("Authorization", authorization);
  if (request_body != nullptr) {
    http->SetHeader("Content-Type", "application/json");
    http->SetContent(std::string(*request_body));
  }
  const bool opened = http->Open(method, kOtaBaseUrl + path);
  const int status = opened ? http->GetStatusCode() : -1;
  if (!opened || status < 200 || status >= 300) {
    ESP_LOGW(TAG, "OneNET OTA HTTP %s failed: status=%d modem_error=%d",
             method, status, http->GetLastError());
    http->Close();
    return false;
  }
  *response_body = read_http_body(http.get());
  http->Close();
  return !response_body->empty();
}

bool response_code_ok(const std::string &body) {
  cJSON *root = cJSON_ParseWithLength(body.data(), body.size());
  if (root == nullptr) {
    return false;
  }
  const cJSON *code = cJSON_GetObjectItemCaseSensitive(root, "code");
  const bool ok = cJSON_IsNumber(code) && code->valueint == 0;
  cJSON_Delete(root);
  return ok;
}

bool task_is_active(AtModem *modem, const onenet_config_t &config,
                    const char *task_id) {
  char path[256];
  const int length = snprintf(path, sizeof(path),
                              "/fuse-ota/%s/%s/%s/check",
                              config.product_id, config.device_name, task_id);
  if (length <= 0 || (size_t)length >= sizeof(path)) {
    return false;
  }
  std::string response;
  const bool active =
      http_json(modem, config, "GET", path, nullptr, &response) &&
      response_code_ok(response);
  if (!active) {
    ESP_LOGW(TAG, "OneNET OTA task is no longer active: id=%s", task_id);
  }
  return active;
}

bool report_status(AtModem *modem, const onenet_config_t &config,
                   const char *task_id, int step) {
  char path[256];
  char payload[48];
  const int path_length = snprintf(
      path, sizeof(path), "/fuse-ota/%s/%s/%s/status", config.product_id,
      config.device_name, task_id);
  const int payload_length =
      snprintf(payload, sizeof(payload), "{\"step\":%d}", step);
  if (path_length <= 0 || (size_t)path_length >= sizeof(path) ||
      payload_length <= 0 || (size_t)payload_length >= sizeof(payload)) {
    return false;
  }
  const std::string request(payload, (size_t)payload_length);
  std::string response;
  const bool ok = http_json(modem, config, "POST", path, &request, &response) &&
                  response_code_ok(response);
  if (ok) {
    ESP_LOGI(TAG, "OneNET OTA status accepted: step=%d", step);
  } else {
    ESP_LOGW(TAG, "OneNET OTA status rejected: step=%d", step);
  }
  return ok;
}

bool partition_md5_matches(const esp_partition_t *partition, uint32_t size,
                           const char *expected_hex) {
  if (partition == nullptr || expected_hex == nullptr) {
    return false;
  }
  mbedtls_md5_context context;
  mbedtls_md5_init(&context);
  if (mbedtls_md5_starts(&context) != 0) {
    mbedtls_md5_free(&context);
    return false;
  }
  uint8_t buffer[kFlashWriteSize];
  uint32_t offset = 0U;
  bool ok = true;
  while (offset < size) {
    const size_t chunk = std::min(sizeof(buffer), (size_t)(size - offset));
    if (esp_partition_read(partition, offset, buffer, chunk) != ESP_OK ||
        mbedtls_md5_update(&context, buffer, chunk) != 0) {
      ok = false;
      break;
    }
    offset += (uint32_t)chunk;
  }
  unsigned char digest[16];
  if (!ok || mbedtls_md5_finish(&context, digest) != 0) {
    mbedtls_md5_free(&context);
    return false;
  }
  mbedtls_md5_free(&context);
  char actual[33];
  for (size_t i = 0U; i < sizeof(digest); ++i) {
    snprintf(actual + i * 2U, 3U, "%02x", digest[i]);
  }
  return strcmp(actual, expected_hex) == 0;
}

bool validate_download_description(const esp_partition_t *partition,
                                   const onenet_ota_task_t &task) {
  esp_app_desc_t description = {};
  if (esp_ota_get_partition_description(partition, &description) != ESP_OK) {
    return false;
  }
  if (strcmp(description.project_name, kExpectedProjectName) != 0 ||
      strcmp(description.version, task.target_version) != 0) {
    ESP_LOGE(TAG,
             "OTA image identity mismatch: project=%s version=%s expected=%s",
             description.project_name, description.version,
             task.target_version);
    return false;
  }
  return true;
}

bool open_download(AtModem *modem, const onenet_config_t &config,
                   const onenet_ota_task_t &task, uint32_t offset,
                   uint32_t end, std::unique_ptr<Http> *output) {
  std::string authorization;
  if (output == nullptr || !make_ota_authorization(config, &authorization)) {
    return false;
  }
  char path[256];
  const int length = snprintf(path, sizeof(path),
                              "/fuse-ota/%s/%s/%s/download",
                              config.product_id, config.device_name,
                              task.task_id);
  if (length <= 0 || (size_t)length >= sizeof(path)) {
    return false;
  }
  auto http = modem->CreateHttp();
  if (!http) {
    return false;
  }
  char range[64];
  snprintf(range, sizeof(range), "bytes=%lu-%lu", (unsigned long)offset,
           (unsigned long)end);
  http->SetTimeout(kHttpTimeoutMs);
  http->SetHeader("Authorization", authorization);
  http->SetHeader("Range", range);
  if (!http->Open("GET", std::string(kOtaBaseUrl) + path)) {
    return false;
  }
  const int status = http->GetStatusCode();
  if (status != 206 && !(status == 200 && offset == 0U)) {
    ESP_LOGW(TAG, "OTA Range request rejected: status=%d offset=%lu", status,
             (unsigned long)offset);
    http->Close();
    return false;
  }
  *output = std::move(http);
  return true;
}

bool download_task(AtModem *modem, const onenet_config_t &config,
                   const onenet_ota_task_t &task) {
  const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
  if (partition == nullptr || task.size > partition->size) {
    report_status(modem, config, task.task_id, 102);
    return false;
  }
  if (!task_is_active(modem, config, task.task_id)) {
    return false;
  }

  PersistedState persisted;
  bool resume = load_persisted_state(&persisted) == ESP_OK &&
                persisted.state == kPersistDownloading &&
                strcmp(persisted.task.task_id, task.task_id) == 0 &&
                strcmp(persisted.task.target_version, task.target_version) == 0 &&
                strcmp(persisted.task.md5, task.md5) == 0 &&
                persisted.task.size == task.size &&
                strcmp(persisted.partition, partition->label) == 0 &&
                persisted.offset <= task.size;
  uint32_t offset = resume ? persisted.offset : 0U;
  esp_ota_handle_t handle = 0U;
  esp_err_t ret = resume ? esp_ota_resume(partition, 0U, offset, &handle)
                         : esp_ota_begin(partition, task.size, &handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "OTA partition prepare failed: %s", esp_err_to_name(ret));
    report_status(modem, config, task.task_id, 102);
    return false;
  }

  persisted = PersistedState{};
  persisted.state = kPersistDownloading;
  persisted.task = task;
  persisted.offset = offset;
  snprintf(persisted.partition, sizeof(persisted.partition), "%s",
           partition->label);
  if (save_persisted_state(persisted) != ESP_OK) {
    esp_ota_abort(handle);
    return false;
  }

  set_ota_status(OTA_STATE_DOWNLOADING, &task, offset, 0);
  report_status(modem, config, task.task_id, 10);
  unsigned last_progress = offset == 0U ? 0U : (unsigned)(offset * 100U / task.size);
  uint8_t buffer[kFlashWriteSize];

  while (offset < task.size) {
    const uint32_t range_end =
        std::min(task.size - 1U,
                 offset + (uint32_t)kHttpRangeSize - 1U);
    const uint32_t expected = range_end - offset + 1U;
    std::unique_ptr<Http> http;
    if (!open_download(modem, config, task, offset, range_end, &http)) {
      esp_ota_abort(handle);
      set_ota_status(OTA_STATE_FAILED, &task, offset, ESP_ERR_TIMEOUT);
      ESP_LOGW(TAG,
               "OTA download paused at offset=%lu; retaining checkpoint for "
               "retry",
               (unsigned long)offset);
      return false;
    }
    uint32_t received = 0U;
    while (received < expected) {
      const size_t wanted =
          std::min(sizeof(buffer), (size_t)(expected - received));
      const int count = http->Read(reinterpret_cast<char *>(buffer), wanted);
      if (count <= 0 || (size_t)count > wanted ||
          esp_ota_write(handle, buffer, (size_t)count) != ESP_OK) {
        http->Close();
        esp_ota_abort(handle);
        set_ota_status(OTA_STATE_FAILED, &task, offset + received,
                       count <= 0 ? ESP_ERR_TIMEOUT : ESP_FAIL);
        ESP_LOGW(TAG,
                 "OTA download interrupted at offset=%lu; retaining "
                 "checkpoint for retry",
                 (unsigned long)(offset + received));
        return false;
      }
      received += (uint32_t)count;
      set_ota_status(OTA_STATE_DOWNLOADING, &task, offset + received, 0);
    }
    http->Close();
    offset += received;
    persisted.offset = offset;
    if (save_persisted_state(persisted) != ESP_OK) {
      esp_ota_abort(handle);
      return false;
    }
    const unsigned progress = (unsigned)((uint64_t)offset * 100U / task.size);
    if (progress / 10U > last_progress / 10U) {
      report_status(modem, config, task.task_id, (int)progress);
      last_progress = progress;
    }
  }

  set_ota_status(OTA_STATE_VERIFYING, &task, task.size, 0);
  if (!partition_md5_matches(partition, task.size, task.md5) ||
      !validate_download_description(partition, task)) {
    esp_ota_abort(handle);
    set_ota_status(OTA_STATE_FAILED, &task, task.size,
                   ESP_ERR_OTA_VALIDATE_FAILED);
    report_status(modem, config, task.task_id, 205);
    return false;
  }
  ret = esp_ota_end(handle);
  if (ret != ESP_OK) {
    set_ota_status(OTA_STATE_FAILED, &task, task.size, ret);
    report_status(modem, config, task.task_id, 205);
    return false;
  }
  report_status(modem, config, task.task_id, 101);

  persisted.state = kPersistRebootPending;
  persisted.offset = task.size;
  if (save_persisted_state(persisted) != ESP_OK ||
      (ret = esp_ota_set_boot_partition(partition)) != ESP_OK) {
    set_ota_status(OTA_STATE_FAILED, &task, task.size, ret);
    report_status(modem, config, task.task_id, 206);
    return false;
  }
  set_ota_status(OTA_STATE_PENDING_REBOOT, &task, task.size, 0);
  ESP_LOGI(TAG, "OTA image verified; rebooting into version %s",
           task.target_version);
  vTaskDelay(pdMS_TO_TICKS(1000U));
  esp_restart();
  return true;
}

void boot_validation_task(void *) {
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t image_state = ESP_OTA_IMG_UNDEFINED;
  PersistedState persisted;
  const bool have_persisted = load_persisted_state(&persisted) == ESP_OK;

  if (running != nullptr &&
      esp_ota_get_state_partition(running, &image_state) == ESP_OK &&
      image_state == ESP_OTA_IMG_PENDING_VERIFY) {
    set_ota_status(OTA_STATE_TRIAL,
                   have_persisted ? &persisted.task : nullptr,
                   have_persisted ? persisted.offset : 0U, 0);
    vTaskDelay(pdMS_TO_TICKS(kBootValidationDelayMs));
    if (!cellular_service_config_ready()) {
      ESP_LOGE(TAG,
               "OTA trial failed core validation: OneNET NVS configuration "
               "did not become ready");
      esp_ota_mark_app_invalid_rollback_and_reboot();
      vTaskDelete(nullptr);
      return;
    }
    const esp_err_t ret = esp_ota_mark_app_valid_cancel_rollback();
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "OTA trial image marked valid after core-service soak");
      if (have_persisted && persisted.state == kPersistRebootPending &&
          strcmp(esp_app_get_description()->version,
                 persisted.task.target_version) == 0) {
        persisted.state = kPersistResultPending;
        persisted.result = 201;
        save_persisted_state(persisted);
      }
      set_ota_status(OTA_STATE_IDLE, nullptr, 0U, 0);
    } else {
      ESP_LOGE(TAG, "failed to validate OTA trial image: %s",
               esp_err_to_name(ret));
      esp_ota_mark_app_invalid_rollback_and_reboot();
    }
  } else if (have_persisted && persisted.state == kPersistRebootPending &&
             strcmp(esp_app_get_description()->version,
                    persisted.task.target_version) != 0) {
    persisted.state = kPersistResultPending;
    persisted.result = 206;
    save_persisted_state(persisted);
    set_ota_status(OTA_STATE_FAILED, &persisted.task, persisted.offset,
                   ESP_ERR_OTA_ROLLBACK_FAILED);
    ESP_LOGW(TAG, "previous OTA image rolled back before validation");
  }
  vTaskDelete(nullptr);
}

} // namespace

OneNetOtaService::OneNetOtaService(AtModem *modem, Mqtt *mqtt,
                                   const onenet_config_t &config)
    : modem_(modem), mqtt_(mqtt), config_(config) {
  inform_topic_ = "$sys/" + std::string(config_.product_id) + "/" +
                  config_.device_name + "/ota/inform";
  inform_reply_topic_ = inform_topic_ + "_reply";
}

bool OneNetOtaService::HandleMqttMessage(const std::string &topic,
                                         const std::string &payload) {
  if (topic != inform_topic_) {
    return false;
  }
  char id[32];
  if (!onenet_ota_parse_inform_id(payload.data(), payload.size(), id,
                                  sizeof(id))) {
    ESP_LOGW(TAG, "ignored malformed OneNET OTA notification");
    return true;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pending_inform_id_ = id;
  check_requested_ = true;
  return true;
}

void OneNetOtaService::OnOnline() {
  version_reported_ = false;
  next_check_epoch_ = 0U;
  std::lock_guard<std::mutex> lock(mutex_);
  check_requested_ = true;
}

bool OneNetOtaService::PublishInformReply() {
  std::string id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    id.swap(pending_inform_id_);
  }
  if (id.empty()) {
    return true;
  }
  char payload[160];
  const int length = snprintf(payload, sizeof(payload),
                              "{\"id\":\"%s\",\"code\":200,"
                              "\"msg\":\"success\",\"data\":{}}",
                              id.c_str());
  if (length <= 0 || (size_t)length >= sizeof(payload) || mqtt_ == nullptr ||
      !mqtt_->Publish(inform_reply_topic_, payload, 0)) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_inform_id_.empty()) {
      pending_inform_id_ = id;
    }
    return false;
  }
  ESP_LOGI(TAG, "OneNET OTA notification acknowledged id=%s", id.c_str());
  return true;
}

bool OneNetOtaService::ReportVersion() {
  char path[256];
  char payload[128];
  const char *version = esp_app_get_description()->version;
  const int path_length = snprintf(path, sizeof(path),
                                   "/fuse-ota/%s/%s/version",
                                   config_.product_id, config_.device_name);
  const int payload_length =
      snprintf(payload, sizeof(payload),
               "{\"s_version\":\"%s\",\"f_version\":\"ML307C\"}",
               version);
  if (path_length <= 0 || (size_t)path_length >= sizeof(path) ||
      payload_length <= 0 || (size_t)payload_length >= sizeof(payload)) {
    return false;
  }
  const std::string request(payload, (size_t)payload_length);
  std::string response;
  const bool ok = http_json(modem_, config_, "POST", path, &request, &response) &&
                  response_code_ok(response);
  if (ok) {
    ESP_LOGI(TAG, "OneNET SOTA version reported: %s", version);
  }
  return ok;
}

bool OneNetOtaService::ReportPendingResult() {
  PersistedState persisted;
  if (load_persisted_state(&persisted) != ESP_OK ||
      persisted.state != kPersistResultPending) {
    return true;
  }
  if (!report_status(modem_, config_, persisted.task.task_id,
                     persisted.result)) {
    return false;
  }
  clear_persisted_state();
  set_ota_status(OTA_STATE_IDLE, nullptr, 0U, 0);
  return true;
}

bool OneNetOtaService::CheckAndApply() {
  const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
  if (partition == nullptr) {
    return false;
  }
  char path[320];
  const int path_length = snprintf(
      path, sizeof(path), "/fuse-ota/%s/%s/check?type=2&version=%s",
      config_.product_id, config_.device_name,
      esp_app_get_description()->version);
  if (path_length <= 0 || (size_t)path_length >= sizeof(path)) {
    return false;
  }
  set_ota_status(OTA_STATE_CHECKING, nullptr, 0U, 0);
  std::string response;
  if (!http_json(modem_, config_, "GET", path, nullptr, &response)) {
    set_ota_status(OTA_STATE_IDLE, nullptr, 0U, ESP_ERR_TIMEOUT);
    return false;
  }
  onenet_ota_task_t task = {};
  if (!onenet_ota_parse_task(response.data(), response.size(),
                             esp_app_get_description()->version,
                             partition->size, &task)) {
    set_ota_status(OTA_STATE_IDLE, nullptr, 0U, 0);
    ESP_LOGI(TAG, "no executable OneNET SOTA task");
    return true;
  }
  ESP_LOGI(TAG, "OneNET SOTA task accepted: id=%s target=%s size=%lu",
           task.task_id, task.target_version, (unsigned long)task.size);
  return download_task(modem_, config_, task);
}

void OneNetOtaService::Service() {
  PublishInformReply();
  if (!ReportPendingResult()) {
    return;
  }
  if (!version_reported_) {
    version_reported_ = ReportVersion();
    if (!version_reported_) {
      return;
    }
  }

  const time_t now = time(nullptr);
  bool requested = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    requested = check_requested_;
    check_requested_ = false;
  }
  if (requested || next_check_epoch_ == 0U ||
      (now > 0 && (uint64_t)now >= next_check_epoch_)) {
    const bool ok = CheckAndApply();
    next_check_epoch_ =
        now > 0 ? (uint64_t)now + (ok ? kOtaPollSeconds : 300U) : 0U;
  }
}

extern "C" esp_err_t onenet_ota_boot_guard_start(void) {
  if (s_boot_guard_started) {
    return ESP_ERR_INVALID_STATE;
  }
  const BaseType_t created =
      xTaskCreate(boot_validation_task, "ota_boot_guard", 4096, nullptr,
                  tskIDLE_PRIORITY + 2, nullptr);
  if (created != pdPASS) {
    return ESP_ERR_NO_MEM;
  }
  s_boot_guard_started = true;
  return ESP_OK;
}

extern "C" bool ota_get_status(ota_status_t *status) {
  if (status == nullptr) {
    return false;
  }
  taskENTER_CRITICAL(&s_ota_status_lock);
  *status = s_ota_status;
  taskEXIT_CRITICAL(&s_ota_status_lock);
  return true;
}
