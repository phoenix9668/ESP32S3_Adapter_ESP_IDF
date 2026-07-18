#include "cellular_service.h"

#include "app_config.h"
#include "at_modem.h"
#include "board.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mqtt.h"
#include "nvs_flash.h"
#include "onenet_config.h"
#include "onenet_ota_service.hpp"
#include "onenet_reply.h"
#include "onenet_time.h"
#include "rfid_store.h"
#include "serial_router.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <sys/time.h>
#include <vector>

namespace {

constexpr gpio_num_t kModemTxPin = GPIO_NUM_43;
constexpr gpio_num_t kModemRxPin = GPIO_NUM_44;
// GPIO43/44 are the ESP32-S3 module's native U0TXD/U0RXD pins (physical
// pins 37/36). The console is routed through USB Serial/JTAG, leaving UART0
// dedicated to the directly connected ML307C while UART1 remains for E34.
constexpr uart_port_t kModemUart = UART_NUM_0;
constexpr int kModemBaudRate = 115200;
constexpr int kNetworkReadyTimeoutMs = 60000;
constexpr int kOneNetKeepAliveSeconds = 300;
constexpr uint32_t kGnssPollIntervalMs = 120U * 1000U;
constexpr uint32_t kReplyTimeoutMs = 10U * 1000U;
constexpr uint32_t kUploadRetryDelayMs = 5U * 1000U;
constexpr uint32_t kMetadataRetryDelayMs = 60U * 1000U;
constexpr uint32_t kIdleDelayMs = 250U;
constexpr uint32_t kOtaServiceIntervalMs = 5U * 1000U;
constexpr uint32_t kGnssStateVerifyDelayMs = 250U;
constexpr unsigned kGnssStateVerifyAttempts = 7U;
constexpr EventBits_t kReplyEvent = BIT0;
constexpr EventBits_t kDisconnectedEvent = BIT1;
constexpr size_t kOneNetPayloadMax = 1024U;

const char *TAG = "CELLULAR";

bool s_started;
bool s_sim_diagnostics_printed;
TaskHandle_t s_task;
EventGroupHandle_t s_events;
portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE s_reply_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE s_gnss_lock = portMUX_INITIALIZER_UNLOCKED;
cellular_status_t s_status = {};
gnss_fix_t s_latest_gnss = {};
bool s_gnss_state_seen;
uint8_t s_gnss_state;
bool s_gnss_nmea_mask_seen;
uint8_t s_gnss_nmea_mask;
bool s_gnss_auto_report_seen;
uint8_t s_gnss_auto_report;
bool s_clock_seen;
uint64_t s_clock_epoch;
bool s_ota_maintenance;
char s_inflight_id[16] = {};
int s_reply_code = -1;
uint32_t s_request_id;
char s_reply_topic[256] = {};

std::unique_ptr<AtModem> s_modem;
std::unique_ptr<Mqtt> s_mqtt;
std::unique_ptr<OneNetOtaService> s_ota;

struct SimSnapshot {
  bool ready;
  int csq;
  int status;
  char iccid[33];
  char imsi[33];
};

SimSnapshot s_sim_snapshot = {};
bool s_sim_snapshot_delivered;

void probe_modem_rx_idle_level() {
  gpio_config_t config = {};
  config.pin_bit_mask = 1ULL << kModemRxPin;
  config.mode = GPIO_MODE_INPUT;
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_ENABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&config));
  vTaskDelay(pdMS_TO_TICKS(20));
  const int level = gpio_get_level(kModemRxPin);
  if (level == 0) {
    ESP_LOGW(TAG,
             "ML307 TX idle is low with GPIO44 pulldown; verify H2.6-to-TX "
             "continuity and connector contact");
  } else {
    ESP_LOGI(TAG, "ML307 TX idle is high on GPIO44; RX path is electrically present");
  }
  gpio_reset_pin(kModemRxPin);
}

void print_sim_diagnostics(const std::shared_ptr<AtUart> &uart) {
  if (!uart || s_sim_diagnostics_printed) {
    return;
  }

  // Enable raw AT logging only for this credential-free, read-only command
  // group. Disable it before any OneNET MQTT command can contain a token.
  static constexpr const char *kCommands[] = {
      "AT+CPIN?",   // SIM/PIN state
      "AT+ICCID",   // SIM ICCID
      "AT+CIMI",    // IMSI
      "AT+CSQ",     // radio signal quality
      "AT+COPS?",   // selected operator
      "AT+CGATT?",  // packet-domain attach state
      "AT+CEREG?",  // EPS registration state
  };

  ESP_LOGI(TAG, "--- ML307C SIM/network diagnostics begin ---");
  uart->SetDebug(true);
  for (const char *command : kCommands) {
    if (!uart->SendCommand(command, 2000)) {
      ESP_LOGW(TAG, "SIM diagnostic command failed: %s", command);
    }
  }
  uart->SetDebug(false);
  ESP_LOGI(TAG, "--- ML307C SIM/network diagnostics end ---");
  s_sim_diagnostics_printed = true;
}

void set_status(cellular_state_t state, int error = 0) {
  taskENTER_CRITICAL(&s_status_lock);
  s_status.state = state;
  s_status.last_error = error;
  if (state != CELLULAR_STATE_ONLINE) {
    s_status.mqtt_online = false;
  }
  taskEXIT_CRITICAL(&s_status_lock);
  board_led_set(BOARD_LED_GREEN, state == CELLULAR_STATE_ONLINE);
}

void set_state_preserving_error(cellular_state_t state) {
  taskENTER_CRITICAL(&s_status_lock);
  s_status.state = state;
  if (state != CELLULAR_STATE_ONLINE) {
    s_status.mqtt_online = false;
  }
  taskEXIT_CRITICAL(&s_status_lock);
  board_led_set(BOARD_LED_GREEN, state == CELLULAR_STATE_ONLINE);
}

void set_modem_status(bool modem, bool sim, bool registered, bool data) {
  taskENTER_CRITICAL(&s_status_lock);
  s_status.modem_ready = modem;
  s_status.sim_ready = sim;
  s_status.registered = registered;
  s_status.data_online = data;
  taskEXIT_CRITICAL(&s_status_lock);
}

void set_mqtt_online(bool online) {
  taskENTER_CRITICAL(&s_status_lock);
  s_status.mqtt_online = online;
  if (online) {
    s_status.state = CELLULAR_STATE_ONLINE;
    s_status.last_error = 0;
  }
  taskEXIT_CRITICAL(&s_status_lock);
  board_led_set(BOARD_LED_GREEN, online);
}

void update_csq() {
  if (!s_modem) {
    return;
  }
  const int csq = s_modem->GetCsq();
  taskENTER_CRITICAL(&s_status_lock);
  s_status.csq = csq;
  taskEXIT_CRITICAL(&s_status_lock);
}

bool copy_decimal_identifier(const std::string &source, char *destination,
                             size_t destination_size) {
  if (source.empty() || source.size() >= destination_size) {
    return false;
  }
  for (const char value : source) {
    if (value < '0' || value > '9') {
      return false;
    }
  }
  memcpy(destination, source.data(), source.size());
  destination[source.size()] = '\0';
  return true;
}

bool collect_sim_snapshot() {
  if (!s_modem) {
    return false;
  }

  SimSnapshot snapshot = {};
  snapshot.csq = s_modem->GetCsq();
  snapshot.status = s_modem->pin_ready() ? 1 : 0;
  const std::string iccid = s_modem->GetIccid();
  const auto uart = s_modem->GetAtUart();
  const bool imsi_command_ok = uart->SendCommand("AT+CIMI", 2000);
  const std::string imsi = imsi_command_ok ? uart->GetResponse() : "";

  const bool csq_valid = snapshot.csq >= 0 && snapshot.csq <= 99;
  const bool iccid_valid =
      copy_decimal_identifier(iccid, snapshot.iccid, sizeof(snapshot.iccid));
  const bool imsi_valid =
      copy_decimal_identifier(imsi, snapshot.imsi, sizeof(snapshot.imsi));
  snapshot.ready = csq_valid && snapshot.status == 1 && iccid_valid && imsi_valid;
  if (!snapshot.ready) {
    ESP_LOGW(TAG,
             "SIM snapshot incomplete: ready=%d csq=%d iccid_len=%u "
             "imsi_len=%u",
             snapshot.status == 1, snapshot.csq, (unsigned)iccid.size(),
             (unsigned)imsi.size());
    return false;
  }

  s_sim_snapshot = snapshot;
  ESP_LOGI(TAG, "SIM snapshot captured: csq=%d iccid_len=%u imsi_len=%u",
           snapshot.csq, (unsigned)strlen(snapshot.iccid),
           (unsigned)strlen(snapshot.imsi));
  ESP_LOGI(TAG,
           "FACTORY_STATUS {\"stage\":\"sim\",\"iccid\":\"%s\","
           "\"ready\":true}",
           snapshot.iccid);
  return true;
}

void handle_gnss_urc(const std::string &command,
                     const std::vector<AtArgumentValue> &arguments) {
  if (command == "CCLK" && !arguments.empty()) {
    uint64_t epoch = 0U;
    if (onenet_time_parse_cclk(arguments[0].string_value.c_str(), &epoch)) {
      taskENTER_CRITICAL(&s_gnss_lock);
      s_clock_epoch = epoch;
      s_clock_seen = true;
      taskEXIT_CRITICAL(&s_gnss_lock);
    }
    return;
  }
  if (command == "MGNSSCFG" && arguments.size() >= 2U &&
      arguments[0].type == AtArgumentValue::Type::String &&
      arguments[0].string_value == "nmea/mask" &&
      arguments[1].type == AtArgumentValue::Type::Int &&
      arguments[1].int_value >= 0 && arguments[1].int_value <= 63) {
    taskENTER_CRITICAL(&s_gnss_lock);
    s_gnss_nmea_mask = static_cast<uint8_t>(arguments[1].int_value);
    s_gnss_nmea_mask_seen = true;
    taskEXIT_CRITICAL(&s_gnss_lock);
    return;
  }
  if (command == "MGNSSLOC" && arguments.size() == 1U &&
      arguments[0].type == AtArgumentValue::Type::Int &&
      arguments[0].int_value >= 0 && arguments[0].int_value <= 1) {
    taskENTER_CRITICAL(&s_gnss_lock);
    s_gnss_auto_report = static_cast<uint8_t>(arguments[0].int_value);
    s_gnss_auto_report_seen = true;
    taskEXIT_CRITICAL(&s_gnss_lock);
    return;
  }
  int state = -1;
  if (command == "MGNSS" && !arguments.empty() &&
      arguments[0].type == AtArgumentValue::Type::Int) {
    state = arguments[0].int_value;
  } else if (command == "MGNSSURC" && arguments.size() >= 2U &&
             arguments[0].string_value == "state" &&
             arguments[1].type == AtArgumentValue::Type::Int) {
    state = arguments[1].int_value;
  }
  if (state >= 0 && state <= 2) {
    taskENTER_CRITICAL(&s_gnss_lock);
    s_gnss_state = static_cast<uint8_t>(state);
    s_gnss_state_seen = true;
    taskEXIT_CRITICAL(&s_gnss_lock);
    ESP_LOGI(TAG, "GNSS engine state=%d", state);
    return;
  }

  if (command != "MGNSSLOC" || arguments.size() < 12U) {
    return;
  }
  const char *fields[12];
  for (size_t i = 0U; i < 12U; ++i) {
    fields[i] = arguments[i].raw_value.c_str();
  }
  gnss_fix_t parsed = {};
  if (!gnss_ml307c_parse_fields(fields, 12U, &parsed)) {
    ESP_LOGW(TAG, "ignored malformed +MGNSSLOC response");
    return;
  }

  taskENTER_CRITICAL(&s_gnss_lock);
  parsed.generation = s_latest_gnss.generation + 1U;
  if (parsed.generation == 0U) {
    parsed.generation = 1U;
  }
  s_latest_gnss = parsed;
  taskEXIT_CRITICAL(&s_gnss_lock);

  if (parsed.valid) {
    ESP_LOGI(TAG, "GNSS fix generation=%lu lat=%.6f lon=%.6f sats=%u",
             (unsigned long)parsed.generation, parsed.latitude,
             parsed.longitude, parsed.satellites);
  } else {
    ESP_LOGI(TAG, "GNSS no fix generation=%lu type=%u",
             (unsigned long)parsed.generation, parsed.fix_type);
  }
}

void handle_mqtt_message(const std::string &topic,
                         const std::string &payload) {
  if (s_ota && s_ota->HandleMqttMessage(topic, payload)) {
    return;
  }
  if (topic != s_reply_topic) {
    return;
  }
  char reply_id[16];
  int code = -1;
  if (!onenet_reply_parse(payload.data(), payload.size(), reply_id,
                          sizeof(reply_id), &code)) {
    ESP_LOGW(TAG, "ignored malformed OneNET reply");
    return;
  }

  bool matches = false;
  taskENTER_CRITICAL(&s_reply_lock);
  if (s_inflight_id[0] != '\0' &&
      strcmp(reply_id, s_inflight_id) == 0) {
    s_reply_code = code;
    matches = true;
  }
  taskEXIT_CRITICAL(&s_reply_lock);
  if (matches) {
    if (code != 200) {
      const size_t log_length = std::min(payload.size(), size_t{256U});
      ESP_LOGW(TAG, "OneNET reply payload=%.*s", (int)log_length,
               payload.data());
    }
    xEventGroupSetBits(s_events, kReplyEvent);
  } else {
    ESP_LOGD(TAG, "ignored OneNET reply for id=%s", reply_id);
  }
}

void next_request_id(char *id, size_t id_size) {
  const uint32_t request = s_request_id++;
  if (s_request_id == 0U) {
    s_request_id = 1U;
  }
  snprintf(id, id_size, "%lu", (unsigned long)request);
}

bool publish_confirmed(const char *request_id, const char *payload,
                       const char *post_topic) {
  xEventGroupClearBits(s_events, kReplyEvent | kDisconnectedEvent);
  taskENTER_CRITICAL(&s_reply_lock);
  snprintf(s_inflight_id, sizeof(s_inflight_id), "%s", request_id);
  s_reply_code = -1;
  taskEXIT_CRITICAL(&s_reply_lock);

  if (!s_mqtt || !s_mqtt->Publish(post_topic, payload, 0)) {
    taskENTER_CRITICAL(&s_reply_lock);
    s_inflight_id[0] = '\0';
    taskEXIT_CRITICAL(&s_reply_lock);
    return false;
  }

  const EventBits_t bits =
      xEventGroupWaitBits(s_events, kReplyEvent | kDisconnectedEvent, pdTRUE,
                          pdFALSE, pdMS_TO_TICKS(kReplyTimeoutMs));
  int reply_code = -1;
  taskENTER_CRITICAL(&s_reply_lock);
  reply_code = s_reply_code;
  s_inflight_id[0] = '\0';
  taskEXIT_CRITICAL(&s_reply_lock);
  if ((bits & kReplyEvent) == 0U) {
    ESP_LOGW(TAG, "OneNET reply timeout id=%s", request_id);
    return false;
  }
  if (reply_code != 200) {
    ESP_LOGW(TAG, "OneNET rejected id=%s code=%d", request_id, reply_code);
    return false;
  }
  return true;
}

bool upload_rfid(const rfid_store_item_t &item, const char *post_topic) {
  char request_id[16];
  next_request_id(request_id, sizeof(request_id));
  char payload[kOneNetPayloadMax];
  const int written = snprintf(
      payload, sizeof(payload),
      "{\"id\":\"%s\",\"version\":\"1.0\",\"params\":{"
      "\"rfid_tag_id\":{\"value\":\"%.*s\"}}}",
      request_id, (int)item.length, (const char *)item.payload);
  if (written < 0 || (size_t)written >= sizeof(payload)) {
    return false;
  }
  if (!publish_confirmed(request_id, payload, post_topic)) {
    return false;
  }
  const esp_err_t ret = rfid_store_mark_delivered(item.sequence);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "RFID seq=%lu confirmed but mark failed: %s",
             (unsigned long)item.sequence, esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "RFID seq=%lu delivered, pending=%u",
           (unsigned long)item.sequence,
           (unsigned)rfid_store_pending_count());
  return true;
}

bool upload_gnss(const gnss_fix_t &fix, const char *post_topic) {
  char request_id[16];
  next_request_id(request_id, sizeof(request_id));
  char payload[kOneNetPayloadMax];
  int written;
  if (!fix.valid) {
    written = snprintf(
        payload, sizeof(payload),
        "{\"id\":\"%s\",\"version\":\"1.0\","
        "\"params\":{\"fix_status\":{\"value\":0}}}",
        request_id);
  } else {
    written = snprintf(
        payload, sizeof(payload),
        "{\"id\":\"%s\",\"version\":\"1.0\",\"params\":{"
        "\"fix_status\":{\"value\":1},"
        "\"latitude\":{\"value\":%.6f},"
        "\"longitude\":{\"value\":%.6f},"
        "\"altitude\":{\"value\":%.1f},"
        "\"speed_kph\":{\"value\":%.1f},"
        "\"course_deg\":{\"value\":%.1f},"
        "\"satellites\":{\"value\":%u},"
        "\"hdop\":{\"value\":%.1f},"
        "\"utc_time\":{\"value\":\"%s\"}}}",
        request_id, fix.latitude, fix.longitude, fix.altitude_m, fix.speed_kph,
        fix.course_deg, fix.satellites, fix.hdop, fix.utc_time);
  }
  return written >= 0 && (size_t)written < sizeof(payload) &&
         publish_confirmed(request_id, payload, post_topic);
}

bool upload_sim_snapshot(const char *post_topic) {
  if (!s_sim_snapshot.ready) {
    return false;
  }

  char request_id[16];
  next_request_id(request_id, sizeof(request_id));
  char payload[kOneNetPayloadMax];
  const int written = snprintf(
      payload, sizeof(payload),
      "{\"id\":\"%s\",\"version\":\"1.0\",\"params\":{"
      "\"sim_csq\":{\"value\":%d},"
      "\"sim_status\":{\"value\":%d},"
      "\"sim_iccid\":{\"value\":\"%s\"},"
      "\"sim_imsi\":{\"value\":\"%s\"}}}",
      request_id, s_sim_snapshot.csq, s_sim_snapshot.status,
      s_sim_snapshot.iccid, s_sim_snapshot.imsi);
  return written >= 0 && (size_t)written < sizeof(payload) &&
         publish_confirmed(request_id, payload, post_topic);
}

bool upload_firmware_version(const char *post_topic) {
  const esp_app_desc_t *description = esp_app_get_description();
  if (description == nullptr || description->version[0] == '\0') {
    ESP_LOGE(TAG, "application version is unavailable");
    return false;
  }

  char request_id[16];
  next_request_id(request_id, sizeof(request_id));
  char payload[kOneNetPayloadMax];
  const int written = snprintf(
      payload, sizeof(payload),
      "{\"id\":\"%s\",\"version\":\"1.0\",\"params\":{"
      "\"firmware_version\":{\"value\":\"%s\"}}}",
      request_id, description->version);
  return written >= 0 && (size_t)written < sizeof(payload) &&
         publish_confirmed(request_id, payload, post_topic);
}

bool suppress_unsolicited_gnss_reports(
    const std::shared_ptr<AtUart> &uart) {
  const auto query_nmea_mask = [&uart](uint8_t *mask) {
    taskENTER_CRITICAL(&s_gnss_lock);
    s_gnss_nmea_mask_seen = false;
    taskEXIT_CRITICAL(&s_gnss_lock);
    if (!uart->SendCommand("AT+MGNSSCFG=\"nmea/mask\"", 3000)) {
      return false;
    }
    taskENTER_CRITICAL(&s_gnss_lock);
    const bool seen = s_gnss_nmea_mask_seen;
    if (seen) {
      *mask = s_gnss_nmea_mask;
    }
    taskEXIT_CRITICAL(&s_gnss_lock);
    return seen;
  };

  uint8_t nmea_mask = 0xFFU;
  if ((!query_nmea_mask(&nmea_mask) || nmea_mask != 0U) &&
      (!uart->SendCommand("AT+MGNSSCFG=\"nmea/mask\",0", 3000) ||
       !query_nmea_mask(&nmea_mask) || nmea_mask != 0U)) {
    ESP_LOGW(TAG, "failed to disable unsolicited GNSS NMEA output");
    return false;
  }

  const auto query_auto_report = [&uart](uint8_t *enabled) {
    taskENTER_CRITICAL(&s_gnss_lock);
    s_gnss_auto_report_seen = false;
    taskEXIT_CRITICAL(&s_gnss_lock);
    if (!uart->SendCommand("AT+MGNSSLOC?", 3000)) {
      return false;
    }
    taskENTER_CRITICAL(&s_gnss_lock);
    const bool seen = s_gnss_auto_report_seen;
    if (seen) {
      *enabled = s_gnss_auto_report;
    }
    taskEXIT_CRITICAL(&s_gnss_lock);
    return seen;
  };

  uint8_t auto_report = 0xFFU;
  if ((!query_auto_report(&auto_report) || auto_report != 0U) &&
      (!uart->SendCommand("AT+MGNSSLOC=0", 3000) ||
       !query_auto_report(&auto_report) || auto_report != 0U)) {
    ESP_LOGW(TAG, "failed to disable automatic GNSS location reports");
    return false;
  }
  ESP_LOGI(TAG,
           "unsolicited GNSS NMEA/location reports disabled");
  return true;
}

bool query_gnss_state(const std::shared_ptr<AtUart> &uart, uint8_t *state) {
  taskENTER_CRITICAL(&s_gnss_lock);
  s_gnss_state_seen = false;
  taskEXIT_CRITICAL(&s_gnss_lock);
  if (!uart->SendCommand("AT+MGNSS?", 3000)) {
    return false;
  }
  taskENTER_CRITICAL(&s_gnss_lock);
  const bool seen = s_gnss_state_seen;
  if (seen) {
    *state = s_gnss_state;
  }
  taskEXIT_CRITICAL(&s_gnss_lock);
  return seen;
}

bool set_gnss_engine_enabled(const std::shared_ptr<AtUart> &uart,
                             bool enabled) {
  const uint8_t expected = enabled ? 1U : 0U;
  uint8_t state = 0xFFU;
  if (query_gnss_state(uart, &state) && state == expected) {
    return true;
  }

  const char *command = enabled ? "AT+MGNSS=1" : "AT+MGNSS=0";
  const bool command_ok = uart->SendCommand(command, 5000);
  // ML307C can acknowledge MGNSS before the GNSS engine state changes. Poll
  // for the asynchronous transition instead of issuing back-to-back queries
  // and reporting a false failure during the next modem operation.
  for (unsigned attempt = 0U; attempt < kGnssStateVerifyAttempts; ++attempt) {
    if (attempt > 0U) {
      vTaskDelay(pdMS_TO_TICKS(kGnssStateVerifyDelayMs));
    }
    if (query_gnss_state(uart, &state) && state == expected) {
      if (!command_ok) {
        ESP_LOGW(TAG, "GNSS state command returned an error but state=%u",
                 state);
      }
      return true;
    }
  }
  ESP_LOGW(TAG, "failed to set GNSS engine state=%u, final state=%u",
           expected, state);
  return false;
}

bool configure_gnss() {
  const auto uart = s_modem->GetAtUart();
  if (!suppress_unsolicited_gnss_reports(uart)) {
    return false;
  }
  uint8_t state = 0U;
  if (query_gnss_state(uart, &state) && state == 1U) {
    ESP_LOGI(TAG, "continuous GNSS already enabled");
    return true;
  }
  if (set_gnss_engine_enabled(uart, true)) {
    ESP_LOGI(TAG, "continuous GNSS enabled");
    return true;
  }
  ESP_LOGW(TAG, "failed to enable continuous GNSS, final state=%u", state);
  return false;
}

bool synchronize_network_time() {
  if (!s_modem) {
    return false;
  }
  const auto uart = s_modem->GetAtUart();
  for (unsigned attempt = 0U; attempt < 5U; ++attempt) {
    taskENTER_CRITICAL(&s_gnss_lock);
    s_clock_seen = false;
    s_clock_epoch = 0U;
    taskEXIT_CRITICAL(&s_gnss_lock);
    if (uart->SendCommand("AT+CCLK?", 3000)) {
      uint64_t epoch = 0U;
      bool seen = false;
      taskENTER_CRITICAL(&s_gnss_lock);
      seen = s_clock_seen;
      epoch = s_clock_epoch;
      taskEXIT_CRITICAL(&s_gnss_lock);
      if (seen) {
        struct timeval value = {};
        value.tv_sec = (time_t)epoch;
        if (settimeofday(&value, nullptr) == 0) {
          ESP_LOGI(TAG, "system UTC synchronized from ML307 network clock");
          return true;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000U));
  }
  ESP_LOGW(TAG, "ML307 network clock is not valid yet");
  return false;
}

bool connect_cloud(const onenet_config_t &config, const char *token,
                   char *post_topic, size_t post_topic_size) {
  const int post_len =
      snprintf(post_topic, post_topic_size,
               "$sys/%s/%s/thing/property/post", config.product_id,
               config.device_name);
  const int reply_len =
      snprintf(s_reply_topic, sizeof(s_reply_topic),
               "$sys/%s/%s/thing/property/post/reply", config.product_id,
               config.device_name);
  if (post_len < 0 || (size_t)post_len >= post_topic_size || reply_len < 0 ||
      (size_t)reply_len >= sizeof(s_reply_topic)) {
    return false;
  }

  s_mqtt = s_modem->CreateMqtt(0);
  if (!s_mqtt) {
    return false;
  }
  s_ota = std::make_unique<OneNetOtaService>(s_modem.get(), s_mqtt.get(),
                                             config);
  s_mqtt->SetKeepAlive(kOneNetKeepAliveSeconds);
  s_mqtt->OnMessage(handle_mqtt_message);
  s_mqtt->OnDisconnected([]() {
    set_mqtt_online(false);
    xEventGroupSetBits(s_events, kDisconnectedEvent);
  });
  s_mqtt->OnError([](const std::string &error) {
    ESP_LOGW(TAG, "MQTT error: %s", error.c_str());
  });
  if (!s_mqtt->Connect(config.broker_host, config.broker_port,
                       config.device_name, config.product_id, token)) {
    return false;
  }
  if (!s_mqtt->Subscribe(s_reply_topic, 0)) {
    s_mqtt->Disconnect();
    return false;
  }
  if (!s_mqtt->Subscribe(s_ota->inform_topic(), 0)) {
    s_mqtt->Disconnect();
    return false;
  }
  set_mqtt_online(true);
  s_ota->OnOnline();
  ESP_LOGI(TAG,
           "FACTORY_STATUS {\"stage\":\"online\",\"device\":\"%s\","
           "\"iccid\":\"%s\",\"mqtt\":true}",
           config.device_name,
           s_sim_snapshot.ready ? s_sim_snapshot.iccid : "");
  return true;
}

void online_loop(const char *post_topic) {
  TickType_t next_gnss_poll = xTaskGetTickCount();
  TickType_t next_mqtt_check =
      xTaskGetTickCount() + pdMS_TO_TICKS(30000U);
  TickType_t next_gnss_upload_attempt = 0U;
  TickType_t next_rfid_upload_attempt = 0U;
  TickType_t next_sim_upload_attempt = 0U;
  TickType_t next_metadata_upload_attempt = 0U;
  TickType_t next_ota_service = 0U;
  uint32_t delivered_gnss_generation = 0U;
  bool firmware_version_delivered = false;
  bool prefer_gnss = true;
  bool ota_maintenance_seen = false;

  while (true) {
    const TickType_t now = xTaskGetTickCount();
    if (s_ota && (int32_t)(now - next_ota_service) >= 0) {
      s_ota->Service();
      next_ota_service = xTaskGetTickCount() +
                         pdMS_TO_TICKS(kOtaServiceIntervalMs);
    }
    if (s_ota && s_ota->maintenance_active()) {
      ota_maintenance_seen = true;
      if ((xEventGroupGetBits(s_events) & kDisconnectedEvent) != 0U) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(kIdleDelayMs));
      continue;
    }
    if (ota_maintenance_seen) {
      gnss_fix_t stale_fix = {};
      cellular_service_get_latest_gnss(&stale_fix);
      delivered_gnss_generation = stale_fix.generation;
      next_gnss_poll = xTaskGetTickCount();
      ota_maintenance_seen = false;
      ESP_LOGI(TAG, "normal telemetry resumed after OTA maintenance");
    }
    if ((int32_t)(now - next_gnss_poll) >= 0) {
      if (!s_modem->GetAtUart()->SendCommand("AT+MGNSSLOC", 5000)) {
        ESP_LOGW(TAG, "GNSS location query failed");
      }
      next_gnss_poll = now + pdMS_TO_TICKS(kGnssPollIntervalMs);
    }

    if ((int32_t)(now - next_mqtt_check) >= 0) {
      if (!s_mqtt->IsConnected()) {
        ESP_LOGW(TAG, "MQTT connection check failed");
        break;
      }
      update_csq();
      next_mqtt_check = now + pdMS_TO_TICKS(30000U);
    }

    const bool metadata_upload_due =
        !firmware_version_delivered &&
        (int32_t)(now - next_metadata_upload_attempt) >= 0;
    if (metadata_upload_due) {
      if (upload_firmware_version(post_topic)) {
        firmware_version_delivered = true;
        next_metadata_upload_attempt = 0U;
        ESP_LOGI(TAG, "firmware_version=%s delivered",
                 esp_app_get_description()->version);
      } else {
        next_metadata_upload_attempt =
            xTaskGetTickCount() + pdMS_TO_TICKS(kMetadataRetryDelayMs);
      }
    }

    const bool sim_upload_due =
        s_sim_snapshot.ready && !s_sim_snapshot_delivered &&
        (int32_t)(now - next_sim_upload_attempt) >= 0;
    if (sim_upload_due) {
      if (upload_sim_snapshot(post_topic)) {
        s_sim_snapshot_delivered = true;
        next_sim_upload_attempt = 0U;
        ESP_LOGI(TAG, "SIM snapshot delivered");
      } else {
        next_sim_upload_attempt =
            xTaskGetTickCount() + pdMS_TO_TICKS(kUploadRetryDelayMs);
      }
    }

    gnss_fix_t fix = {};
    cellular_service_get_latest_gnss(&fix);
    const bool gnss_pending =
        fix.generation != 0U &&
        fix.generation != delivered_gnss_generation;
    const bool gnss_upload_due =
        gnss_pending &&
        (int32_t)(now - next_gnss_upload_attempt) >= 0;

    if (gnss_upload_due && prefer_gnss) {
      if (upload_gnss(fix, post_topic)) {
        delivered_gnss_generation = fix.generation;
        next_gnss_upload_attempt = 0U;
        ESP_LOGI(TAG, "GNSS generation=%lu delivered",
                 (unsigned long)fix.generation);
      } else {
        next_gnss_upload_attempt =
            xTaskGetTickCount() + pdMS_TO_TICKS(kUploadRetryDelayMs);
      }
      prefer_gnss = false;
    } else {
      rfid_store_item_t item = {};
      const esp_err_t ret = rfid_store_peek_oldest(&item);
      if (ret == ESP_OK) {
        if ((int32_t)(now - next_rfid_upload_attempt) >= 0) {
          if (upload_rfid(item, post_topic)) {
            next_rfid_upload_attempt = 0U;
          } else {
            next_rfid_upload_attempt =
                xTaskGetTickCount() + pdMS_TO_TICKS(kUploadRetryDelayMs);
          }
          prefer_gnss = true;
        }
      } else if (ret == ESP_ERR_NOT_FOUND && gnss_upload_due) {
        if (upload_gnss(fix, post_topic)) {
          delivered_gnss_generation = fix.generation;
          next_gnss_upload_attempt = 0U;
          ESP_LOGI(TAG, "GNSS generation=%lu delivered",
                   (unsigned long)fix.generation);
        } else {
          next_gnss_upload_attempt =
              xTaskGetTickCount() + pdMS_TO_TICKS(kUploadRetryDelayMs);
        }
        prefer_gnss = false;
      } else if (ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "RFID queue read failed: %s", esp_err_to_name(ret));
      }
    }

    EventBits_t bits = xEventGroupGetBits(s_events);
    if ((bits & kDisconnectedEvent) != 0U) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(kIdleDelayMs));
  }
}

void cellular_task(void *) {
  probe_modem_rx_idle_level();

  onenet_config_t config = {};
  esp_err_t ret = onenet_config_load(&config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "OneNET NVS configuration unavailable: %s",
             esp_err_to_name(ret));
    set_status(CELLULAR_STATE_STOPPED, ret);
    s_task = nullptr;
    vTaskDelete(nullptr);
    return;
  }
  ESP_LOGI(TAG,
           "FACTORY_STATUS {\"stage\":\"nvs\",\"device\":\"%s\","
           "\"ready\":true}",
           config.device_name);
  taskENTER_CRITICAL(&s_status_lock);
  s_status.config_ready = true;
  taskEXIT_CRITICAL(&s_status_lock);

  uint32_t backoff_ms = 2000U;
  while (true) {
    set_status(CELLULAR_STATE_MODEM_DETECTING);
    set_modem_status(false, false, false, false);
    s_modem = AtModem::Detect(kModemTxPin, kModemRxPin, GPIO_NUM_NC,
                              kModemBaudRate, 20000, kModemUart);
    if (!s_modem) {
      ESP_LOGW(TAG, "ML307C detection failed");
      set_status(CELLULAR_STATE_BACKOFF, ESP_ERR_NOT_FOUND);
    } else {
      set_modem_status(true, false, false, false);
      ESP_LOGI(TAG,
               "FACTORY_STATUS {\"stage\":\"modem\",\"ready\":true}");
      s_modem->GetAtUart()->RegisterUrcCallback(handle_gnss_urc);
      // Stop persistent NMEA output before verbose SIM diagnostics or any
      // future HTTP transfer. This does not stop the GNSS engine; the normal
      // post-attach configuration below keeps MGNSS=1 for hot continuous fix.
      if (!suppress_unsolicited_gnss_reports(s_modem->GetAtUart())) {
        ESP_LOGW(TAG,
                 "early GNSS UART quiesce failed; retrying after attach");
      }
      print_sim_diagnostics(s_modem->GetAtUart());
      set_status(CELLULAR_STATE_NETWORK_ATTACHING);
      const NetworkStatus network =
          s_modem->WaitForNetworkReady(kNetworkReadyTimeoutMs);
      if (network == NetworkStatus::Ready) {
        set_modem_status(true, true, true, true);
        if (!synchronize_network_time()) {
          set_status(CELLULAR_STATE_BACKOFF, ESP_ERR_INVALID_STATE);
          goto reconnect;
        }
        configure_gnss();
        update_csq();
        if (!s_sim_snapshot.ready) {
          for (unsigned attempt = 0U;
               attempt < 3U && !collect_sim_snapshot(); ++attempt) {
            vTaskDelay(pdMS_TO_TICKS(250U));
          }
        }
        set_status(CELLULAR_STATE_MQTT_CONNECTING);
        char token[ONENET_TOKEN_MAX_LEN];
        const time_t now = time(nullptr);
        ret = onenet_generate_mqtt_token(
            &config, (uint64_t)now + config.token_ttl, token, sizeof(token));
        if (ret != ESP_OK) {
          ESP_LOGE(TAG, "OneNET token generation failed: %s",
                   esp_err_to_name(ret));
          set_status(CELLULAR_STATE_BACKOFF, ret);
          goto reconnect;
        }
        ESP_LOGI(TAG,
                 "OneNET authorization prepared: utc=%lld expiry=%llu "
                 "ttl=%lu token_len=%u",
                 (long long)now,
                 (unsigned long long)((uint64_t)now + config.token_ttl),
                 (unsigned long)config.token_ttl, (unsigned)strlen(token));
        char post_topic[256];
        if (connect_cloud(config, token, post_topic, sizeof(post_topic))) {
          backoff_ms = 2000U;
          online_loop(post_topic);
        } else {
          ESP_LOGW(TAG, "OneNET MQTT connection failed");
          set_status(CELLULAR_STATE_BACKOFF, ESP_FAIL);
        }
      } else {
        ESP_LOGW(TAG, "cellular network unavailable, status=%d",
                 static_cast<int>(network));
        set_status(CELLULAR_STATE_BACKOFF, static_cast<int>(network));
      }
    }

  reconnect:
    set_mqtt_online(false);
    s_ota.reset();
    if (s_mqtt) {
      s_mqtt->Disconnect();
      s_mqtt.reset();
    }
    s_modem.reset();
    set_modem_status(false, false, false, false);
    set_state_preserving_error(CELLULAR_STATE_BACKOFF);
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    backoff_ms = backoff_ms < 30000U ? backoff_ms * 2U : 60000U;
  }
}

} // namespace

extern "C" esp_err_t cellular_service_start(void) {
  if (s_started) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t ret = nvs_flash_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS init failed without erase: %s", esp_err_to_name(ret));
    return ret;
  }
  s_events = xEventGroupCreate();
  if (s_events == nullptr) {
    return ESP_ERR_NO_MEM;
  }
  memset(&s_status, 0, sizeof(s_status));
  s_sim_diagnostics_printed = false;
  memset(&s_sim_snapshot, 0, sizeof(s_sim_snapshot));
  s_sim_snapshot.csq = -1;
  s_sim_snapshot.status = -1;
  s_sim_snapshot_delivered = false;
  s_gnss_state_seen = false;
  s_gnss_state = 0U;
  s_gnss_nmea_mask_seen = false;
  s_gnss_nmea_mask = 0U;
  s_gnss_auto_report_seen = false;
  s_gnss_auto_report = 0U;
  s_clock_seen = false;
  s_clock_epoch = 0U;
  s_ota_maintenance = false;
  s_request_id = esp_random();
  if (s_request_id == 0U) {
    s_request_id = 1U;
  }
  s_status.csq = -1;
  const BaseType_t created =
      xTaskCreate(cellular_task, "cellular_service", 16384, nullptr,
                  tskIDLE_PRIORITY + 5, &s_task);
  if (created != pdPASS) {
    vEventGroupDelete(s_events);
    s_events = nullptr;
    return ESP_ERR_NO_MEM;
  }
  s_started = true;
  ESP_LOGI(TAG, "ML307C service started UART%d tx=%d rx=%d baud=%d",
           kModemUart, kModemTxPin, kModemRxPin, kModemBaudRate);
  return ESP_OK;
}

extern "C" bool cellular_service_get_status(cellular_status_t *status) {
  if (status == nullptr) {
    return false;
  }
  taskENTER_CRITICAL(&s_status_lock);
  *status = s_status;
  taskEXIT_CRITICAL(&s_status_lock);
  return s_started;
}

extern "C" bool cellular_service_config_ready(void) {
  taskENTER_CRITICAL(&s_status_lock);
  const bool ready = s_status.config_ready;
  taskEXIT_CRITICAL(&s_status_lock);
  return ready;
}

extern "C" bool cellular_service_get_latest_gnss(gnss_fix_t *fix) {
  if (fix == nullptr) {
    return false;
  }
  taskENTER_CRITICAL(&s_gnss_lock);
  *fix = s_latest_gnss;
  taskEXIT_CRITICAL(&s_gnss_lock);
  return fix->generation != 0U;
}

extern "C" bool cellular_service_set_ota_maintenance(bool enabled) {
  taskENTER_CRITICAL(&s_status_lock);
  const bool unchanged = s_ota_maintenance == enabled;
  taskEXIT_CRITICAL(&s_status_lock);
  if (unchanged) {
    return true;
  }

  if (enabled) {
    const esp_err_t pause_ret = serial_router_set_paused(true);
    if (pause_ret != ESP_OK) {
      ESP_LOGE(TAG, "failed to pause RFID/CH9434 for OTA: %s",
               esp_err_to_name(pause_ret));
      return false;
    }
    if (!s_modem ||
        !set_gnss_engine_enabled(s_modem->GetAtUart(), false)) {
      ESP_LOGE(TAG, "failed to stop GNSS engine for OTA");
      serial_router_set_paused(false);
      return false;
    }
    taskENTER_CRITICAL(&s_status_lock);
    s_ota_maintenance = true;
    taskEXIT_CRITICAL(&s_status_lock);
    ESP_LOGI(TAG,
             "OTA maintenance active: RFID/CH9434 and GNSS engine stopped");
    return true;
  }

  bool gnss_ready = true;
  if (s_modem) {
    gnss_ready = configure_gnss();
  }
  const esp_err_t resume_ret = serial_router_set_paused(false);
  taskENTER_CRITICAL(&s_status_lock);
  s_ota_maintenance = false;
  taskEXIT_CRITICAL(&s_status_lock);
  if (!gnss_ready) {
    ESP_LOGW(TAG, "GNSS engine did not resume cleanly after OTA");
  }
  if (resume_ret != ESP_OK) {
    ESP_LOGW(TAG, "RFID/CH9434 did not resume cleanly after OTA: %s",
             esp_err_to_name(resume_ret));
  }
  ESP_LOGI(TAG, "OTA maintenance cleared; normal services resumed");
  return gnss_ready && resume_ret == ESP_OK;
}
