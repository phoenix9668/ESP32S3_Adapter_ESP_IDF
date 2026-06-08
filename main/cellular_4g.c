#include "cellular_4g.h"

#include "app_config.h"
#include "board.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define CELLULAR_4G_UART_NUM UART_NUM_2
#define CELLULAR_4G_TXD_PIN GPIO_NUM_2
#define CELLULAR_4G_RXD_PIN GPIO_NUM_1
#define CELLULAR_4G_EN_PIN GPIO_NUM_44
#define CELLULAR_4G_BAUD_RATE 115200
#define CELLULAR_4G_RX_BUF_SIZE 1024
#define CELLULAR_4G_TX_BUF_SIZE 1024
#define CELLULAR_4G_RX_CHUNK_SIZE 128

static const char *TAG = "CELLULAR_4G";

static bool s_initialized;
static TickType_t s_last_valid_frame_tick;
static char s_line_buffer[CELLULAR_4G_PROTOCOL_MAX_FRAME_LEN];
static size_t s_line_length;
static bool s_discarding_line;
static bool s_discard_previous_cr;
static uint32_t s_get_sequence;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static cellular_4g_status_t s_status;

static void cellular_4g_line_parser_reset(void) {
  s_line_length = 0U;
  s_discarding_line = false;
  s_discard_previous_cr = false;
}

static void cellular_4g_set_stale(bool stale) {
  taskENTER_CRITICAL(&s_status_lock);
  s_status.communication_stale = stale;
  taskEXIT_CRITICAL(&s_status_lock);
  if (stale) {
    board_led_set(BOARD_LED_GREEN, false);
  }
}

static void cellular_4g_apply_event_state(cellular_4g_protocol_frame_t *frame) {
  if (strcmp(frame->event, "SIM_READY") == 0) {
    frame->sim = 1U;
  } else if (strcmp(frame->event, "SIM_LOST") == 0) {
    frame->sim = 0U;
  } else if (strcmp(frame->event, "REG_SEARCH") == 0) {
    frame->registration = 1U;
  } else if (strcmp(frame->event, "REG_HOME") == 0) {
    frame->registration = 2U;
  } else if (strcmp(frame->event, "REG_ROAM") == 0) {
    frame->registration = 3U;
  } else if (strcmp(frame->event, "REG_REJECTED") == 0) {
    frame->registration = 4U;
  } else if (strcmp(frame->event, "PDP_UP") == 0) {
    frame->pdp = 1U;
  } else if (strcmp(frame->event, "PDP_DOWN") == 0) {
    frame->pdp = 0U;
  } else if (strcmp(frame->event, "MQTT_CONNECTING") == 0) {
    frame->mqtt = 1U;
  } else if (strcmp(frame->event, "MQTT_UP") == 0) {
    frame->mqtt = 2U;
  } else if (strcmp(frame->event, "MQTT_DOWN") == 0) {
    frame->mqtt = 0U;
  }
}

static void cellular_4g_store_frame(const cellular_4g_protocol_frame_t *frame) {
  bool reboot_detected = false;
  bool recovered = false;
  bool network_online = false;

  taskENTER_CRITICAL(&s_status_lock);
  if (s_status.has_valid_frame &&
      (frame->uptime_sec < s_status.latest.uptime_sec ||
       frame->seq < s_status.latest.seq)) {
    reboot_detected = true;
  }
  recovered = s_status.communication_stale;

  if (frame->type == CELLULAR_4G_MESSAGE_STAT ||
      !s_status.has_complete_status) {
    s_status.latest = *frame;
    if (frame->type == CELLULAR_4G_MESSAGE_EVENT) {
      cellular_4g_apply_event_state(&s_status.latest);
    }
  } else {
    cellular_4g_protocol_frame_t merged = *frame;
    merged.sim = s_status.latest.sim;
    merged.registration = s_status.latest.registration;
    merged.pdp = s_status.latest.pdp;
    merged.mqtt = s_status.latest.mqtt;
    merged.csq = s_status.latest.csq;
    merged.last_tx_sec = s_status.latest.last_tx_sec;
    memcpy(merged.rat, s_status.latest.rat, sizeof(merged.rat));
    cellular_4g_apply_event_state(&merged);
    s_status.latest = merged;
  }

  s_status.has_valid_frame = true;
  if (frame->type == CELLULAR_4G_MESSAGE_STAT) {
    s_status.has_complete_status = true;
  }
  s_status.communication_stale = false;
  network_online = s_status.latest.state == 4U || s_status.latest.state == 5U;
  taskEXIT_CRITICAL(&s_status_lock);

  board_led_set(BOARD_LED_GREEN, network_online);

  if (reboot_detected) {
    ESP_LOGW(TAG, "4G application restart detected: seq=%lu up=%lus",
             (unsigned long)frame->seq, (unsigned long)frame->uptime_sec);
  }
  if (recovered) {
    ESP_LOGI(TAG, "4G communication recovered");
  }
}

static bool cellular_4g_process_line(const char *line, size_t length) {
  cellular_4g_protocol_frame_t frame;
  if (!cellular_4g_protocol_parse(line, length, &frame)) {
    ESP_LOGW(TAG, "ignored invalid 4G protocol line len=%u", (unsigned)length);
    ESP_LOG_BUFFER_HEXDUMP(TAG, line, length, ESP_LOG_DEBUG);
    return false;
  }

  cellular_4g_store_frame(&frame);
  s_last_valid_frame_tick = xTaskGetTickCount();

  if (frame.type == CELLULAR_4G_MESSAGE_EVENT) {
    ESP_LOGI(TAG, "4G EVENT seq=%lu up=%lus ev=%s state=%u cause=%u",
             (unsigned long)frame.seq, (unsigned long)frame.uptime_sec,
             frame.event, frame.state, frame.cause);
  } else {
    ESP_LOGD(TAG,
             "4G STAT seq=%lu up=%lus state=%u sim=%u reg=%u pdp=%u mqtt=%u "
             "csq=%u rat=%s lasttx=%ld cause=%u",
             (unsigned long)frame.seq, (unsigned long)frame.uptime_sec,
             frame.state, frame.sim, frame.registration, frame.pdp, frame.mqtt,
             frame.csq, frame.rat, (long)frame.last_tx_sec, frame.cause);
  }
  return true;
}

static bool cellular_4g_process_rx_data(const uint8_t *data, size_t length) {
  bool valid_frame_seen = false;

  for (size_t i = 0; i < length; ++i) {
    const uint8_t byte = data[i];
    if (s_discarding_line) {
      if (byte == '\n' && s_discard_previous_cr) {
        cellular_4g_line_parser_reset();
      } else {
        s_discard_previous_cr = byte == '\r';
      }
      continue;
    }

    if (byte == '\n') {
      if (s_line_length > 0U && s_line_buffer[s_line_length - 1U] == '\r') {
        const size_t protocol_length = s_line_length - 1U;
        s_line_buffer[protocol_length] = '\0';
        if (cellular_4g_process_line(s_line_buffer, protocol_length)) {
          valid_frame_seen = true;
        }
      } else if (s_line_length > 0U) {
        ESP_LOGW(TAG, "discarded 4G UART line without CRLF");
      }
      cellular_4g_line_parser_reset();
      continue;
    }

    const size_t maximum_before_lf = CELLULAR_4G_PROTOCOL_MAX_FRAME_LEN - 1U;
    if (s_line_length >= maximum_before_lf) {
      ESP_LOGW(TAG, "discarding overlength 4G UART line");
      s_line_length = 0U;
      s_discarding_line = true;
      s_discard_previous_cr = byte == '\r';
      continue;
    }
    s_line_buffer[s_line_length++] = (char)byte;
  }

  return valid_frame_seen;
}

static esp_err_t cellular_4g_enable_gpio_init(void) {
  const gpio_config_t config = {
      .pin_bit_mask = BIT64(CELLULAR_4G_EN_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&config);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = gpio_set_level(CELLULAR_4G_EN_PIN, 1);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "4G EN gpio=%d set high", CELLULAR_4G_EN_PIN);
  }
  return ret;
}

static void cellular_4g_restart(void) {
  ESP_LOGW(TAG, "no valid 4G frame for %u ms, toggling EN gpio=%d",
           APP_CELLULAR_RESTART_TIMEOUT_MS, CELLULAR_4G_EN_PIN);

  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CELLULAR_4G_EN_PIN, 0));
  ESP_LOGW(TAG, "4G EN gpio=%d set low", CELLULAR_4G_EN_PIN);
  vTaskDelay(pdMS_TO_TICKS(APP_CELLULAR_RESTART_LOW_MS));
  uart_flush_input(CELLULAR_4G_UART_NUM);
  cellular_4g_line_parser_reset();
  board_led_set(BOARD_LED_GREEN, false);
  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CELLULAR_4G_EN_PIN, 1));
  ESP_LOGW(TAG, "4G EN gpio=%d set high", CELLULAR_4G_EN_PIN);
  s_last_valid_frame_tick = xTaskGetTickCount();
}

static void cellular_4g_rx_watchdog_task(void *arg) {
  (void)arg;

  uint8_t rx_buf[CELLULAR_4G_RX_CHUNK_SIZE];
  const TickType_t poll_timeout =
      pdMS_TO_TICKS(APP_CELLULAR_RX_POLL_TIMEOUT_MS);
  const TickType_t stale_timeout = pdMS_TO_TICKS(APP_CELLULAR_STALE_TIMEOUT_MS);
  const TickType_t restart_timeout =
      pdMS_TO_TICKS(APP_CELLULAR_RESTART_TIMEOUT_MS);

  while (true) {
    const int rx_bytes = uart_read_bytes(CELLULAR_4G_UART_NUM, rx_buf,
                                         sizeof(rx_buf), poll_timeout);
    if (rx_bytes > 0) {
      cellular_4g_process_rx_data(rx_buf, (size_t)rx_bytes);
    }

    const TickType_t now = xTaskGetTickCount();
    const TickType_t silence = now - s_last_valid_frame_tick;
    if (silence >= stale_timeout) {
      bool already_stale;
      taskENTER_CRITICAL(&s_status_lock);
      already_stale = s_status.communication_stale;
      taskEXIT_CRITICAL(&s_status_lock);
      if (!already_stale) {
        cellular_4g_set_stale(true);
        ESP_LOGW(TAG, "4G communication stale after %u ms",
                 APP_CELLULAR_STALE_TIMEOUT_MS);
        const esp_err_t ret = cellular_4g_request_status();
        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "failed to request immediate 4G status: %s",
                   esp_err_to_name(ret));
        }
      }
    }

    if (silence >= restart_timeout) {
      cellular_4g_restart();
    }
  }
}

/*
 * ESP32 UART2 TX connects to the 4G module RXD, and ESP32 UART2 RX connects
 * to the module TXD. The schematic routes the module EN signal to ESP32-S3
 * GPIO44, where low disables the module power and high enables it.
 */
esp_err_t cellular_4g_init(void) {
  if (s_initialized) {
    return ESP_OK;
  }

  esp_err_t ret = cellular_4g_enable_gpio_init();
  if (ret != ESP_OK) {
    return ret;
  }

  const uart_config_t uart_config = {
      .baud_rate = CELLULAR_4G_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  ret = uart_driver_install(CELLULAR_4G_UART_NUM, CELLULAR_4G_RX_BUF_SIZE,
                            CELLULAR_4G_TX_BUF_SIZE, 0, NULL, 0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    return ret;
  }

  ret = uart_param_config(CELLULAR_4G_UART_NUM, &uart_config);
  if (ret != ESP_OK) {
    return ret;
  }

  ret =
      uart_set_pin(CELLULAR_4G_UART_NUM, CELLULAR_4G_TXD_PIN,
                   CELLULAR_4G_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    return ret;
  }

  uart_flush(CELLULAR_4G_UART_NUM);
  memset(&s_status, 0, sizeof(s_status));
  cellular_4g_line_parser_reset();
  s_get_sequence = 0U;
  s_last_valid_frame_tick = xTaskGetTickCount();
  s_initialized = true;

  const BaseType_t task_created =
      xTaskCreate(cellular_4g_rx_watchdog_task, "cellular_4g_rx",
                  APP_TASK_STACK_DEFAULT, NULL, tskIDLE_PRIORITY + 5, NULL);
  if (task_created != pdPASS) {
    s_initialized = false;
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG,
           "4G UART%d initialized tx=%d rx=%d en=%d baud=%d stale=%ums "
           "restart=%ums",
           CELLULAR_4G_UART_NUM, CELLULAR_4G_TXD_PIN, CELLULAR_4G_RXD_PIN,
           CELLULAR_4G_EN_PIN, CELLULAR_4G_BAUD_RATE,
           APP_CELLULAR_STALE_TIMEOUT_MS, APP_CELLULAR_RESTART_TIMEOUT_MS);
  return ESP_OK;
}

esp_err_t cellular_4g_send(const uint8_t *data, size_t length) {
  if (!s_initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (data == NULL || length == 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  const int tx_bytes = uart_write_bytes(CELLULAR_4G_UART_NUM, data, length);
  if (tx_bytes < 0 || (size_t)tx_bytes != length) {
    ESP_LOGW(TAG, "4G UART write incomplete: %d/%u", tx_bytes,
             (unsigned)length);
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "sent %u bytes to 4G module", (unsigned)length);
  ESP_LOG_BUFFER_HEXDUMP(TAG, data, length, ESP_LOG_DEBUG);
  return ESP_OK;
}

esp_err_t cellular_4g_request_status(void) {
  uint32_t sequence;
  taskENTER_CRITICAL(&s_status_lock);
  sequence = s_get_sequence++;
  taskEXIT_CRITICAL(&s_status_lock);

  char body[80];
  const int body_length =
      snprintf(body, sizeof(body), "$ESP,V=1,T=GET,SEQ=%lu,WHAT=STAT",
               (unsigned long)sequence);
  if (body_length < 0 || (size_t)body_length >= sizeof(body)) {
    return ESP_ERR_INVALID_SIZE;
  }

  const uint16_t crc = cellular_4g_protocol_crc16(body, (size_t)body_length);
  char frame[96];
  const int frame_length =
      snprintf(frame, sizeof(frame), "%s*%04X\r\n", body, crc);
  if (frame_length < 0 || (size_t)frame_length >= sizeof(frame)) {
    return ESP_ERR_INVALID_SIZE;
  }
  return cellular_4g_send((const uint8_t *)frame, (size_t)frame_length);
}

bool cellular_4g_get_status(cellular_4g_status_t *status) {
  if (status == NULL) {
    return false;
  }

  taskENTER_CRITICAL(&s_status_lock);
  *status = s_status;
  taskEXIT_CRITICAL(&s_status_lock);
  return status->has_valid_frame;
}
