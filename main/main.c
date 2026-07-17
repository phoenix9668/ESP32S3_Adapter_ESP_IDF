#include "app_protocol.h"
#include "board.h"
#include "cellular_service.h"
#include "esp_check.h"
#include "esp_log.h"
#include "radio_service.h"
#include "rfid_store.h"
#include "serial_router.h"

static const char *TAG = "APP";

void app_main(void) {
  app_protocol_init();

  ESP_ERROR_CHECK(board_init());
  ESP_ERROR_CHECK(rfid_store_init());
  ESP_LOGI(TAG, "boot with board address 0x%02x", board_get_address());

  ESP_ERROR_CHECK(serial_router_init());
  ESP_ERROR_CHECK(radio_service_start(board_get_address()));
  ESP_ERROR_CHECK(serial_router_start());

  const esp_err_t cellular_ret = cellular_service_start();
  if (cellular_ret != ESP_OK) {
    ESP_LOGE(TAG, "cellular service unavailable: %s",
             esp_err_to_name(cellular_ret));
  }
}
