#include "app_protocol.h"
#include "board.h"
#include "esp_err.h"
#include "esp_log.h"
#include "radio_service.h"
#include "serial_router.h"

static const char *TAG = "APP";

void app_main(void) {
  app_protocol_init();

  ESP_ERROR_CHECK(board_init());
  ESP_ERROR_CHECK(serial_router_init());

  ESP_LOGI(TAG, "boot with board address 0x%02x", board_get_address());

  ESP_ERROR_CHECK(radio_service_start(board_get_address()));
  ESP_ERROR_CHECK(serial_router_start());
}
