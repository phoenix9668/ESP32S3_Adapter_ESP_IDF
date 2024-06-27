/********************************** (C) COPYRIGHT *******************************
 * File Name          : e34_2g4d20d.h
 * Author             : pheonix
 * Version            : V1.0
 * Date               : 2023/11/22
 * Description        : e34_2g4d20d driver file
 *******************************************************************************/

#include "e34_2g4d20d.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "string.h"

/*
 * e34_2g4d20d parameters definition
 */
static const char *E34_2G4D20D_TAG = "E34_2G4D20D";
char read_parameter[4] = {0xC1, 0xC1, 0xC1, '\0'};
char read_version[4] = {0xC3, 0xC3, 0xC3, '\0'};
char reset_device[4] = {0xC4, 0xC4, 0xC4, '\0'};

/* -----------------------------------------------------------------------------
 *                      define E34_2G4D20D interface functions
 * -----------------------------------------------------------------------------
 */
void e34_2g4d20d_uart1_init(void)
{
    esp_log_level_set(E34_2G4D20D_TAG, ESP_LOG_DEBUG);
    ESP_LOGD(E34_2G4D20D_TAG, "Initializing bus UART%d...", UART_NUM_1);
    const uart_config_t uart1_config = {
        .baud_rate = E34_2G4D20D_Baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart1_config);
    uart_set_pin(UART_NUM_1, UART1_TXD_PIN, UART1_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void e34_2g4d20d_gpio_init(void)
{
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (1ULL << M0) | (1ULL << M1);
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_conf);

    gpio_set_level(M0, 1);
    gpio_set_level(M1, 0);
}

int e34_2g4d20d_sendData(const char *logName, const char *data, int len)
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGD(logName, "Wrote %d bytes", txBytes);
    // ESP_LOG_BUFFER_HEXDUMP(logName, data, len, ESP_LOG_DEBUG);
    return txBytes;
}

/* -----------------------------------------------------------------------------
 *                                  api function
 * -----------------------------------------------------------------------------
 */

void e34_2g4d20d_model_sel(modes_t mode)
{
    if (mode == HALF_DUPLEX)
    {
        gpio_set_level(M0, 0);
        gpio_set_level(M1, 0);
    }
    else if (mode == FULL_DUPLEX)
    {
        gpio_set_level(M0, 1);
        gpio_set_level(M1, 0);
    }
    else if (mode == RESERVE)
    {
        gpio_set_level(M0, 0);
        gpio_set_level(M1, 1);
    }
    else if (mode == SET)
    {
        gpio_set_level(M0, 1);
        gpio_set_level(M1, 1);
    }
}

void e34_2g4d20d_parameter_set(char dev_head, char dev_addh, char dev_addl, char dev_sped, char dev_chan, char dev_option)
{
    // char dev_head = 0xc0;
    // char dev_addh = 0x00;
    // char dev_addl = 0x00;
    // char dev_sped = 0x18;
    // char dev_chan = 0x00;
    // char dev_option = 0x40;
    char set_para[7] = {dev_head, dev_addh, dev_addl, dev_sped, dev_chan, dev_option, '\0'};
    e34_2g4d20d_model_sel(SET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    e34_2g4d20d_sendData(E34_2G4D20D_TAG, (char *)set_para, 6);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    e34_2g4d20d_sendData(E34_2G4D20D_TAG, read_parameter, 3);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    e34_2g4d20d_model_sel(FULL_DUPLEX);
}

void e34_2g4d20d_reset()
{
    e34_2g4d20d_model_sel(SET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    e34_2g4d20d_sendData(E34_2G4D20D_TAG, reset_device, 3);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    e34_2g4d20d_model_sel(FULL_DUPLEX);
}
