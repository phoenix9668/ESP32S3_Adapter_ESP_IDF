#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "freertos/portmacro.h"
#include "driver/spi_master.h"
#include "ch9434.h"
#include "esp_rom_sys.h"

/*
 * UART pins and parameters definition
 */
static const int RX_BUF_SIZE = 1024;
uint8_t result_data[100];
int result_index;
char read_parameter[4] = {0xC1, 0xC1, 0xC1, '\0'};
char read_version[4] = {0xC3, 0xC3, 0xC3, '\0'};
char reset_device[4] = {0xC4, 0xC4, 0xC4, '\0'};

#define UART1_TXD_PIN (GPIO_NUM_6)
#define UART1_RXD_PIN (GPIO_NUM_7)
#define UART2_TXD_PIN (GPIO_NUM_48)
#define UART2_RXD_PIN (GPIO_NUM_38)

#define M0 (GPIO_NUM_5)
#define M1 (GPIO_NUM_4)
#define AUX (GPIO_NUM_15)

#define ADDR1 (GPIO_NUM_42)
#define ADDR2 (GPIO_NUM_41)
#define ADDR3 (GPIO_NUM_40)
#define ADDR4 (GPIO_NUM_39)

typedef enum
{
    HALF_DUPLEX = 0,
    FULL_DUPLEX = 1,
    RESERVE = 2,
    SET = 3
} modes_t;

/*
 * CH9434 SPI parameters definition
 */
uint8_t uart_idx;
uint8_t uart_iir;
uint8_t uart_lsr;
uint8_t uart_msr;

uint16_t rec_buf_cnt = 0;
uint8_t uart_rec_buf[512];

uint32_t uart_rec_total_cnt[4] = {0, 0, 0, 0};

#ifndef u8_t
typedef unsigned char u8_t;
#endif
#ifndef u16_t
typedef unsigned short u16_t;
#endif
#ifndef u32_t
typedef unsigned long u32_t;
#endif

/*
 * Function Name  : CH9434InitClkMode
 * Description    : CH9434芯片时钟模式设置
 * Input          : xt_en：外部晶振使能
 *                  freq_mul_en：倍频功能使能
 *                  div_num：分频系数
 * Output         : None
 * Return         : None
 */
void CH9434InitClkMode(u8_t xt_en, u8_t freq_mul_en, u8_t div_num)
{
    uint8_t clk_ctrl_reg;
    u16_t i;
    uint8_t data[1] = {0x00};
    spi_transaction_t t = {
        .tx_buffer = data,
        .length = 1 * 8};

    clk_ctrl_reg = 0;
    if (freq_mul_en)
        clk_ctrl_reg |= (1 << 7);
    if (xt_en)
        clk_ctrl_reg |= (1 << 6);
    clk_ctrl_reg |= (div_num & 0x1f);

    /* ���㵱ǰ�Ĵ��ڻ�׼ʱ�� */
    // sys_frequency
    switch (clk_ctrl_reg & 0xc0)
    {
    case 0x00: // �ڲ�32M�ṩʱ��
        sys_frequency = 32000000;
        break;
    case 0x40:                                                              // �ⲿ�����ṩʱ��
        if ((osc_xt_frequency > 36000000) || (osc_xt_frequency < 24000000)) // ʱ�Ӵ���
        {
            return;
        }
        sys_frequency = osc_xt_frequency;
        break;
    case 0x80: // ʹ���ڲ�32M����������Ƶ
        sys_frequency = 480000000 / (div_num & 0x1f);
        if (sys_frequency > 40000000) // ʱ�Ӵ���
        {
            sys_frequency = 32000000;
            return;
        }
        break;
    case 0xc0:                                                              // ʹ���ⲿ���񣬲�������Ƶ
        if ((osc_xt_frequency > 36000000) || (osc_xt_frequency < 24000000)) // ʱ�Ӵ���
        {
            return;
        }
        sys_frequency = osc_xt_frequency * 15 / (div_num & 0x1f);
        if (sys_frequency > 40000000) // ʱ�Ӵ���
        {
            sys_frequency = 32000000;
            return;
        }
        break;
    }

    // CH9434_SPI_SCS_OP(CH9434_DISABLE);
    data[0] = CH9434_REG_OP_WRITE | CH9434_CLK_CTRL_CFG_ADD;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
    // CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE | CH9434_CLK_CTRL_CFG_ADD);
    CH9434_US_DELAY();
    data[0] = clk_ctrl_reg;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
    // CH9434_SPI_WRITE_BYTE(clk_ctrl_reg);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    data[0] = CH9434_REG_OP_READ | CH9434_CLK_CTRL_CFG_ADD;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    // CH9434_SPI_SCS_OP(CH9434_ENABLE);
    // for (i = 0; i < 50000; i++)
    //     CH9434_US_DELAY();
}

esp_err_t spi_eeprom_read(void)
{
    uint8_t data[1] = {0xFF};
    spi_transaction_t t = {
        .length = 8,
        // .addr = CH9434_REG_OP_READ | CH9434_CLK_CTRL_CFG_ADD,
        .rxlength = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    esp_err_t err = spi_device_polling_transmit(spi2, &t);
    if (err != ESP_OK)
        return err;

    ESP_LOGI(TAG, "Receive data :%d...", t.rx_data[0]);
    return ESP_OK;
}

void uart2_init(void)
{
    const uart_config_t uart2_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart2_config);
    uart_set_pin(UART_NUM_2, UART2_TXD_PIN, UART2_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1)
    {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            memset(result_data, 0, sizeof(result_data));
            memset(&result_index, 0, sizeof(result_index));
            // 查找第一个0x20
            char *start_ptr = strchr((char *)data, 0x20);
            if (start_ptr == NULL)
            {
                printf("No start byte (0x20) found.\n");
            }

            // 查找第一个0x0d，从第一个0x20开始搜索
            char *end_ptr = strchr(start_ptr, 0x0d);
            if (end_ptr == NULL)
            {
                printf("No end byte (0x0d) found.\n");
            }

            // 复制数据到结果数组
            while (start_ptr != end_ptr)
            {
                result_data[result_index++] = *start_ptr++;
            }

            // 将提取的数据打印为字符串
            result_data[result_index++] = 0x0d; // 添加字符串结束符
            result_data[result_index] = '\0';   // 添加字符串结束符
            printf("Extracted Data: %s\n", result_data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, result_data, result_index, ESP_LOG_INFO);
        }
    }
    free(data);
}

void uart1_init(void)
{
    const uart_config_t uart1_config = {
        .baud_rate = 9600,
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

void e34_2g4d20d_init(void)
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

// void e34_2g4d20d_parameter_set(char *read_parameter)
// {
//     char dev_head = 0xc0;
//     char dev_addh = 0x00;
//     char dev_addl = 0x00;
//     char dev_sped = 0x18;
//     char dev_chan = 0x00;
//     char dev_option = 0x40;
//     char *read_parameter = {dev_head, dev_addh, dev_addl, dev_sped, dev_chan, dev_option, '\0'};
// }

int e34_2g4d20d_sendData(const char *logName, const char *data, int len)
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    ESP_LOG_BUFFER_HEXDUMP(logName, data, len, ESP_LOG_INFO);
    return txBytes;
}

static void e34_2g4d20d_tx_task(void *arg)
{
    static const char *E34_2G4D20D_TX_TASK_TAG = "E34_2G4D20D_TX_TASK";
    esp_log_level_set(E34_2G4D20D_TX_TASK_TAG, ESP_LOG_INFO);
    e34_2g4d20d_model_sel(SET);
    char set_para[7] = {0xC0, 0x00, 0x00, 0x18, 0x00, 0x40, '\0'};
    e34_2g4d20d_sendData(E34_2G4D20D_TX_TASK_TAG, (char *)set_para, 6);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    e34_2g4d20d_sendData(E34_2G4D20D_TX_TASK_TAG, read_parameter, 3);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    e34_2g4d20d_model_sel(FULL_DUPLEX);
    while (1)
    {
        e34_2g4d20d_sendData(E34_2G4D20D_TX_TASK_TAG, (char *)result_data, result_index);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

static void e34_2g4d20d_rx_task(void *arg)
{
    static const char *E34_2G4D20D_RX_TASK_TAG = "E34_2G4D20D_RX_TASK";
    esp_log_level_set(E34_2G4D20D_RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(E34_2G4D20D_RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(E34_2G4D20D_RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

static void spi_tx_task(void *arg)
{
    static const char *SPI_TX_TASK_TAG = "SPI_TX_TASK";
    esp_log_level_set(SPI_TX_TASK_TAG, ESP_LOG_INFO);
    // uint8_t data[2] = {0x44, 0x66};

    // spi_transaction_t t = {
    //     .tx_buffer = data,
    //     .length = 2 * 8};
    while (1)
    {
        // ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        /* SPI transmit test */
        /* init CH9434 */
        CH9434InitClkMode(CH9434_ENABLE, // extern Crystal oscillator
                          CH9434_ENABLE, // enable frequency doubling
                          13);           // Frequency division coefficient
        spi_eeprom_read();
        // vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    uart1_init();
    uart2_init();
    e34_2g4d20d_init();
    ch9434_rst_init();
    // spi2_init();
    // xTaskCreate(rx_task, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
    // xTaskCreate(e34_2g4d20d_rx_task, "e34_2g4d20d_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(e34_2g4d20d_tx_task, "e34_2g4d20d_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
    // xTaskCreate(spi_tx_task, "spi_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
}
