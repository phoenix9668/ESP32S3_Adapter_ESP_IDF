#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "freertos/portmacro.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "ch9434.h"
#include "e34_2g4d20d.h"

#define TAG "MAIN"
SemaphoreHandle_t mutex;
QueueHandle_t tableQueue;
/*
 * LED pins definition
 */
#define LED_GREEN_PIN (GPIO_NUM_8)
#define LED_BLUE_PIN (GPIO_NUM_9)

/*
 * UART2 pins and parameters definition
 */
#define UART2_TXD_PIN (GPIO_NUM_48)
#define UART2_RXD_PIN (GPIO_NUM_38)
uint8_t *packet_to_android = NULL;
#define RX_BUF_SIZE 512
typedef struct
{
    uint8_t data[RX_BUF_SIZE];
    size_t length;
} table_data_t;

/*
 * CH9434 parameters definition
 */
uint8_t uart_idx;
uint8_t uart_iir;
uint8_t uart_lsr;
uint8_t uart_msr;

uint16_t rec_buf_cnt = 0;
uint8_t uart_rec_buf[512];

uint32_t uart_rec_total_cnt[4] = {0, 0, 0, 0};

uint8_t ch9434_init_end = 0;

/*
 * business pins and parameters definition
 */
// 定义包头和地址
#define PACKET_HEADER 0xE55E
#define PACKET_TO_ANDROID_LENGTH 6
uint8_t board_address = 0x00;

#define ADDR1 (GPIO_NUM_42)
#define ADDR2 (GPIO_NUM_41)
#define ADDR3 (GPIO_NUM_40)
#define ADDR4 (GPIO_NUM_39)

// CRC32多项式
#define CRC32_POLYNOMIAL 0xEDB88320L

// 计算CRC32查找表
uint32_t crc32_table[256];

void generate_crc32_table()
{
    for (uint32_t i = 0; i < 256; ++i)
    {
        uint32_t crc = i;
        for (int j = 0; j < 8; ++j)
        {
            crc = (crc >> 1) ^ ((crc & 1) ? CRC32_POLYNOMIAL : 0);
        }
        crc32_table[i] = crc;
    }
}

// 计算CRC32
uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; ++i)
    {
        crc = (crc >> 8) ^ crc32_table[(crc & 0xFF) ^ data[i]];
    }

    return ~crc;
}

// 计算Modbus RTU CRC16
uint16_t calculateModbusCRC16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

// 初始化LED
void led_gpio_init(void)
{
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (1ULL << LED_GREEN_PIN) | (1ULL << LED_BLUE_PIN);
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_conf);

    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_BLUE_PIN, 0);
}

// get switch value on PCB
void get_switch_value(void)
{
    static const char *SWITCH_TAG = "SWITCH";
    esp_log_level_set(SWITCH_TAG, ESP_LOG_INFO);
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1ULL << ADDR1) | (1ULL << ADDR2) | (1ULL << ADDR3) | (1ULL << ADDR4);
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_conf);

    board_address = (gpio_get_level(ADDR4) << 3) | (gpio_get_level(ADDR3) << 2) | (gpio_get_level(ADDR2) << 1) | gpio_get_level(ADDR1);
    ESP_LOGI(SWITCH_TAG, "board_address:%02x", board_address);
    for (int i = 0; i < board_address; i++)
    {
        gpio_set_level(LED_GREEN_PIN, 1);
        gpio_set_level(LED_BLUE_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GREEN_PIN, 0);
        gpio_set_level(LED_BLUE_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void uart2_init(void)
{
    const uart_config_t uart2_config = {
        .baud_rate = 115200,
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
        // sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_DEBUG);
    uint8_t cnt = 0;
    while (1)
    {
        table_data_t table_data;
        size_t from_table_rxBytes = uart_read_bytes(UART_NUM_2, table_data.data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GREEN_PIN, 0);
        if (from_table_rxBytes > 0)
        {
            table_data.length = from_table_rxBytes;
            table_data.data[table_data.length] = 0;
            ESP_LOGD(RX_TASK_TAG, "Read %d bytes: '%s'", table_data.length, table_data.data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, table_data.data, table_data.length, ESP_LOG_DEBUG);
            // Send data to queue
            if (xQueueSend(tableQueue, &table_data, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(RX_TASK_TAG, "Failed to send data to queue");
            }
            cnt++;
            if (cnt >= 5)
            {
                gpio_set_level(LED_GREEN_PIN, 1);
                cnt = 0;
            }
        }
        memset(table_data.data, 0, RX_BUF_SIZE);
    }
}

static void e34_2g4d20d_reset_task(void *arg)
{
    while (1)
    {
        e34_2g4d20d_reset();
        vTaskDelay(600000 / portTICK_PERIOD_MS);
    }
}

static void e34_2g4d20d_tx_task(void *arg)
{
    static const char *E34_2G4D20D_TX_TASK_TAG = "E34_2G4D20D_TX_TASK";
    esp_log_level_set(E34_2G4D20D_TX_TASK_TAG, ESP_LOG_DEBUG);
    // 在开头插入包头和地址
    packet_to_android = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    uint32_t crc32_result;
    uint8_t cnt = 0;
    while (1)
    {
        // e34_2g4d20d_sendData(E34_2G4D20D_TX_TASK_TAG, "Hello world", strlen("Hello world"));
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        table_data_t table_data;
        if (xQueueReceive(tableQueue, &table_data, portMAX_DELAY) == pdPASS)
        {
            table_data.data[table_data.length] = 0;
            ESP_LOGD(E34_2G4D20D_TX_TASK_TAG, "Read from queue %d bytes: '%s'", table_data.length, table_data.data);
            ESP_LOG_BUFFER_HEXDUMP(E34_2G4D20D_TX_TASK_TAG, table_data.data, table_data.length, ESP_LOG_DEBUG);
            if (table_data.data[table_data.length - 1] == 0x0d || (table_data.data[table_data.length - 1] == 0x0a && table_data.data[table_data.length - 2] == 0x0d))
            {
                cnt++;
                if (cnt >= 5)
                {
                    gpio_set_level(LED_BLUE_PIN, 1);
                    cnt = 0;
                }
                packet_to_android[0] = (PACKET_HEADER >> 8) & 0xFF;
                packet_to_android[1] = PACKET_HEADER & 0xFF;
                packet_to_android[2] = 0x00;
                packet_to_android[3] = board_address & 0xFF;
                packet_to_android[4] = 0x00;
                packet_to_android[5] = 0x01; // 称重数据为01，RFID数据为02
                // 复制原始数据到新数组
                for (int i = 0; i < table_data.length; ++i)
                {
                    packet_to_android[i + 6] = table_data.data[i];
                }

                crc32_result = calculateCRC32(packet_to_android, table_data.length + 6);
                // 计算并追加CRC校验码
                packet_to_android[table_data.length + 6] = (crc32_result >> 24) & 0xFF;
                packet_to_android[table_data.length + 7] = (crc32_result >> 16) & 0xFF;
                packet_to_android[table_data.length + 8] = (crc32_result >> 8) & 0xFF;
                packet_to_android[table_data.length + 9] = crc32_result & 0xFF;
                if (xSemaphoreTake(mutex, portMAX_DELAY))
                {
                    e34_2g4d20d_sendData(E34_2G4D20D_TX_TASK_TAG, (char *)packet_to_android, table_data.length + 10);
                    xSemaphoreGive(mutex);
                }
            }
            else if (table_data.data[0] == 0x01 && table_data.data[1] == 0x03)
            {
                // Modbus RTU数据处理：根据第三个字节判断格式
                int modbus_packet_len = 0;
                uint8_t *modbus_packet = NULL;
                
                // 第一种格式: 01 03 04 xx xx xx xx [CRC16] - 长度 9 字节
                if (table_data.length >= 9 && table_data.data[2] == 0x04)
                {
                    modbus_packet_len = 9;
                    modbus_packet = table_data.data;
                }
                // 第二种格式: 01 03 2b xx ... [CRC16] - 长度 12 字节
                else if (table_data.length >= 12 && (table_data.data[2] == 0x2b || table_data.data[2] == 0x2d))
                {
                    modbus_packet_len = 12;
                    modbus_packet = table_data.data;
                }
                
                if (modbus_packet_len > 0 && modbus_packet != NULL)
                {
                    // 计算CRC16（不包含最后2个字节的CRC）
                    uint16_t calculated_crc = calculateModbusCRC16(modbus_packet, modbus_packet_len - 2);
                    // Modbus RTU的CRC是低字节在前，高字节在后
                    uint16_t received_crc = (modbus_packet[modbus_packet_len - 1] << 8) | modbus_packet[modbus_packet_len - 2];
                    
                    if (calculated_crc == received_crc)
                    {
                        // CRC校验通过，转发完整的Modbus RTU数据包（包含CRC）
                        if (xSemaphoreTake(mutex, portMAX_DELAY))
                        {
                            e34_2g4d20d_sendData(E34_2G4D20D_TX_TASK_TAG, (char *)modbus_packet, modbus_packet_len);
                            xSemaphoreGive(mutex);
                        }
                        ESP_LOGI(E34_2G4D20D_TX_TASK_TAG, "Sent Modbus RTU data: %d bytes", modbus_packet_len);
                        ESP_LOG_BUFFER_HEXDUMP(E34_2G4D20D_TX_TASK_TAG, modbus_packet, modbus_packet_len, ESP_LOG_INFO);
                    }
                    else
                    {
                        ESP_LOGW(E34_2G4D20D_TX_TASK_TAG, "Modbus CRC error: calculated=0x%04X, received=0x%04X", calculated_crc, received_crc);
                        ESP_LOG_BUFFER_HEXDUMP(E34_2G4D20D_TX_TASK_TAG, modbus_packet, modbus_packet_len, ESP_LOG_WARN);
                    }
                }
                else
                {
                    ESP_LOGW(E34_2G4D20D_TX_TASK_TAG, "Unknown Modbus format, byte[2]=0x%02X, length=%d", table_data.data[2], table_data.length);
                    ESP_LOG_BUFFER_HEXDUMP(E34_2G4D20D_TX_TASK_TAG, table_data.data, table_data.length, ESP_LOG_WARN);
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(LED_BLUE_PIN, 0);
    }
}

static void e34_2g4d20d_rx_task(void *arg)
{
    static const char *E34_2G4D20D_RX_TASK_TAG = "E34_2G4D20D_RX_TASK";
    esp_log_level_set(E34_2G4D20D_RX_TASK_TAG, ESP_LOG_DEBUG);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    uint8_t rfid_one_shot[5] = {0x04, 0xFF, 0x01, 0x1B, 0xB4};
    uint32_t crc32_result;
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 200 / portTICK_PERIOD_MS);
        if (rxBytes > 4)
        {
            data[rxBytes] = 0;
            ESP_LOGD(E34_2G4D20D_RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(E34_2G4D20D_RX_TASK_TAG, data, rxBytes, ESP_LOG_DEBUG);
            if (ch9434_init_end)
            {
                crc32_result = calculateCRC32(data, rxBytes - 4);
                if (data[rxBytes - 4] == ((crc32_result >> 24) & 0xFF) && data[rxBytes - 3] == ((crc32_result >> 16) & 0xFF) && data[rxBytes - 2] == ((crc32_result >> 8) & 0xFF) && data[rxBytes - 1] == (crc32_result & 0xFF))
                {
                    if ((data[0] == ((PACKET_HEADER >> 8) & 0xFF)) && (data[1] == (PACKET_HEADER & 0xFF)) && (data[2] == 0x00) && (data[3] == (board_address & 0xFF)) && (data[4] == 0x00) && (data[5] == 0x02))
                    {
                        for (int i = 0; i < sizeof(rfid_one_shot); ++i)
                        {
                            rfid_one_shot[i] = data[i + 6];
                        }

                        CH9434UARTxSetTxFIFOData(2, rfid_one_shot, sizeof(rfid_one_shot));
                    }
                    else if ((data[0] == ((PACKET_HEADER >> 8) & 0xFF)) && (data[1] == (PACKET_HEADER & 0xFF)) && (data[2] == 0x00) && (data[3] == (board_address & 0xFF)) && (data[4] == 0x00) && (data[5] == 0x01))
                    {
                        int weight_data_len = rxBytes - 4 - 6; // 去掉帧头(6字节)和CRC(4字节)
                        if (weight_data_len > 0)
                        {
                            const int txBytes = uart_write_bytes(UART_NUM_2, &data[6], weight_data_len);
                            ESP_LOGD(E34_2G4D20D_RX_TASK_TAG, "Sent weight data: %d bytes", txBytes);
                            ESP_LOG_BUFFER_HEXDUMP(E34_2G4D20D_RX_TASK_TAG, &data[6], weight_data_len, ESP_LOG_DEBUG);
                        }
                    }
                }
            }
        }
    }
    free(data);
}

static void ch9434_task(void *arg)
{
    static const char *CH9434_TASK_TAG = "CH9434_TASK";
    esp_log_level_set(CH9434_TASK_TAG, ESP_LOG_DEBUG);
    ESP_LOGD(CH9434_TASK_TAG, "** CH9434 hardware test demo. ** \n");
    /* init CH9434 */
    CH9434InitClkMode(CH9434_ENABLE, // extern Crystal oscillator
                      CH9434_ENABLE, // enable frequency doubling
                      13);           // Frequency division coefficient
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // init uart0
    CH9434UARTxInit(CH9434_UART_IDX_0, UART_BPS);
    // init uart1
    CH9434UARTxInit(CH9434_UART_IDX_1, UART_BPS);
    // init uart2
    CH9434UARTxInit(CH9434_UART_IDX_2, UART_BPS);
    // init uart3
    CH9434UARTxInit(CH9434_UART_IDX_3, UART_BPS);

    ch9434_init_end = 1;
    packet_to_android = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    uint32_t crc32_result;
    while (1)
    {
        if (gpio_get_level(CH9434_INT) == 0) // INT level is low
        {
            for (uart_idx = 0; uart_idx < 4; uart_idx++)
            {
                uart_iir = CH9434UARTxReadIIR(uart_idx);
                ESP_LOGD(CH9434_TASK_TAG, "idx:%d uart_iir:%02x", uart_idx, uart_iir);
                switch (uart_iir & 0x0f)
                {
                case 0x01: // no interrupt
                    break;
                case 0x06: // receive line status
                {
                    uart_lsr = CH9434UARTxReadLSR(uart_idx);
                    ESP_LOGD(CH9434_TASK_TAG, "uart_lsr:%02x", uart_lsr);
                    rec_buf_cnt = CH9434UARTxGetRxFIFOLen(uart_idx);
                    if (rec_buf_cnt)
                    {
                        CH9434UARTxGetRxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
                        uart_rec_total_cnt[uart_idx] += rec_buf_cnt;
                        ESP_LOGD(CH9434_TASK_TAG, "idx:%d rec:%d total:%d", uart_idx, rec_buf_cnt,
                                 (int)uart_rec_total_cnt[uart_idx]);
                        // CH9434UARTxSetTxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
                        if (rec_buf_cnt >= 20)
                        {
                            packet_to_android[0] = (PACKET_HEADER >> 8) & 0xFF;
                            packet_to_android[1] = PACKET_HEADER & 0xFF;
                            packet_to_android[2] = 0x00;
                            packet_to_android[3] = board_address & 0xFF;
                            packet_to_android[4] = 0x00;
                            packet_to_android[5] = 0x02; // 称重数据为01，RFID数据为02
                            // 复制原始数据到新数组
                            for (int i = 0; i < rec_buf_cnt; ++i)
                            {
                                packet_to_android[i + 6] = uart_rec_buf[i];
                            }

                            crc32_result = calculateCRC32(packet_to_android, rec_buf_cnt + 6);
                            // 计算并追加CRC校验码
                            packet_to_android[rec_buf_cnt + 6] = (crc32_result >> 24) & 0xFF;
                            packet_to_android[rec_buf_cnt + 7] = (crc32_result >> 16) & 0xFF;
                            packet_to_android[rec_buf_cnt + 8] = (crc32_result >> 8) & 0xFF;
                            packet_to_android[rec_buf_cnt + 9] = crc32_result & 0xFF;
                            if (xSemaphoreTake(mutex, portMAX_DELAY))
                            {
                                e34_2g4d20d_sendData(CH9434_TASK_TAG, (char *)packet_to_android, rec_buf_cnt + 10);
                                xSemaphoreGive(mutex);
                            }
                        }
                    }
                    break;
                }
                case 0x04: // receive data available
                {
                    rec_buf_cnt = CH9434UARTxGetRxFIFOLen(uart_idx);
                    if (rec_buf_cnt)
                    {
                        CH9434UARTxGetRxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
                        uart_rec_total_cnt[uart_idx] += rec_buf_cnt;
                        ESP_LOGD(CH9434_TASK_TAG, "idx:%d rec:%d total:%d", uart_idx, rec_buf_cnt,
                                 (int)uart_rec_total_cnt[uart_idx]);
                        // CH9434UARTxSetTxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
                        if (rec_buf_cnt >= 20)
                        {
                            packet_to_android[0] = (PACKET_HEADER >> 8) & 0xFF;
                            packet_to_android[1] = PACKET_HEADER & 0xFF;
                            packet_to_android[2] = 0x00;
                            packet_to_android[3] = board_address & 0xFF;
                            packet_to_android[4] = 0x00;
                            packet_to_android[5] = 0x02; // 称重数据为01，RFID数据为02
                            // 复制原始数据到新数组
                            for (int i = 0; i < rec_buf_cnt; ++i)
                            {
                                packet_to_android[i + 6] = uart_rec_buf[i];
                            }

                            crc32_result = calculateCRC32(packet_to_android, rec_buf_cnt + 6);
                            // 计算并追加CRC校验码
                            packet_to_android[rec_buf_cnt + 6] = (crc32_result >> 24) & 0xFF;
                            packet_to_android[rec_buf_cnt + 7] = (crc32_result >> 16) & 0xFF;
                            packet_to_android[rec_buf_cnt + 8] = (crc32_result >> 8) & 0xFF;
                            packet_to_android[rec_buf_cnt + 9] = crc32_result & 0xFF;
                            if (xSemaphoreTake(mutex, portMAX_DELAY))
                            {
                                e34_2g4d20d_sendData(CH9434_TASK_TAG, (char *)packet_to_android, rec_buf_cnt + 10);
                                xSemaphoreGive(mutex);
                            }
                        }
                    }
                    break;
                }
                case 0x0C: // receive data timeout
                {
                    rec_buf_cnt = CH9434UARTxGetRxFIFOLen(uart_idx);
                    if (rec_buf_cnt)
                    {
                        CH9434UARTxGetRxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
                        uart_rec_total_cnt[uart_idx] += rec_buf_cnt;
                        ESP_LOGD(CH9434_TASK_TAG, "idx:%d rec:%d total:%d", uart_idx, rec_buf_cnt,
                                 (int)uart_rec_total_cnt[uart_idx]);
                        // CH9434UARTxSetTxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
                        if (rec_buf_cnt >= 20)
                        {
                            packet_to_android[0] = (PACKET_HEADER >> 8) & 0xFF;
                            packet_to_android[1] = PACKET_HEADER & 0xFF;
                            packet_to_android[2] = 0x00;
                            packet_to_android[3] = board_address & 0xFF;
                            packet_to_android[4] = 0x00;
                            packet_to_android[5] = 0x02; // 称重数据为01，RFID数据为02
                            // 复制原始数据到新数组
                            for (int i = 0; i < rec_buf_cnt; ++i)
                            {
                                packet_to_android[i + 6] = uart_rec_buf[i];
                            }

                            crc32_result = calculateCRC32(packet_to_android, rec_buf_cnt + 6);
                            // 计算并追加CRC校验码
                            packet_to_android[rec_buf_cnt + 6] = (crc32_result >> 24) & 0xFF;
                            packet_to_android[rec_buf_cnt + 7] = (crc32_result >> 16) & 0xFF;
                            packet_to_android[rec_buf_cnt + 8] = (crc32_result >> 8) & 0xFF;
                            packet_to_android[rec_buf_cnt + 9] = crc32_result & 0xFF;
                            if (xSemaphoreTake(mutex, portMAX_DELAY))
                            {
                                e34_2g4d20d_sendData(CH9434_TASK_TAG, (char *)packet_to_android, rec_buf_cnt + 10);
                                xSemaphoreGive(mutex);
                            }
                        }
                    }
                    break;
                }
                case 0x02: // THR register empty
                    break;
                case 0x00: // modem signal change
                {
                    uart_msr = CH9434UARTxReadMSR(uart_idx);
                    ESP_LOGD(CH9434_TASK_TAG, "uart_msr:%02x", uart_msr);
                    break;
                }
                }
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    tableQueue = xQueueCreate(20, sizeof(table_data_t));
    if (tableQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }
    generate_crc32_table();
    led_gpio_init();
    get_switch_value();
    e34_2g4d20d_uart1_init();
    e34_2g4d20d_gpio_init();
    e34_2g4d20d_parameter_set(0xc0, 0x00, 0x00, 0x18, 0x00, 0x40);
    uart2_init();
    ch9434_spi2_init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(e34_2g4d20d_rx_task, "e34_2g4d20d_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(e34_2g4d20d_tx_task, "e34_2g4d20d_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(e34_2g4d20d_reset_task, "e34_2g4d20d_reset_task", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(ch9434_task, "ch9434_task", 1024 * 8, NULL, configMAX_PRIORITIES - 3, NULL);
}
