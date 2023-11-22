/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH9434.c
 * Author             : tech18
 * Version            : V1.0
 * Date               : 2020/05/08
 * Description        : CH9434 deriver file
 *******************************************************************************/

#include "ch9434.h"
#include "esp_log.h"
#include "string.h"

/*
 * CH9434 SPI parameters definition
 */
#define SPI_CLK_FREQ (1 * 1000 * 1000)
static const char *TAG = "CH9434_SPI";
spi_device_handle_t spi2;

uint32_t osc_xt_frequency = 32000000;

uint32_t sys_frequency = 32000000;

uint8_t lower_power_reg = 0;

uint32_t ch9434_gpio_x_val = 0;

/* -----------------------------------------------------------------------------
 *                      define CH9434 interface functions
 * -----------------------------------------------------------------------------
 */

static void ch9434_rst_gpio_init(void)
{
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (1ULL << CH9434_RST);
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_conf);

    gpio_set_level(CH9434_RST, 1);
}

static void ch9434_cs_gpio_init(void)
{
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (1ULL << PIN_NUM_CS);
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_conf);

    gpio_set_level(PIN_NUM_CS, 1);
}

static void ch9434_int_gpio_init(void)
{
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1ULL << CH9434_INT);
    gpio_conf.pull_up_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpio_conf);
}

static void cs_high(void)
{
    gpio_set_level(PIN_NUM_CS, 1);
}

static void cs_low(spi_transaction_t *t)
{
    gpio_set_level(PIN_NUM_CS, 0);
}

/*
 * Function Name  : spi2_init
 * Description    : 初始化spi2外设
 * Input          : None
 * Output         : None
 * Return         : None
 */
void ch9434_spi2_init(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", CH9434_HOST + 1);
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(CH9434_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg = {
        // .address_bits = 8,
        // .dummy_bits = 3,
        .clock_speed_hz = SPI_CLK_FREQ, // 1 MHz
        .mode = 0,                      // SPI mode 0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        // .flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = cs_low,
        .post_cb = NULL};

    ESP_ERROR_CHECK(spi_bus_add_device(CH9434_HOST, &devcfg, &spi2));

    ch9434_cs_gpio_init();
    ch9434_rst_gpio_init();
    ch9434_int_gpio_init();
}

void CH9434_US_DELAY(void)
{
    esp_rom_delay_us(1);
}

/* SPI SCS pin control, 0:low level  1:high level */
void CH9434_SPI_SCS_OP(uint8_t byte)
{
    if (byte)
        gpio_set_level(PIN_NUM_CS, 1); // SCS high
    else
        gpio_set_level(PIN_NUM_CS, 0); // SCS low
}

uint8_t spi_read_write_byte(uint8_t addr, uint8_t byte)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    // t.addr = addr;
    t.length = 8;
    t.tx_buffer = &addr; // Data
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void *)1;                                     // D/C needs to be set to 1
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t)); // Transmit!
    t.tx_buffer = &byte;                                    // Data
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t)); // Transmit!
    cs_high();

    return t.rx_data[0];
}

/* -----------------------------------------------------------------------------
 *                                  api function
 * -----------------------------------------------------------------------------
 */

/*
 * Function Name  : CH9434OscXtFreqSet
 * Description    : 外部晶振频率记录
 * Input          : x_freq：当前芯片连接的晶振频率
 * Output         : None
 * Return         : None
 */
void CH9434OscXtFreqSet(uint32_t x_freq)
{
    osc_xt_frequency = x_freq;
}

/*
 * Function Name  : CH9434InitClkMode
 * Description    : CH9434芯片时钟模式设置
 * Input          : xt_en：外部晶振使能
 *                  freq_mul_en：倍频功能使能
 *                  div_num：分频系数
 * Output         : None
 * Return         : None
 */
void CH9434InitClkMode(uint8_t xt_en, uint8_t freq_mul_en, uint8_t div_num)
{
    uint8_t clk_ctrl_reg;
    uint16_t i;

    clk_ctrl_reg = 0;
    if (freq_mul_en)
        clk_ctrl_reg |= (1 << 7);
    if (xt_en)
        clk_ctrl_reg |= (1 << 6);
    clk_ctrl_reg |= (div_num & 0x1f);

    // sys_frequency
    switch (clk_ctrl_reg & 0xc0)
    {
    case 0x00:
        sys_frequency = 32000000;
        break;
    case 0x40:
        if ((osc_xt_frequency > 36000000) || (osc_xt_frequency < 24000000))
        {
            return;
        }
        sys_frequency = osc_xt_frequency;
        break;
    case 0x80:
        sys_frequency = 480000000 / (div_num & 0x1f);
        if (sys_frequency > 40000000)
        {
            sys_frequency = 32000000;
            return;
        }
        break;
    case 0xc0:
        if ((osc_xt_frequency > 36000000) || (osc_xt_frequency < 24000000))
        {
            return;
        }
        sys_frequency = osc_xt_frequency * 15 / (div_num & 0x1f);
        if (sys_frequency > 40000000)
        {
            sys_frequency = 32000000;
            return;
        }
        break;
    }

    spi_read_write_byte(CH9434_REG_OP_WRITE | CH9434_CLK_CTRL_CFG_ADD, clk_ctrl_reg);
    for (i = 0; i < 50000; i++)
        CH9434_US_DELAY();
    ESP_LOGI(TAG, "CLK_CTRL = %02X", spi_read_write_byte(CH9434_REG_OP_READ | CH9434_CLK_CTRL_CFG_ADD, 0xFF));
}

/*
 * Function Name  : CH9434UARTxParaSet
 * Description    : 串口参数设置
 * Input          : uart_idx：串口号
 *                  bps：串口的波特率
 *                  data_bits：数据位
 *                  stop_bits：停止位
 *                  veri_bits：校验位
 * Output         : None
 * Return         : None
 */
void CH9434UARTxParaSet(uint8_t uart_idx, uint32_t bps, uint8_t data_bits, uint8_t stop_bits, uint8_t veri_bits)
{
    uint8_t uart_reg_dll;
    uint8_t uart_reg_dlm;
    uint32_t x;
    uint8_t uart_reg_lcr;

    x = 10 * sys_frequency / 8 / bps;
    x = (x + 5) / 10;

    uart_reg_dll = x & 0xff;
    uart_reg_dlm = (x >> 8) & 0xff;

    uart_reg_lcr = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_LCR_ADD + 0x10 * uart_idx), 0xFF);

    ESP_LOGI(TAG, "LCR 1 = %02X", uart_reg_lcr);

    uart_reg_lcr |= CH9434_UARTx_BIT_DLAB;

    uart_reg_lcr &= ~0x03;
    switch (data_bits)
    {
    case CH9434_UART_5_BITS_PER_CHAR:
        break;
    case CH9434_UART_6_BITS_PER_CHAR:
        uart_reg_lcr |= 0x01;
        break;
    case CH9434_UART_7_BITS_PER_CHAR:
        uart_reg_lcr |= 0x02;
        break;
    case CH9434_UART_8_BITS_PER_CHAR:
        uart_reg_lcr |= 0x03;
        break;
    default:
        uart_reg_lcr |= 0x03;
        break;
    }

    ESP_LOGI(TAG, "LCR 2 = %02X", uart_reg_lcr);

    uart_reg_lcr &= ~(1 << 2);
    if (stop_bits == CH9434_UART_TWO_STOP_BITS)
    {
        uart_reg_lcr |= (1 << 2);
    }

    uart_reg_lcr &= ~(1 << 3);
    uart_reg_lcr &= ~(3 << 4);
    switch (veri_bits)
    {
    case CH9434_UART_NO_PARITY:
        break;
    case CH9434_UART_ODD_PARITY:
        uart_reg_lcr |= (1 << 3);
        break;
    case CH9434_UART_EVEN_PARITY:
        uart_reg_lcr |= (1 << 3);
        uart_reg_lcr |= (1 << 4);
        break;
    case CH9434_UART_MARK_PARITY:
        uart_reg_lcr |= (1 << 3);
        uart_reg_lcr |= (2 << 4);
        break;
    case CH9434_UART_SPACE_PARITY:
        uart_reg_lcr |= (1 << 3);
        uart_reg_lcr |= (3 << 4);
        break;
    default:
        break;
    }
    ESP_LOGI(TAG, "LCR 3 = %02X", uart_reg_lcr);

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_LCR_ADD + 0x10 * uart_idx), uart_reg_lcr);
    ESP_LOGI(TAG, "LCR_ADD 1 = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_LCR_ADD + 0x10 * uart_idx), 0xFF));

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_DLL_ADD + 0x10 * uart_idx), uart_reg_dll);
    ESP_LOGI(TAG, "DLL_ADD = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_DLL_ADD + 0x10 * uart_idx), 0xFF));

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_DLM_ADD + 0x10 * uart_idx), uart_reg_dlm);
    ESP_LOGI(TAG, "DLM_ADD = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_DLM_ADD + 0x10 * uart_idx), 0xFF));

    uart_reg_lcr &= ~CH9434_UARTx_BIT_DLAB;
    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_LCR_ADD + 0x10 * uart_idx), uart_reg_lcr);
    ESP_LOGI(TAG, "LCR_ADD 2 = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_LCR_ADD + 0x10 * uart_idx), 0xFF));
}

/*
 * Function Name  : CH9434UARTxFIFOSet
 * Description    : 串口FIFO设置
 * Input          : uart_idx：串口号
 *                  fifo_en：FIFO功能使能
 *                  fifo_level：FIFO触发等级
 * Output         : None
 * Output         : None
 * Return         : None
 */
void CH9434UARTxFIFOSet(uint8_t uart_idx, uint8_t fifo_en, uint8_t fifo_level)
{
    uint8_t uart_reg_fcr;

    uart_reg_fcr = 0;
    if (fifo_en)
    {
        uart_reg_fcr |= 0x01;
        uart_reg_fcr |= fifo_level << 6;
    }

    ESP_LOGI(TAG, "uart_reg_fcr = %02X", uart_reg_fcr);

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_FCR_ADD + 0x10 * uart_idx), uart_reg_fcr);
    CH9434_US_DELAY();
    ESP_LOGI(TAG, "FCR_ADD = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_FCR_ADD + 0x10 * uart_idx), 0xFF));
}

/*
 * Function Name  : CH9434UARTxIrqSet
 * Description    : 串口中断设置
 * Input          : uart_idx：串口号
 *                  modem：modem信号中断
 *                  line：线路状态中断
 *                  tx：发送中断
 *                  rx：接收中断
 * Output         : None
 * Return         : None
 */
void CH9434UARTxIrqSet(uint8_t uart_idx, uint8_t modem, uint8_t line, uint8_t tx, uint8_t rx)
{
    uint8_t uart_reg_ier;

    uart_reg_ier = 0;
    if (modem)
        uart_reg_ier |= (1 << 3);
    if (line)
        uart_reg_ier |= (1 << 2);
    if (tx)
        uart_reg_ier |= (1 << 1);
    if (rx)
        uart_reg_ier |= (1 << 0);

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_IER_ADD + 0x10 * uart_idx), uart_reg_ier);
    ESP_LOGI(TAG, "IER_ADD = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_IER_ADD + 0x10 * uart_idx), 0xFF));
}

/*
 * Function Name  : CH9434UARTxFlowSet
 * Description    : 流控设置
 * Input          : uart_idx：串口号
 *                  flow_en：流控使能
 * Output         : None
 * Return         : None
 */
void CH9434UARTxFlowSet(uint8_t uart_idx, uint8_t flow_en)
{
    uint8_t uart_reg_mcr;

    uart_reg_mcr = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), 0xFF);

    ESP_LOGI(TAG, "uart_reg_mcr 1 = %02X", uart_reg_mcr);

    uart_reg_mcr &= ~(1 << 5);
    if (flow_en)
        uart_reg_mcr |= (1 << 5);

    ESP_LOGI(TAG, "uart_reg_mcr 2 = %02X", uart_reg_mcr);

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), uart_reg_mcr);
    ESP_LOGI(TAG, "MCR_ADD = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), 0xFF));
}

/*
 * Function Name  : CH9434UARTxIrqOpen
 * Description    : 开启中断串口请求
 * Input          : uart_idx：串口号
 * Output         : None
 * Return         : None
 */
void CH9434UARTxIrqOpen(uint8_t uart_idx)
{
    uint8_t uart_reg_mcr;

    uart_reg_mcr = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), 0xFF);

    ESP_LOGI(TAG, "uart_reg_mcr 1 = %02X", uart_reg_mcr);

    uart_reg_mcr |= (1 << 3);

    ESP_LOGI(TAG, "uart_reg_mcr 2 = %02X", uart_reg_mcr);

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), uart_reg_mcr);
    ESP_LOGI(TAG, "MCR_ADD = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), 0xFF));
}

/*
 * Function Name  : CH9434UARTxRtsDtrPin
 * Description    : 设置串口RTS、DTR引脚
 * Input          : uart_idx：串口号
 *                  rts_val：RTS引脚电平状态
 *                  dtr_val：DTR引脚电平状态
 * Output         : None
 * Return         : None
 */
void CH9434UARTxRtsDtrPin(uint8_t uart_idx, uint8_t rts_val, uint8_t dtr_val)
{
    uint8_t uart_reg_mcr;

    uart_reg_mcr = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), 0xFF);

    ESP_LOGI(TAG, "uart_reg_mcr 1 = %02X", uart_reg_mcr);

    if (rts_val)
        uart_reg_mcr |= (1 << 1);
    else
        uart_reg_mcr &= ~(1 << 1);
    if (dtr_val)
        uart_reg_mcr |= (1 << 0);
    else
        uart_reg_mcr &= ~(1 << 0);

    ESP_LOGI(TAG, "uart_reg_mcr 2 = %02X", uart_reg_mcr);

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), uart_reg_mcr);
    ESP_LOGI(TAG, "MCR_ADD = %02X", spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_MCR_ADD + 0x10 * uart_idx), 0xFF));
}

/*
 * Function Name  : CH9434UARTxWriteSRC
 * Description    : SRC寄存器写操作
 * Input          : uart_idx：串口号
 *                  src_val：SRC寄存器值
 * Output         : None
 * Return         : None
 */
void CH9434UARTxWriteSRC(uint8_t uart_idx, uint8_t src_val)
{
    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_UARTx_SCR_ADD + 0x10 * uart_idx), src_val);
}

/*
 * Function Name  : CH9434UARTxReadSRC
 * Description    : SRC寄存器读操作
 * Input          : uart_idx：串口号
 * Output         : None
 * Return         : SRC寄存器值
 */
uint8_t CH9434UARTxReadSRC(uint8_t uart_idx)
{
    uint8_t uart_reg_src = 0;

    uart_reg_src = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_SCR_ADD + 0x10 * uart_idx), 0xFF);

    return uart_reg_src;
}

/*
 * Function Name  : CH9434UARTxReadIIR
 * Description    : 串口中断码查询
 * Input          : uart_idx：串口号
 * Output         : None
 * Return         : IIR寄存器值
 */
uint8_t CH9434UARTxReadIIR(uint8_t uart_idx)
{
    uint8_t uart_reg_iir = 0;

    uart_reg_iir = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_IIR_ADD + 0x10 * uart_idx), 0xFF);

    return uart_reg_iir;
}

/*
 * Function Name  : CH9434UARTxReadLSR
 * Description    : 串口LSR寄存器读取
 * Input          : uart_idx：串口号
 * Output         : None
 * Return         : LSR寄存器值
 */
uint8_t CH9434UARTxReadLSR(uint8_t uart_idx)
{
    uint8_t uart_reg_lsr = 0;

    uart_reg_lsr = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_LSR_ADD + 0x10 * uart_idx), 0xFF);

    return uart_reg_lsr;
}

/*
 * Function Name  : CH9434UARTxReadMSR
 * Description    : 串口MSR寄存器读取
 * Input          : uart_idx：串口号
 * Output         : None
 * Return         : MSR寄存器值
 */
uint8_t CH9434UARTxReadMSR(uint8_t uart_idx)
{
    uint8_t uart_reg_msr = 0;

    uart_reg_msr = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_UARTx_MSR_ADD + 0x10 * uart_idx), 0xFF);

    return uart_reg_msr;
}

/*
 * Function Name  : CH9434UARTxGetRxFIFOLen
 * Description    : 获取串口接收数据长度
 * Input          : uart_idx：串口号
 * Output         : None
 * Return         : 串口接收FIFO的大小
 */
uint16_t CH9434UARTxGetRxFIFOLen(uint8_t uart_idx)
{
    uint8_t uart_fifo_ctrl = 0;
    uint8_t uart_fifo_cnt_l;
    uint8_t uart_fifo_cnt_h;
    uint16_t uart_fifo_cnt = 0;

    uart_fifo_ctrl |= uart_idx;

    spi_read_write_byte(CH9434_REG_OP_WRITE | CH9434_FIFO_CTRL_ADD, uart_fifo_ctrl);

    uart_fifo_cnt_l = spi_read_write_byte(CH9434_REG_OP_READ | CH9434_FIFO_CTRL_L_ADD, 0xFF);

    uart_fifo_cnt_h = spi_read_write_byte(CH9434_REG_OP_READ | CH9434_FIFO_CTRL_H_ADD, 0xFF);

    uart_fifo_cnt = uart_fifo_cnt_h;
    uart_fifo_cnt = (uart_fifo_cnt << 8) | uart_fifo_cnt_l;

    return uart_fifo_cnt;
}

/*
 * Function Name  : CH9434UARTxGetRxFIFOData
 * Description    : 读取串口接收数据
 * Input          : uart_idx：串口号
 *                  p_data：数据存储指针
 *                  read_len：读取的数据长度
 * Output         : None
 * Return         : None
 */
uint8_t CH9434UARTxGetRxFIFOData(uint8_t uart_idx, uint8_t *p_data, uint16_t read_len)
{
    uint16_t i;
    uint8_t *p_sv_data;
    uint8_t uart_reg_add;

    uart_reg_add = CH9434_REG_OP_READ | (CH9434_UARTx_RBR_ADD + 0x10 * uart_idx);
    p_sv_data = p_data;
    for (i = 0; i < read_len; i++)
    {
        *p_sv_data++ = spi_read_write_byte(uart_reg_add, 0xFF);
    }

    return 0;
}

/*
 * Function Name  : CH9434UARTxGetTxFIFOLen
 * Description    : 获取串口发送FIFO长度
 * Input          : uart_idx：串口号
 * Output         : None
 * Return         : 当前串口的接收数据长度
 */
uint16_t CH9434UARTxGetTxFIFOLen(uint8_t uart_idx)
{
    uint8_t uart_fifo_ctrl = 0;
    uint8_t uart_fifo_cnt_l;
    uint8_t uart_fifo_cnt_h;
    uint16_t uart_fifo_cnt = 0;

    uart_fifo_ctrl |= CH9434_FIFO_CTRL_TR;
    uart_fifo_ctrl |= uart_idx;

    spi_read_write_byte(CH9434_REG_OP_WRITE | CH9434_FIFO_CTRL_ADD, uart_fifo_ctrl);

    uart_fifo_cnt_l = spi_read_write_byte(CH9434_REG_OP_READ | CH9434_FIFO_CTRL_L_ADD, 0xFF);

    uart_fifo_cnt_h = spi_read_write_byte(CH9434_REG_OP_READ | CH9434_FIFO_CTRL_H_ADD, 0xFF);

    uart_fifo_cnt = uart_fifo_cnt_h;
    uart_fifo_cnt = (uart_fifo_cnt << 8) | uart_fifo_cnt_l;

    return uart_fifo_cnt;
}

/*
 * Function Name  : CH9434UARTxSetTxFIFOData
 * Description    : 串口填入发送数据
 * Input          : uart_idx：串口号
 *                  p_data：发送数据指针
 *                  send_len：发送的数据长度
 * Output         : None
 * Return         : None
 */
uint8_t CH9434UARTxSetTxFIFOData(uint8_t uart_idx, uint8_t *p_data, uint16_t send_len)
{
    uint16_t i;
    uint8_t *p_sv_data;
    uint8_t uart_reg_add;

    uart_reg_add = CH9434_REG_OP_WRITE | (CH9434_UARTx_RBR_ADD + 0x10 * uart_idx);
    p_sv_data = p_data;
    for (i = 0; i < send_len; i++)
    {
        spi_read_write_byte(uart_reg_add, *p_sv_data++);
    }

    return 0;
}

/*
 * Function Name  : CH9434UARTxTnowSet
 * Description    : 串口485切换引脚设置
 * Input          : uart_idx：串口号
 *                  tnow_en：串口tnow使能状态
 *                  polar：极性反向设置
 * Output         : None
 * Return         : None
 */
void CH9434UARTxTnowSet(uint8_t uart_idx, uint8_t tnow_en, uint8_t polar)
{
    uint8_t tnow_ctrl_reg;

    tnow_ctrl_reg = spi_read_write_byte(CH9434_REG_OP_READ | CH9434_TNOW_CTRL_CFG_ADD, 0xFF);

    if (tnow_en)
        tnow_ctrl_reg |= (1 << uart_idx);
    else
        tnow_ctrl_reg &= ~(1 << uart_idx);

    if (polar)
        tnow_ctrl_reg |= (1 << (uart_idx + 4));
    else
        tnow_ctrl_reg &= ~(1 << (uart_idx + 4));

    spi_read_write_byte(CH9434_REG_OP_WRITE | CH9434_TNOW_CTRL_CFG_ADD, tnow_ctrl_reg);
}

/*
 * Function Name  : CH9434LowerPowerModeSet
 * Description    : CH9434芯片低功耗设置
 * Input          : mode：低功耗模式
 * Output         : None
 * Return         : None
 */
void CH9434LowerPowerModeSet(uint8_t mode)
{
    lower_power_reg = mode;

    spi_read_write_byte(CH9434_REG_OP_WRITE | CH9434_SLEEP_MOD_CFG_ADD, lower_power_reg);
}

/*
 * Function Name  : CH9434WakeUp
 * Description    : CH9434唤醒操作，从低功耗模式中唤醒
 * Input          : None
 * Output         : None
 * Return         : None
 */
void CH9434WakeUp(void)
{
    CH9434_SPI_SCS_OP(CH9434_DISABLE);
    CH9434_US_DELAY();
    CH9434_US_DELAY();
    CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*
 * Function Name  : CH9434GPIOFuncSet
 * Description    : GPIO功能设置
 * Input          : gpio_idx：GPIO序号
 *                  en：使能状态
 *                  dir：GPIO方向
 *                  pu：上拉设置
 *                  pd：下拉设置
 * Output         : None
 * Return         : None
 */
void CH9434GPIOFuncSet(uint8_t gpio_idx, uint8_t en, uint8_t dir, uint8_t pu, uint8_t pd)
{
    uint8_t gpio_func_reg; //  GPIO_FUNC
    uint8_t gpio_dir_reg;
    uint8_t gpio_pu_reg;
    uint8_t gpio_pd_reg;

    if (en)
    {
        gpio_func_reg = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8)), 0xFF);

        gpio_func_reg |= (1 << (gpio_idx % 8));

        spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8)), gpio_func_reg);

        gpio_dir_reg = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_GPIO_DIR_MOD_0 + (gpio_idx / 8)), 0xFF);

        if (dir)
            gpio_dir_reg |= (1 << (gpio_idx % 8));
        else
            gpio_dir_reg &= ~(1 << (gpio_idx % 8));

        spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_GPIO_DIR_MOD_0 + (gpio_idx / 8)), gpio_dir_reg);

        gpio_pu_reg = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_GPIO_PU_MOD_0 + (gpio_idx / 8)), 0xFF);

        if (pu)
            gpio_pu_reg |= (1 << (gpio_idx % 8));
        else
            gpio_pu_reg &= ~(1 << (gpio_idx % 8));

        spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_GPIO_PU_MOD_0 + (gpio_idx / 8)), gpio_pu_reg);

        gpio_pd_reg = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_GPIO_PD_MOD_0 + (gpio_idx / 8)), 0xFF);

        if (pd)
            gpio_pd_reg |= (1 << (gpio_idx % 8));
        else
            gpio_pd_reg &= ~(1 << (gpio_idx % 8));

        spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_GPIO_PD_MOD_0 + (gpio_idx / 8)), gpio_pd_reg);
    }
    else
    {
        gpio_func_reg = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8)), 0xFF);

        gpio_func_reg &= ~(1 << (gpio_idx % 8));

        spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_GPIO_FUNC_EN_0 + (gpio_idx / 8)), gpio_func_reg);
    }
}

/*
 * Function Name  : CH9434GPIOPinOut
 * Description    : GPIO输出电平设置
 * Input          : gpio_idx：GPIO序号
 *                  out_val：输出电平设置
 * Output         : None
 * Return         : None
 */
void CH9434GPIOPinOut(uint8_t gpio_idx, uint8_t out_val)
{
    uint8_t pin_val_reg;

    if (out_val)
        ch9434_gpio_x_val |= (1 << gpio_idx);
    else
        ch9434_gpio_x_val &= ~(1 << gpio_idx);

    pin_val_reg = (uint8_t)(ch9434_gpio_x_val >> ((gpio_idx / 8) * 8));

    spi_read_write_byte(CH9434_REG_OP_WRITE | (CH9434_GPIO_PIN_VAL_0 + (gpio_idx / 8)), pin_val_reg);
}

/*
 * Function Name  : CH9434GPIOPinVal
 * Description    : GPIO电平读取
 * Input          : gpio_idx：GPIO序号
 * Output         : None
 * Return         : 电平状态：1：高电平 0：低电平
 */
uint8_t CH9434GPIOPinVal(uint8_t gpio_idx)
{
    uint8_t pin_val_reg;

    pin_val_reg = spi_read_write_byte(CH9434_REG_OP_READ | (CH9434_GPIO_PIN_VAL_0 + (gpio_idx / 8)), 0xFF);

    if (pin_val_reg & (1 << (gpio_idx % 8)))
        return 1;
    else
        return 0;
}

/* -----------------------------------------------------------------------------
 *                                  init function
 * -----------------------------------------------------------------------------
 */

void CH9434UARTxInit(uint8_t uart_idx, uint32_t bps)
{
    CH9434UARTxParaSet(uart_idx, bps,
                       CH9434_UART_8_BITS_PER_CHAR,
                       CH9434_UART_ONE_STOP_BIT,
                       CH9434_UART_NO_PARITY);
    CH9434UARTxFIFOSet(uart_idx,
                       CH9434_ENABLE,
                       CH9434_UART_FIFO_MODE_1280);
    CH9434UARTxFlowSet(uart_idx,
                       CH9434_DISABLE);
    CH9434UARTxIrqSet(uart_idx,
                      CH9434_DISABLE, // modem signal interrupt
                      CH9434_ENABLE,  // line status interrupt
                      CH9434_ENABLE,  // send interrupt
                      CH9434_ENABLE); // receive interrupt
    CH9434UARTxIrqOpen(uart_idx);
    CH9434UARTxRtsDtrPin(uart_idx,
                         CH9434_DISABLE,  // RTS pin level status
                         CH9434_DISABLE); // DTR pin level status
}