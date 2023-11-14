/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : phoenix
 * Version            : V1.0
 * Date               : 2023/11/1
 * Description        : CH9434 driver on esp32s3
 *******************************************************************************/

/******************************************************************************/
/* header file */
#include "debug.h"
#include "string.h"
#include "CH9434.h"
#include "esp_rom_sys.h"

uint8_t uart_idx;
uint8_t uart_iir;
uint8_t uart_lsr;
uint8_t uart_msr;

uint16_t rec_buf_cnt = 0;
uint8_t uart_rec_buf[512];

uint32_t uart_rec_total_cnt[4] = {0, 0, 0, 0};

#define dg_log printf

/* uart init */
void UARTPrintfInit(void)
{
	USART_Printf_Init(460800);
}

/* define CH9434 interface functions */
void CH9434_US_DELAY(void)
{
	esp_rom_delay_us(1);
}

/* SPI SCS pin control, 0:low level  1:high level */
void CH9434_SPI_SCS_OP(uint8_t dat)
{
	if (dat)
		GPIO_SetBits(GPIOA, GPIO_Pin_4); // SCS high
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_4); // SCS low
}

/* SPI exchange one byte interface */
uint8_t CH9434_SPI_WRITE_BYTE(uint8_t dat)
{
	SPI_I2S_SendData(SPI1, (uint16_t)dat);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
		;
	return ((uint8_t)SPI_I2S_ReceiveData(SPI1));
}

/* SPI host init */
void SPIHostInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // SPI1_NSS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // SPI1_SCK
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // SPI1_MISO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // SPI1_MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);
}

/* INT# pin init */
void InitIntGPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // PA0--INT#
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
 * Function Name  : main
 * Description    : main function
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
int main(void)
{
	uint32_t i;
	uint32_t test_bps;
	uint32_t tim_cnt = 0;
	uint8_t pin_val = 0;

	Delay_Init();

	/* delay a moment */
	Delay_Ms(50);

	/* init uart print */
	UARTPrintfInit();

	// clock init
	dg_log("CH9434 TEST START %s %s \r\n", __TIME__, __DATE__);

	SPIHostInit();

	/* SPI transmit test */
	/* init CH9434 */
	CH9434InitClkMode(CH9434_ENABLE, // extern Crystal oscillator
					  CH9434_ENABLE, // enable frequency doubling
					  13);			 // Frequency division coefficient
	Delay_Ms(50);

	/* init uart */
	test_bps = 115200;

	// init uart1
	CH9434UARTxParaSet(CH9434_UART_IDX_0,
					   test_bps,
					   CH9434_UART_8_BITS_PER_CHAR,
					   CH9434_UART_ONE_STOP_BIT,
					   CH9434_UART_NO_PARITY);
	CH9434UARTxFIFOSet(CH9434_UART_IDX_0,
					   CH9434_ENABLE,
					   CH9434_UART_FIFO_MODE_1280);
	CH9434UARTxFlowSet(CH9434_UART_IDX_0,
					   CH9434_ENABLE);
	CH9434UARTxIrqSet(CH9434_UART_IDX_0,
					  CH9434_DISABLE, // modem signal interrupt
					  CH9434_ENABLE,  // line status interrupt
					  CH9434_ENABLE,  // send interrupt
					  CH9434_ENABLE); // receive interrupt
	CH9434UARTxIrqOpen(CH9434_UART_IDX_0);
	CH9434UARTxRtsDtrPin(CH9434_UART_IDX_0,
						 CH9434_ENABLE,	 // RTS pin level status
						 CH9434_ENABLE); // DTR pin level status

	// init uart1
	CH9434UARTxParaSet(CH9434_UART_IDX_1,
					   test_bps,
					   CH9434_UART_8_BITS_PER_CHAR,
					   CH9434_UART_ONE_STOP_BIT,
					   CH9434_UART_NO_PARITY);
	CH9434UARTxFIFOSet(CH9434_UART_IDX_1,
					   CH9434_ENABLE,
					   CH9434_UART_FIFO_MODE_1280);
	CH9434UARTxFlowSet(CH9434_UART_IDX_1,
					   CH9434_ENABLE);
	CH9434UARTxIrqSet(CH9434_UART_IDX_1,
					  CH9434_DISABLE, // modem signal interrupt
					  CH9434_ENABLE,  // line status interrupt
					  CH9434_ENABLE,  // send interrupt
					  CH9434_ENABLE); // receive interrupt
	CH9434UARTxIrqOpen(CH9434_UART_IDX_1);
	CH9434UARTxRtsDtrPin(CH9434_UART_IDX_1,
						 CH9434_ENABLE,	 // RTS pin level status
						 CH9434_ENABLE); // DTR pin level status

	// init uart2
	CH9434UARTxParaSet(CH9434_UART_IDX_2,
					   test_bps,
					   CH9434_UART_8_BITS_PER_CHAR,
					   CH9434_UART_ONE_STOP_BIT,
					   CH9434_UART_NO_PARITY);
	CH9434UARTxFIFOSet(CH9434_UART_IDX_2,
					   CH9434_ENABLE,
					   CH9434_UART_FIFO_MODE_1280);
	CH9434UARTxFlowSet(CH9434_UART_IDX_2,
					   CH9434_ENABLE);
	CH9434UARTxIrqSet(CH9434_UART_IDX_2,
					  CH9434_DISABLE, // modem signal interrupt
					  CH9434_ENABLE,  // line status interrupt
					  CH9434_ENABLE,  // send interrupt
					  CH9434_ENABLE); // receive interrupt
	CH9434UARTxIrqOpen(CH9434_UART_IDX_2);
	CH9434UARTxRtsDtrPin(CH9434_UART_IDX_2,
						 CH9434_ENABLE,	 // RTS pin level status
						 CH9434_ENABLE); // DTR pin level status

	// ��ʼ������3
	CH9434UARTxParaSet(CH9434_UART_IDX_3,
					   test_bps,
					   CH9434_UART_8_BITS_PER_CHAR,
					   CH9434_UART_ONE_STOP_BIT,
					   CH9434_UART_NO_PARITY);
	CH9434UARTxFIFOSet(CH9434_UART_IDX_3,
					   CH9434_ENABLE,
					   CH9434_UART_FIFO_MODE_1280);
	CH9434UARTxFlowSet(CH9434_UART_IDX_3,
					   CH9434_ENABLE);
	CH9434UARTxIrqSet(CH9434_UART_IDX_3,
					  CH9434_DISABLE, // modem signal interrupt
					  CH9434_ENABLE,  // line status interrupt
					  CH9434_ENABLE,  // send interrupt
					  CH9434_ENABLE); // receive interrupt
	CH9434UARTxIrqOpen(CH9434_UART_IDX_3);
	CH9434UARTxRtsDtrPin(CH9434_UART_IDX_3,
						 CH9434_ENABLE,	 // RTS pin level status
						 CH9434_ENABLE); // DTR pin level status

	while (1)
	{
		/* uart Threading */
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET) // INT level is low
		{
			for (uart_idx = 0; uart_idx < 4; uart_idx++)
			{
				uart_iir = CH9434UARTxReadIIR(uart_idx);
				dg_log("idx:%d uart_iir:%02x\r\n", uart_idx, uart_iir);
				switch (uart_iir & 0x0f)
				{
				case 0x01: // no interrupt
					break;
				case 0x06: // receive line status
				{
					uart_lsr = CH9434UARTxReadLSR(uart_idx);
					dg_log("uart_lsr:%02x\r\n", uart_lsr);
					rec_buf_cnt = CH9434UARTxGetRxFIFOLen(uart_idx);
					if (rec_buf_cnt)
					{
						CH9434UARTxGetRxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
						uart_rec_total_cnt[uart_idx] += rec_buf_cnt;
						dg_log("idx:%d rec:%d total:%d\r\n", uart_idx, rec_buf_cnt, (int)uart_rec_total_cnt[uart_idx]);
						CH9434UARTxSetTxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
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
						dg_log("idx:%d rec:%d total:%d\r\n", uart_idx, rec_buf_cnt, (int)uart_rec_total_cnt[uart_idx]);
						CH9434UARTxSetTxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
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
						dg_log("idx:%d rec:%d total:%d\r\n", uart_idx, rec_buf_cnt, (int)uart_rec_total_cnt[uart_idx]);
						CH9434UARTxSetTxFIFOData(uart_idx, uart_rec_buf, rec_buf_cnt);
					}
					break;
				}
				case 0x02: // THR register empty
					break;
				case 0x00: // modem signal change
				{
					uart_msr = CH9434UARTxReadMSR(uart_idx);
					dg_log("uart_msr:%02x\r\n", uart_msr);
					break;
				}
				}
			}
		}
	}
}

/*************************************************************************************************
**************************************************************************************************/
