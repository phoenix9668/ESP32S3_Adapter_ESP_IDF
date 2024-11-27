/********************************** (C) COPYRIGHT *******************************
 * File Name          : e34_2g4d20d.h
 * Author             : pheonix
 * Version            : V1.0
 * Date               : 2023/11/22
 * Description        : e34_2g4d20d header file
 *******************************************************************************/

#ifndef __E34_2G4D20D_H__
#define __E34_2G4D20D_H__
#include "driver/uart.h"
#include "driver/gpio.h"

/*
 * e34_2g4d20d pins definition
 */
#define UART1_TXD_PIN (GPIO_NUM_6)
#define UART1_RXD_PIN (GPIO_NUM_7)

#define M0 (GPIO_NUM_5)
#define M1 (GPIO_NUM_4)
#define AUX (GPIO_NUM_15)

/* -----------------------------------------------------------------------------
 *                               macro definition
 * -----------------------------------------------------------------------------
 */
#define E34_2G4D20D_Baudrate 9600

typedef enum
{
    HALF_DUPLEX = 0,
    FULL_DUPLEX = 1,
    RESERVE = 2,
    SET = 3
} modes_t;

/* -----------------------------------------------------------------------------
 *                      define E34_2G4D20D interface functions
 * -----------------------------------------------------------------------------
 */

void e34_2g4d20d_uart1_init(void);
void e34_2g4d20d_gpio_init(void);
int e34_2g4d20d_sendData(const char *logName, const char *data, int len);

/* -----------------------------------------------------------------------------
 *                                  api function
 * -----------------------------------------------------------------------------
 */

void e34_2g4d20d_model_sel(modes_t mode);
void e34_2g4d20d_parameter_set(char dev_head, char dev_addh, char dev_addl, char dev_sped, char dev_chan, char dev_option);
void e34_2g4d20d_reset();

#endif /*__E34_2G4D20D_H__ */