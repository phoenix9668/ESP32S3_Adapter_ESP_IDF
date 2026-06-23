/********************************** (C) COPYRIGHT
 ******************************** File Name          : e34_2g4d20d.h Author :
 *pheonix Version            : V1.0 Date               : 2023/11/22 Description
 *: e34_2g4d20d header file
 *******************************************************************************/

#ifndef __E34_2G4D20D_H__
#define __E34_2G4D20D_H__

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/*
 * e34_2g4d20d pins definition
 */
#define UART1_TXD_PIN (GPIO_NUM_6)
#define UART1_RXD_PIN (GPIO_NUM_7)

#define M0 (GPIO_NUM_4)
#define M1 (GPIO_NUM_5)
#define AUX (GPIO_NUM_15)

/* -----------------------------------------------------------------------------
 *                               macro definition
 * -----------------------------------------------------------------------------
 */
#define E34_2G4D20D_Baudrate 9600

typedef enum { HALF_DUPLEX = 0, FULL_DUPLEX = 1, RESERVE = 2, SET = 3 } modes_t;

/* -----------------------------------------------------------------------------
 *                      define E34_2G4D20D interface functions
 * -----------------------------------------------------------------------------
 */

esp_err_t e34_2g4d20d_uart1_init(void);
esp_err_t e34_2g4d20d_gpio_init(void);
esp_err_t e34_2g4d20d_wait_aux(uint32_t timeout_ms);
int e34_2g4d20d_sendData(const char *logName, const uint8_t *data, size_t len);

/* -----------------------------------------------------------------------------
 *                                  api function
 * -----------------------------------------------------------------------------
 */

void e34_2g4d20d_model_sel(modes_t mode);
esp_err_t e34_2g4d20d_parameter_set(uint8_t dev_head, uint8_t dev_addh,
                                    uint8_t dev_addl, uint8_t dev_sped,
                                    uint8_t dev_chan, uint8_t dev_option);
esp_err_t e34_2g4d20d_reset(void);

#endif /*__E34_2G4D20D_H__ */
