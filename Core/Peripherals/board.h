/*
 * board.h
 *
 *  Created on: Jan 17, 2019
 *      Author: edmch
 */

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

#include "stm32f4xx_hal.h"

#define LD2_PIN GPIO_PIN_5
#define LD2_GPIO_PORT GPIOA
#define B1_PIN GPIO_PIN_13
#define B1_GPIO_PORT GPIOC

void BoardInit(void);

#endif /* INC_BOARD_H_ */
