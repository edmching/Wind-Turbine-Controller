/*
 * app.h
 *
 *  Created on: Mar 3, 2019
 *      Author: Edm
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "adc.h"
#include "tim.h"
#include "x_nucleo_ihmxx.h"
#include "l6474.h"
#include "x_nucleo_ihm01a1_stm32f4xx.h"

void StepperMotor_App_Init(void);

#endif /* INC_APP_H_ */
