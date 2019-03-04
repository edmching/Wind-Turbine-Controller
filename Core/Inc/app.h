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

#define ADC_12B_MAX_RESOLUTION 4095
#define SENSOR_RESOLUTION 100
#define V_SENS_MAX 5
#define I_SENS_MAX 5

void init_sens(uint16_t* voltage,uint16_t* current, uint32_t* power);

typedef struct{
  uint16_t voltage[2];
  uint16_t current[2];
  uint32_t power[2];
  uint8_t duty_cycle;
}mppt_sens_data;


#endif /* INC_APP_H_ */
