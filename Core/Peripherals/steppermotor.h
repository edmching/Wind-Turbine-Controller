/*
 * steppermotor.h
 *
 *  Created on: Jan 15, 2019
 *      Author: edmch
 */

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

#include "stm32f4xx_hal.h"

#define STEPMOTOR_PORT_IN			GPIOB

#define STEPMOTOR_PIN_IN1			GPIO_PORT_D_PIN_3
#define STEPMOTOR_PIN_IN2			GPIO_PORT_D_PIN_4
#define STEPMOTOR_PIN_IN3			GPIO_PORT_D_PIN_5
#define STEPMOTOR_PIN_IN4			GPIO_PORT_D_PIN_6

#define GPIO_PORT_D_PIN_6			GPIO_PIN_10
#define GPIO_PORT_D_PIN_5			GPIO_PIN_4
#define GPIO_PORT_D_PIN_4			GPIO_PIN_5
#define GPIO_PORT_D_PIN_3			GPIO_PIN_3

#define NUM_STEPS_360_DEG			512
#define FULL_ROTATATION_IN_DEG		360.0

//uint8_t wave_CW[4] = {0b1000, 0b0100, 0b0010, 0b0001};
//uint8_t wave_CCW[4] = {0b0001, 0b0100, 0b0010, 0b0001};

typedef enum {CW,CCW} direction;

void StepmotorGPIOInit(void);
void StepmotorWaveDriveCW (void);
void StepmotorWaveDriveCCW (void);
void StepmotorFullDriveCW (void);
void StepmotorHalfDriveCW (void);
void StepmotorMoveAngle(int angle, direction _direction);
void StepmotorMoveAngleHalfStep(int angle, direction _direction);

#endif /* STEPPERMOTOR_H_ */
