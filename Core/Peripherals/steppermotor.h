/*
 * steppermotor.h
 *
 *  Created on: Jan 15, 2019
 *      Author: edmch
 */

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f401xe.h"

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

#define STEP_IN1_ON			STEPMOTOR_PIN_IN1 | \
								 ((uint32_t)(STEPMOTOR_PIN_IN2|STEPMOTOR_PIN_IN3|STEPMOTOR_PIN_IN4) << 16U)
#define STEP_IN2_ON			STEPMOTOR_PIN_IN2 | \
								 ((uint32_t) (STEPMOTOR_PIN_IN1|STEPMOTOR_PIN_IN3|STEPMOTOR_PIN_IN4) << 16U)
#define STEP_IN3_ON			STEPMOTOR_PIN_IN3 | \
								 ((uint32_t) (STEPMOTOR_PIN_IN1|STEPMOTOR_PIN_IN2|STEPMOTOR_PIN_IN4) << 16U)
#define STEP_IN4_ON			STEPMOTOR_PIN_IN4 |	\
								 ((uint32_t) (STEPMOTOR_PIN_IN1|STEPMOTOR_PIN_IN2|STEPMOTOR_PIN_IN3) << 16U)
#define STEP_IN1_N_IN2_ON  STEPMOTOR_PIN_IN1 | STEPMOTOR_PIN_IN2 | \
								 ((uint32_t) (STEPMOTOR_PIN_IN3|STEPMOTOR_PIN_IN4) << 16U)
#define STEP_IN2_N_IN3_ON  STEPMOTOR_PIN_IN2 | STEPMOTOR_PIN_IN3 | \
								 ((uint32_t) (STEPMOTOR_PIN_IN1|STEPMOTOR_PIN_IN4) << 16U)
#define STEP_IN3_N_IN4_ON  STEPMOTOR_PIN_IN3 | STEPMOTOR_PIN_IN4 | \
								 ((uint32_t) (STEPMOTOR_PIN_IN1|STEPMOTOR_PIN_IN2) << 16U)
#define STEP_IN4_N_IN1_ON  STEPMOTOR_PIN_IN4 | STEPMOTOR_PIN_IN1 | \
								 ((uint32_t) (STEPMOTOR_PIN_IN2|STEPMOTOR_PIN_IN3) << 16U)

typedef enum {CW,CCW}direction;

void StepmotorGPIOInit(void);
void StepmotorWaveDriveCW (void);
void StepmotorWaveDriveCCW (void);
void StepmotorFullDriveCW (void);
void StepmotorHalfDriveCW (void);
void StepmotorMoveAngle(int angle, direction _direction);
void StepmotorMoveAngleHalfStep(int angle, direction _direction);
void StepmotorMoveAngleFullDrive(int angle, direction _direction);
void Stepmotor_Nonblocking_Move(int angle, direction _direction, uint32_t* step_counter);

#endif /* STEPPERMOTOR_H_ */
