/*
 * steppermotor.c
 *
 *  Created on: Jan 15, 2019
 *      Author: edmch
 */

#include "steppermotor.h"

static void Stepmotor_IN1_ON(void);
static void Stepmotor_IN1_N_IN2_ON(void);
static void Stepmotor_IN2_ON(void);
static void Stepmotor_IN2_N_IN3_ON(void);
static void Stepmotor_IN3_ON(void);
static void Stepmotor_IN3_N_IN4_ON(void);
static void Stepmotor_IN4_ON(void);
static void Stepmotor_IN4_N_IN1_ON(void);
static void Stepmotor_IN_Reset(void);

static uint32_t half_step_sequence[8] = {STEP_IN1_ON,
										STEP_IN1_N_IN2_ON,
										STEP_IN2_ON,
										STEP_IN2_N_IN3_ON,
										STEP_IN3_ON,
										STEP_IN3_N_IN4_ON,
										STEP_IN4_ON,
										STEP_IN4_N_IN1_ON};

static uint32_t full_step_sequence[4] = {STEP_IN1_N_IN2_ON,
										STEP_IN2_N_IN3_ON,
										STEP_IN3_N_IN4_ON,
										STEP_IN4_N_IN1_ON};

static uint32_t wave_step_sequence[4] = {STEP_IN1_ON,
										STEP_IN2_ON,
										STEP_IN3_ON,
										STEP_IN4_ON};
/**
  * @brief Stepmotor GPIO Initialization Function
  * @param None
  * @retval None
  */
void StepmotorGPIOInit(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

	  /*Configure GPIO pins : PB10 PB4 PB5 */
	  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Stepmotor_Nonblocking_Move(int angle, direction _direction, uint32_t* step_counter)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;
	int numStepsAngle = angle/(float) (FULL_ROTATATION_IN_DEG/NUM_STEPS_360_DEG);
	int RPM_delay_time = 1;
	assert_param(numStepsAngle >= 0 && numStepsAngle <= 360);

	if (*step_counter == numStepsAngle){
		*step_counter = 0;
		HAL_Delay(1000);
	}
	else if(*step_counter < numStepsAngle){
		if(_direction == CW){
			for(int i = 0; i<8; ++i){
				GPIOx->BSRR = half_step_sequence[i];
				HAL_Delay(RPM_delay_time);
			}

		}
		else if(_direction == CCW){
			for(int i = 7; i>=0; --i){
				GPIOx->BSRR = half_step_sequence[i];
				HAL_Delay(RPM_delay_time);
			}
		}
		Stepmotor_IN_Reset();
		*step_counter = *step_counter + 1;
	}

}

void StepmotorMoveAngleHalfStep(int angle, direction _direction)
{
	int numStepsAngle = angle/(float) (FULL_ROTATATION_IN_DEG/NUM_STEPS_360_DEG);
	int RPM_delay_time = 1;
	assert_param(numStepsAngle >= 0 && numStepsAngle <= 360);

	for (int i = 0; i< numStepsAngle; ++i){
		if(_direction == CW){
			Stepmotor_IN1_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN1_N_IN2_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN2_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN2_N_IN3_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN3_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN3_N_IN4_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN4_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN4_N_IN1_ON();
		}
		else if(_direction == CCW){
			Stepmotor_IN4_N_IN1_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN4_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN3_N_IN4_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN3_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN2_N_IN3_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN2_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN1_N_IN2_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN1_ON();
		}
	  }
	Stepmotor_IN_Reset();
}

void StepmotorMoveAngleFullDrive(int angle, direction _direction)
{
	int numStepsAngle = angle/(float) (FULL_ROTATATION_IN_DEG/NUM_STEPS_360_DEG);
	int RPM_delay_time = 2;
	assert_param(numStepsAngle >= 0 && numStepsAngle <= 360);

	for (int i = 0; i< numStepsAngle; ++i){
		if(_direction == CW){
			Stepmotor_IN1_N_IN2_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN2_N_IN3_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN3_N_IN4_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN4_N_IN1_ON();
		}
		else if(_direction == CCW){
			Stepmotor_IN4_N_IN1_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN3_N_IN4_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN2_N_IN3_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN1_N_IN2_ON();
		}
	  }
	Stepmotor_IN_Reset();
}


void StepmotorMoveAngle(int angle, direction _direction)
{
	int numStepsAngle = angle/(float) (FULL_ROTATATION_IN_DEG/NUM_STEPS_360_DEG);
	int RPM_delay_time = 2;
	assert_param(numStepsAngle >= 0 && numStepsAngle <= 360);

	for (int i = 0; i< numStepsAngle; ++i){
		if(_direction == CW){
			Stepmotor_IN1_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN2_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN3_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN4_ON();
		}
		else if(_direction == CCW){
			Stepmotor_IN4_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN3_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN2_ON();
			HAL_Delay(RPM_delay_time);
			Stepmotor_IN1_ON();
		}
	  }
	Stepmotor_IN_Reset();
}






static void Stepmotor_IN1_N_IN2_ON(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	/* can use '=' since it is atomic instruction */
	GPIOx->BSRR = STEPMOTOR_PIN_IN1 | STEPMOTOR_PIN_IN2; 
	GPIOx->BSRR = ((uint32_t) ( STEPMOTOR_PIN_IN3 | STEPMOTOR_PIN_IN4)) << 16U;

}

static void Stepmotor_IN2_N_IN3_ON(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	/* can use '=' since it is atomic instruction */
	GPIOx->BSRR = STEPMOTOR_PIN_IN2 | STEPMOTOR_PIN_IN3; 
	GPIOx->BSRR = ((uint32_t) ( STEPMOTOR_PIN_IN1 |STEPMOTOR_PIN_IN4)) << 16U;

}

static void Stepmotor_IN3_N_IN4_ON(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	/* can use '=' since it is atomic instruction */
	GPIOx->BSRR = STEPMOTOR_PIN_IN3 | STEPMOTOR_PIN_IN4; 
	GPIOx->BSRR = ((uint32_t) ( STEPMOTOR_PIN_IN1 | STEPMOTOR_PIN_IN2)) << 16U;

}

static void Stepmotor_IN4_N_IN1_ON(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	/* can use '=' since it is atomic instruction */
	GPIOx->BSRR = STEPMOTOR_PIN_IN4 | STEPMOTOR_PIN_IN1; 
	GPIOx->BSRR = ((uint32_t) ( STEPMOTOR_PIN_IN2 | STEPMOTOR_PIN_IN3)) << 16U;

}

static void Stepmotor_IN1_ON(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	/* can use '=' since it is atomic instruction */
	GPIOx->BSRR = STEPMOTOR_PIN_IN1; 
	GPIOx->BSRR = ((uint32_t) (STEPMOTOR_PIN_IN2  | STEPMOTOR_PIN_IN3 |
					STEPMOTOR_PIN_IN4)) << 16U;

}

static void Stepmotor_IN2_ON(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	GPIOx->BSRR = STEPMOTOR_PIN_IN2;
	GPIOx->BSRR = ((uint32_t) (STEPMOTOR_PIN_IN1  | STEPMOTOR_PIN_IN3 |
					STEPMOTOR_PIN_IN4)) << 16U;

}


static void Stepmotor_IN3_ON(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	GPIOx->BSRR = STEPMOTOR_PIN_IN3;
	GPIOx->BSRR = ((uint32_t) (STEPMOTOR_PIN_IN1  | STEPMOTOR_PIN_IN2 |
					STEPMOTOR_PIN_IN4)) << 16U;

}


static void Stepmotor_IN4_ON(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	GPIOx->BSRR = STEPMOTOR_PIN_IN4;
	GPIOx->BSRR = ((uint32_t) (STEPMOTOR_PIN_IN1  | STEPMOTOR_PIN_IN2 |
					STEPMOTOR_PIN_IN3)) << 16U;
}

static void Stepmotor_IN_Reset(void)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;

	GPIOx->BSRR = ((uint32_t) (STEPMOTOR_PIN_IN1 | STEPMOTOR_PIN_IN2|
					STEPMOTOR_PIN_IN3 | STEPMOTOR_PIN_IN4)) << 16U;
}


