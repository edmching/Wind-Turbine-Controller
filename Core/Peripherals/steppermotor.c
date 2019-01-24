/*
 * steppermotor.c
 *
 *  Created on: Jan 15, 2019
 *      Author: edmch
 */

#include "steppermotor.h"

static void Stepmotor_IN1_ON(void);
static void Stepmotor_IN2_ON(void);
static void Stepmotor_IN3_ON(void);
static void Stepmotor_IN4_ON(void);
static void Stepmotor_IN_reset(void);

/**
  * @brief Stepmotor GPIO Initialization Function
  * @param None
  * @retval None
  */
// TODO: separate LED and PB from this file
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

void StepmotorMoveAngle(int angle, direction _direction)
{
	volatile int numStepsAngle = angle/(float) (FULL_ROTATATION_IN_DEG/NUM_STEPS_360_DEG);

	assert_param(numStepsAngle >= 0 && numStepsAngle <= 360);

	for (int i = 0; i< numStepsAngle; ++i){
		if(_direction == CW){
			Stepmotor_IN1_ON();
			HAL_Delay(2);
			Stepmotor_IN2_ON();
			HAL_Delay(2);
			Stepmotor_IN3_ON();
			HAL_Delay(2);
			Stepmotor_IN4_ON();

		}
		else if(_direction == CCW){
			Stepmotor_IN4_ON();
			HAL_Delay(2);
			Stepmotor_IN3_ON();
			HAL_Delay(2);
			Stepmotor_IN2_ON();
			HAL_Delay(2);
			Stepmotor_IN1_ON();
		}
	  }
}


void StepmotorWaveDriveCW (void)
{
  for(int i = 0; i< NUM_STEPS_360_DEG; ++i)
  {
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_SET); //INT1
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT2
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2,  GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT3
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT4
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_SET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET);  // turns all off
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);
  }

}

void StepmotorWaveDriveCCW (void)
{
  for(int i = 0; i<NUM_STEPS_360_DEG; ++i)
  {

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT4
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_SET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT3
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);
	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT2
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2,  GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_SET); //INT1
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);
  }

}

void StepmotorFullDriveCW (void)
{
	for(int i = 0; i<NUM_STEPS_360_DEG; ++i)
	{

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_SET); //INT1 & INT2
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT2 & INT3
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT3 & INT4
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_SET);
	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_SET); //INT4 & INT1
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_SET);

	  HAL_Delay(2);

	}

}

void StepmotorHalfDriveCW (void)
{
	for(int i = 0; i<NUM_STEPS_360_DEG; ++i)
	{
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_SET); //INT1
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_SET); //INT1 & INT2
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT2
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2,  GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT2 & INT3
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT3
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2,  GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3,  GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT3 & INT4
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_SET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT4
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2,  GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4,  GPIO_PIN_SET);

	  HAL_Delay(2);

	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_SET); //INT4 & INT1
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_SET);

	  HAL_Delay(2);

	}

}


static void Stepmotor_IN1_ON(void)
{
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_SET); //INT1
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);
}

static void Stepmotor_IN2_ON(void)
{
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT2
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);
}

static void Stepmotor_IN3_ON(void)
{
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT3
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);
}

static void Stepmotor_IN4_ON(void)
{
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET); //INT4
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_SET);
}

static void Stepmotor_IN_reset(void)
{
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN1, GPIO_PIN_RESET);  // turns all off
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN2, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN3, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(STEPMOTOR_PORT_IN, STEPMOTOR_PIN_IN4, GPIO_PIN_RESET);
}





