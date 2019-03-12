/*
 * steppermotor.c
 *
 *  Created on: Jan 15, 2019
 *      Author: edmch
 */

#include "steppermotor.h"

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
void StepmotorGPIOInit(Stepmotor_Status* motor_status)
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

	  motor_status->current_position = 0;
	  motor_status->reference_position = 0;
	  motor_status->_direction = CW;
	  motor_status->speed = 1;
	  motor_status->state = MOTOR_IS_STOPPED;
	  motor_status->steps_to_move = 0;
}

stepmotor_state Stepmotor_run_halfstep(Stepmotor_Status* motor_status)
{
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;
	
	uint32_t steps_required = motor_status->steps_to_move;
	direction _direction = motor_status->_direction;
	int RPM_delay_time = motor_status->speed;

	if(steps_required > 0){
		if(_direction == CW){
			/* moves one step CW */
			for(int i = 0; i<8; ++i){
				GPIOx->BSRR = half_step_sequence[i];
				HAL_Delay(RPM_delay_time);
			}
			motor_status->current_position++;
		}
		else if(_direction == CCW){
			/* moves one step CCW */
			for(int i = 7; i>=0; --i){
				GPIOx->BSRR = half_step_sequence[i];
				HAL_Delay(RPM_delay_time);
			}
			motor_status->current_position--;
		}
		GPIOx->BSRR = STEP_IN_RESET_ALL;

		/*updates the motor status*/
		motor_status->steps_to_move--;

		motor_status->state = MOTOR_IS_RUNNING;
	}
	else{
		motor_status->state = MOTOR_IS_STOPPED;
	}

	return motor_status->state;
}

void Stepmotor_set_goal_position(Stepmotor_Status* motor_status, int32_t _steps_to_move)
{
	if(_steps_to_move < 0){
		motor_status->steps_to_move = -(_steps_to_move);
		motor_status->_direction = CCW;
	}
	else{
		motor_status->steps_to_move = _steps_to_move;
		motor_status->_direction = CW; //CW is positive
	}
}

void Stepmotor_set_direction(Stepmotor_Status* motor_status, direction __direction)
{
	motor_status->_direction = __direction;
}

////////////////UNUSED FUNCTIONS////////////////////////////
//untested and not working as intended
void Stepmotor_set_reference_position(Stepmotor_Status* motor_status)
{
	motor_status->reference_position = motor_status->current_position;
	motor_status->current_position   = 0;
}

//unused
void StepmotorMoveAngleHalfStep(int angle, direction _direction)
{

	int numStepsAngle = angle/(float) (FULL_ROTATATION_IN_DEG/NUM_STEPS_360_DEG);
	int RPM_delay_time = 1;
	GPIO_TypeDef* GPIOx = STEPMOTOR_PORT_IN;
	assert_param(numStepsAngle >= 0 && numStepsAngle <= 360);

	for (int i = 0; i< numStepsAngle; ++i){
		if(_direction == CW){
			for(int j = 0; j<8; ++j){
				GPIOx->BSRR = half_step_sequence[j];
				HAL_Delay(RPM_delay_time);
			}
		}
		else if(_direction == CCW){
			for(int j = 7; j>=0; --j){
				GPIOx->BSRR = half_step_sequence[j];
				HAL_Delay(RPM_delay_time);
			}
		}
	  }
	GPIOx->BSRR = STEP_IN_RESET_ALL;
}


