/*
 * app.c
 *
 *  Created on: Mar 3, 2019
 *      Author: Edm
 */
#include "app.h"

void StepperMotor_App_Init(void){
//----- Init of the Motor control library
/* Set the L6474 library to use 1 device */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the L6474 registers and parameters are set with the predefined values from file   */
  /* l6474_target_config.h, otherwise the registers are set using the   */
  /* L6474_Init_t pointer structure                */
  /* The first call to BSP_MotorControl_Init initializes the first device     */
  /* whose Id is 0.                                                           */
  /* The nth call to BSP_MotorControl_Init initializes the nth device         */
  /* whose Id is n-1.                                                         */
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);

  /* Attach the function Error_Handler (defined below) to the error Handler*/
 // BSP_MotorControl_AttachErrorHandler(Error_Handler);

//----- Autocheck sequence

  /* Set Systick Interrupt to the highest priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0);

  /* Set full step mode */
  BSP_MotorControl_SelectStepMode(0, STEP_MODE_1_16);

  ///To disable power outputs when stepper is not running
  ///causes shaft to move around
  //BSP_MotorControl_SetStopMode(0, HIZ_MODE);

  /* Set min speed to 8 pps */
  //BSP_MotorControl_SetMinSpeed(0, 8);

  /* Set max speed to 1200step/s */
  BSP_MotorControl_SetMaxSpeed(0, 2400);
  BSP_MotorControl_SetMinSpeed(0, 2400);
  //BSP_MotorControl_SetAcceleration(0,480);
 // BSP_MotorControl_SetDeceleration(0,160);
  BSP_MotorControl_SetDirection(0, FORWARD);
}

