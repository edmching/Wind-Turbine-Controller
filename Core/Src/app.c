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
  //BSP_MotorControl_AttachErrorHandler(Error_Handler);

//----- Autocheck sequence

  /* Set Systick Interrupt to the highest priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0);

  /* Set 1/16 microstepping */
  BSP_MotorControl_SelectStepMode(0, STEP_MODE_1_16);

  /* Set max speed to 1200step/s */
  BSP_MotorControl_SetMinSpeed(0, 4800);
  BSP_MotorControl_SetMaxSpeed(0, 4800);

  BSP_MotorControl_SetAcceleration(0,480);
  BSP_MotorControl_SetDeceleration(0,480);

  BSP_MotorControl_SetDirection(0, FORWARD);
}

void ReadSensor_Task(SensorVar_t* sensorVars)
{
  static bool on_power_cycle = true;

  /// save adc values locally
  __disable_irq();// TODO: change to disable only DMA interrupts
  uint32_t ADC_V_rect    =  g_adc_val[0];
  uint32_t ADC_current   =  g_adc_val[1];
  uint32_t ADC_pot_angle =  g_adc_val[2];
  uint32_t ADC_V_load    =  g_adc_val[3];
  __enable_irq();
  static float Vcc_div_by_2 = 2.445;

  //reads rectifier voltage, boosted voltage and current
  sensorVars->voltage_rectifier[1] = ADC_V_rect*ADC_VAL_TO_VOLTS*V_RECTIFIER_TRANSFER_RATIO*SENSOR_RESOLUTION;
  sensorVars->voltage_load         = ADC_V_load*ADC_VAL_TO_VOLTS*V_LOAD_TRANSFER_RATIO*SENSOR_RESOLUTION;
  sensorVars->current[1]           = ((ADC_current*ADC_VAL_TO_VOLTS - Vcc_div_by_2)/I_SENS_V_TO_I_RATIO)*SENSOR_RESOLUTION;
  sensorVars->power_rectifier[1]   =  sensorVars->voltage_rectifier[1]*sensorVars->current[1];
  sensorVars->power_load           =  sensorVars->voltage_load*sensorVars->current[1];

  //read wind vane angle
  sensorVars->pot_angle[1] = map_fvalues(
                                ADC_pot_angle,
                                POT_ADC_ZERO_ANGLE_VAL, 
                                POT_ADC_MAX_ANGLE_VAL, 
                                POT_MIN_ANGLE, 
                                POT_MAX_ANGLE
                              );
/*
  if(ADC_current*ADC_VAL_TO_VOLTS*SENSOR_RESOLUTION < Vcc_div_by_2*SENSOR_RESOLUTION)
  {
	  Vcc_div_by_2 = (float) ADC_current*ADC_VAL_TO_VOLTS; // convert from I to V
  }
*/
  ///On power cycle, we save the first sample, so that have a val to compare it
  if(on_power_cycle == true){
	on_power_cycle                     = false;
    sensorVars->pot_angle[0]           =  sensorVars->pot_angle[1];
    sensorVars->voltage_rectifier[0]   =  sensorVars->voltage_rectifier[1];
    sensorVars->current[0]             =  sensorVars->current[1];
    sensorVars->power_rectifier[0]     =  sensorVars->power_rectifier[1];
  }
}

void StepMotor_Task(SensorVar_t* sensorVars)
{
  motorState_t motor_state;
  int32_t target_position;
  static bool on_power_cycle = true;

  ///On power cycle, initialize the current position as the home position
  if(on_power_cycle == true)
  {
	on_power_cycle = false;

    int32_t home_position;
    //converts angle values to step values
    home_position = map_fvalues(
                        sensorVars->pot_angle[1],
                        POT_MIN_ANGLE,
                        POT_MAX_ANGLE,
                        POT_NUM_OF_STEPS_MIN,
                        POT_NUM_OF_STEPS_MAX
					);
    BSP_MotorControl_SetHome(0, -home_position);// negative since we subtract from ABS_POS

  }

  motor_state = BSP_MotorControl_GetDeviceState(0);

  if(motor_state == INACTIVE){
	//converts angle values to step values
    target_position = map_fvalues(
    					  sensorVars->pot_angle[1],
                          POT_MIN_ANGLE,
                          POT_MAX_ANGLE,
						  POT_NUM_OF_STEPS_MIN,
						  POT_NUM_OF_STEPS_MAX
                        );
    BSP_MotorControl_GoTo(0, target_position);
  }

  //saved current angle
  sensorVars->pot_angle[0] = sensorVars-> pot_angle[1];
}


void MPPT_Task(SensorVar_t* sensorVars)
{
  uint16_t updated_duty_cycle;

  updated_duty_cycle = Perturb_N_Observe(
                        sensorVars->power_rectifier,
                        sensorVars->voltage_rectifier,
                        sensorVars->duty_cycle 
                      );

  //adjust duty cycle of PWM        
  htim10.Instance->CCR1  = updated_duty_cycle;
  sensorVars->duty_cycle = updated_duty_cycle;
  
#ifdef DEBUG_MPPT
  int32_t power_integral     = sensorVars->power_rectifier[1]/(SENSOR_RESOLUTION*SENSOR_RESOLUTION);
  uint32_t power_decimal      = sensorVars->power_rectifier[1]%(SENSOR_RESOLUTION*SENSOR_RESOLUTION);
  uint32_t voltage_integral   = sensorVars->voltage_rectifier[1]/SENSOR_RESOLUTION;
  uint32_t voltage_decimal    = sensorVars->voltage_rectifier[1]%SENSOR_RESOLUTION;
  int32_t current_integral   = sensorVars->current[1]/SENSOR_RESOLUTION;
  int32_t current_decimal    = sensorVars->current[1]%SENSOR_RESOLUTION;


  uint32_t v_load_integral   = sensorVars->voltage_load/SENSOR_RESOLUTION;
  uint32_t v_load_decimal    = sensorVars->voltage_load%SENSOR_RESOLUTION;

  int32_t delta_power            = sensorVars->power_rectifier[1] - sensorVars->power_rectifier[0];
  int32_t delta_voltage          = sensorVars->voltage_rectifier[1] - sensorVars->voltage_rectifier[0];
  uint32_t duty_cycle_percentage = sensorVars->duty_cycle*100/168;

  if(current_decimal > 0){
	  printf(
		"\r\npower = %d.%06d, v_rect = %d.%03d, V_boost = %d.%03dV  adc_val[1] = %d, current = %d.%03d, delta_power = %d, delta_voltage = %d, duty_cycle = %d",
		power_integral, power_decimal,
		voltage_integral, voltage_decimal,
		v_load_integral, v_load_decimal,
		g_adc_val[1],
		current_integral, current_decimal,
		delta_power, delta_voltage,
		duty_cycle_percentage
	  );
  }
  else{
	  current_decimal = -current_decimal;
	  printf(
		"\r\npower = %d.%06d, v_rect = %d.%03d, V_boost = %d.%03dV  adc_val[1] = %d, current = -%d.%03d, delta_power = %d, delta_voltage = %d, duty_cycle = %d",
		power_integral, power_decimal,
		voltage_integral, voltage_decimal,
		v_load_integral, v_load_decimal,
		g_adc_val[1],
		current_integral, current_decimal,
		delta_power, delta_voltage,
		duty_cycle_percentage
	  );
  }
#endif

  //save current values
  sensorVars->voltage_rectifier[0] = sensorVars->voltage_rectifier[1];
  sensorVars->current[0]           = sensorVars->current[1];
  sensorVars->power_rectifier[0]   = sensorVars->power_rectifier[1];
}


void PrintVariables(SensorVar_t* sensorVars)
{
#ifdef DEBUG_SENS
  uint32_t power_integral     = sensorVars->power_rectifier[1]/(SENSOR_RESOLUTION*SENSOR_RESOLUTION);
  uint32_t power_decimal      = sensorVars->power_rectifier[1]%(SENSOR_RESOLUTION*SENSOR_RESOLUTION);
  uint32_t voltage_integral   = sensorVars->voltage_rectifier[1]/SENSOR_RESOLUTION;
  uint32_t voltage_decimal    = sensorVars->voltage_rectifier[1]%SENSOR_RESOLUTION;
  uint32_t current_integral   = sensorVars->current[1]/SENSOR_RESOLUTION;
  uint32_t current_decimal    = sensorVars->current[1]%SENSOR_RESOLUTION;


  uint32_t v_load_integral   = sensorVars->voltage_load/SENSOR_RESOLUTION;
  uint32_t v_load_decimal    = sensorVars->voltage_load%SENSOR_RESOLUTION;

  printf(
	"\r\n  power = %d.%06dW, adc_val[0] = %fV, V_rectifier = %d.%03dV, adc_val[1] = %fV, current = %d.%03dA, adc_val[3] = %fV, V_load = %d.%03dV",
    power_integral, power_decimal,
    g_adc_val[0]*ADC_VAL_TO_VOLTS,
    voltage_integral,voltage_decimal,
    g_adc_val[1]*ADC_VAL_TO_VOLTS,
    current_integral, current_decimal,
    g_adc_val[3]*ADC_VAL_TO_VOLTS,
    v_load_integral, v_load_decimal
  );

#endif



#ifdef DEBUG_WIND_VANE
  float delta_angle  = sensorVars->pot_angle[1] -  sensorVars->pot_angle[0];
  printf(
	"\r\n adc_value2 = %d, angle = %f, previous_angle = %f, difference_angle = %f\n",
	g_adc_val[2],
	sensorVars->pot_angle[1],sensorVars->pot_angle[1],
	delta_angle
  );
#endif

}

uint16_t Perturb_N_Observe(int32_t power[], uint32_t voltage[], uint16_t duty_cycle)
{
  int32_t delta_power = power[1] - power[0];
  int32_t delta_voltage = voltage[1] - voltage[0];
  uint16_t new_duty_cycle = duty_cycle;

  /*
   * if dp/dv > 0, increase duty cycle
   * if dp/dv < 0, decrease duty cycle
   */
  if(delta_power > 0){
    if(delta_voltage > 0){
    	if(new_duty_cycle < DUTY_CYCLE_MAX){
    		new_duty_cycle += DUTY_CYCLE_STEP_SIZE ;
    	}
    }
    else if(delta_voltage < 0){
    	if(new_duty_cycle > DUTY_CYCLE_MIN)
    		new_duty_cycle -= DUTY_CYCLE_STEP_SIZE ;
    }
  }
  else if (delta_power < 0){
    if(delta_voltage > 0){
    	if(new_duty_cycle > DUTY_CYCLE_MIN){
    		new_duty_cycle-= DUTY_CYCLE_STEP_SIZE ;
    	}
    }
    else if(delta_voltage < 0){
    	if(new_duty_cycle < DUTY_CYCLE_MAX){
    		new_duty_cycle += DUTY_CYCLE_STEP_SIZE ;
    	}
    }
  }

  return new_duty_cycle;
}
