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
#include "math.h"

/* uncomment to enable printf statements */
#define ENABLE_PRINTF
//#define DEBUG_SENS
#define DEBUG_MPPT
//#define DEBUG_WIND_VANE

/* uncomment to enable the following hardware */
#define ENABLE_MPPT
#define ENABLE_MOTORS


#define ADC_VAL_TO_VOLTS                0.0008058608
#define ADC_12B_RESOLUTION              4095

#define SENSOR_RESOLUTION               1000

//#define VCC_DIV_BY_2					2.5

#define V_LOAD_TRANSFER_RATIO           5.16
#define V_RECTIFIER_TRANSFER_RATIO      2.2

#define I_SENS_V_TO_I_RATIO				0.4 // slope of current sensor is 400mV/A

#define I_SENS_ADC_ZERO_VAL             3100
#define I_SENS_ADC_3V3_VAL              4095
#define I_SENS_MIN_AMPS                 0.0
#define I_SENS_MAX_AMPS                 2.0

#define POT_ADC_ZERO_ANGLE_VAL          7
#define POT_ADC_MAX_ANGLE_VAL           2467
#define POT_MIN_ANGLE                   1.0   //in degrees
#define POT_MAX_ANGLE                   300.0 //in degrees

#define POT_NUM_OF_STEPS_MIN			0.0   //in steps
#define	POT_NUM_OF_STEPS_MAX			5162 //in steps;gear ratio 120/62
#define MOTOR_NUM_STEPS_TO_360Deg		3200 //in steps

#define DUTY_CYCLE_STEP_SIZE            2		//1%
#define DUTY_CYCLE_MIN 					17 		//10% duty cycle
#define DUTY_CYCLE_MAX 					151		//90% duty cycle
#define PWM_TIMER_PERIOD				168

typedef enum{
    READ_SENSORS,
    MPPT_TASK,
    MOTOR_TASK,
    PRINT_VARIABLES,
} ControlState_e;

typedef struct{
   uint32_t voltage_rectifier[2];    //voltage at the rectifier, where index = 1 means current val

   uint32_t voltage_load;            //Voltage at the load

   int32_t current[2];              //current at the rectifier

   int32_t power_rectifier[2];      //Power at the rectifier

   int32_t power_load;              //Power at the load
      
   uint16_t duty_cycle;              //duty cycle of the MOSFET

   float pot_angle[2];             //potentiometer angle
}SensorVar_t;



void StepperMotor_App_Init(void);
void StepMotor_Task(SensorVar_t* sensVar);
void ReadSensor_Task(SensorVar_t* sensVars);
void PrintVariables(SensorVar_t* sensorVars);
void MPPT_Task(SensorVar_t* sensVars);
uint16_t Perturb_N_Observe(int32_t power[], uint32_t voltage[], uint16_t duty_cycle);


#endif /* INC_APP_H_ */
