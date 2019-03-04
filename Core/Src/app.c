/*
 * app.c
 *
 *  Created on: Mar 3, 2019
 *      Author: Edm
 */
#include "app.h"

/*
 * TODO: restructure code
void init_mppt_sens(mppt_sens_data* sens_data)
{

	//wait to get first sample
	__disable_irq();
	int conversion_ready = g_is_conversion_ready;
	__enable_irq();
	while(conversion_ready != true){
	  __disable_irq();
	  conversion_ready = g_is_conversion_ready;
	  __enable_irq();

	}

	//calculate initial values
	sens_data->voltage[0] = map_values(g_adc_val[0], 0, ADC_12B_MAX_RESOLUTION, 0, V_SENS_MAX*SENSOR_RESOLUTION);
	sens_data->current[0] = map_values(g_adc_val[1], 0, ADC_12B_MAX_RESOLUTION, 0, I_SENS_MAX*SENSOR_RESOLUTION);
	sens_data->power[0] = sens_data->voltage[0]*sens_data->current[0];

	printf("\r\n adc_value0 = %d, voltage[1] = %d, voltage[0]= %d", g_adc_val[0], sens_data->voltage[1], sens_data->voltage[0]);
	printf("\r\n adc_value1 = %d, current[1] = %d, current[0] = %d", g_adc_val[1], sens_data->current[1], sens_data->current[0]);

	__disable_irq();
	g_is_conversion_ready = false;
	__enable_irq();
}

void run_mppt(mppt_sens_data* sens_data)
{

	__disable_irq();
	int conversion_ready = g_is_conversion_ready;
	__enable_irq();
	if(conversion_ready == true)
	{
	  sens_data->voltage[1] = map_values(g_adc_val[0], 0, ADC_12B_MAX_RESOLUTION, 0, V_SENS_MAX*SENSOR_RESOLUTION);
	  sens_data->current[1] = map_values(g_adc_val[1], 0, ADC_12B_MAX_RESOLUTION, 0, I_SENS_MAX*SENSOR_RESOLUTION);
	  sens_data->power[1] = sens_data->voltage[1]*sens_data->current[1];

	  printf("\r\n adc_value0 = %d, voltage[1] = %d, voltage[0]= %d\n", g_adc_val[0], sens_data->voltage[1], sens_data->voltage[0]);
	  printf("\r\n adc_value1 = %d, current[1] = %d, current[0] = %d\n", g_adc_val[1], sens_data->current[1], sens_data->current[0]);

	  sens_data->duty_cycle = Perturb_N_Observe(sens_data->power, sens_data->voltage, sens_data->current, sens_data->duty_cycle);

	  htim1.Instance->CCR1 = sens_data->duty_cycle;
	  printf("\r\n power[1] = %d, power[0] = %d, duty_cycle = %d %\n", sens_data->power[1], sens_data->power[0], sens_data->duty_cycle*100/168);

	  //updates the previous values
	  sens_data->voltage[0] = sens_data->voltage[1];
	  sens_data->current[0] = sens_data->current[1];
	  sens_data->power[0] = sens_data->power[1];

	  __disable_irq();
	  g_is_conversion_ready = false;
	  __enable_irq();
	}
	HAL_Delay(1000);
}
*/
