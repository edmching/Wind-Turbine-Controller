/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "steppermotor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile bool g_is_conversion_ready = false;
volatile uint32_t g_adc_val[NUM_OF_CONVERSIONS], g_adc_buf[ADC_BUFFER_LENGTH];
Stepmotor_Status motor_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t Perturb_N_Observe(uint32_t power[], uint16_t voltage[], uint16_t current[], uint16_t duty_cycle);
float map_values(int32_t val, int32_t input_min, int32_t input_max, int32_t output_min, int32_t output_max);
float map_fvalues(float val, float input_min, float input_max, float output_min, float output_max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
 {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  /* assume v > 0, i > 0  */
  float voltage[2], current[2];
  const float v_transfer_ratio = 39.0/139.0;
  const float adc_val_to_volts = 3.3/4095.0;
  const float I_sens_adc_zero_val =  2968.0;
  const float I_sens_adc_3V3_val = 4095;
  const uint16_t pot_zero_angle = 1861;
  const uint16_t pot_max_angle = 3575;
  float angle, previous_angle, delta_angle;
  uint32_t power[2];
  uint16_t duty_cycle;
  bool conversion_ready;
  float vtest = 0.0;
  angle = 0.0;
  previous_angle = 0.0;
  delta_angle = 0.0;
 
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &g_adc_buf, ADC_BUFFER_LENGTH);

 // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 // duty_cycle = 50*168/100; //50% duty cycle
  //htim1.Instance->CCR1 = duty_cycle;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//check if converison is finished//
    __disable_irq();
    conversion_ready = g_is_conversion_ready;
    __enable_irq();
    if(conversion_ready == true)
    {
      //read mppt sens data
      voltage[1] = (g_adc_val[0]*adc_val_to_volts)/(v_transfer_ratio);
      current[1] = map_fvalues(g_adc_val[1], I_sens_adc_zero_val, I_sens_adc_3V3_val, 0.0, 20.0);
      //vtest = g_adc_val[2]*adc_val_to_volts;
      //read wind vane data
      angle = map_fvalues(g_adc_val[2], pot_zero_angle, pot_max_angle, 0.0, 360.0);

      __disable_irq();
      g_is_conversion_ready = false;
      __enable_irq();
      delta_angle = angle - previous_angle;
      //printf("\r\n adc_value2 = %d, angle = %d, previous_angle = %d, difference_angle = %d \n", g_adc_val[2], angle, previous_angle, diff_angle);
      printf("\r\n adc_value0 = %d, voltage[1] = %f, adc_value1 = %d, current[1] = %f, adc_value2 = %d, angle = %f, previous_angle = %f, delta_angle = %f ",
    		  g_adc_val[0], voltage[1], g_adc_val[1], current[1], g_adc_val[2], angle, previous_angle, delta_angle);

      //updates the previous values
      voltage[0] = voltage[1];
      current[0] = current[1];
      previous_angle = angle;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint16_t Perturb_N_Observe(uint32_t power[], uint16_t voltage[], uint16_t current[], uint16_t duty_cycle) //might need to change to signed because voltage
{
  int32_t delta_power = power[1] - power[0];
  int32_t delta_voltage = voltage[1] - voltage[0];
  uint16_t new_duty_cycle = duty_cycle;
  const int8_t duty_cycle_step = 1;

 /*printf("\r\n power = %d, voltage = %d, current = %d, delta_power = %d, delta_voltage = %d, duty_cycle = %d",
		 power[1], voltage[1], current[1], delta_power, delta_voltage, duty_cycle*100/168);
	*/
  /*
   * if dp/dv > 0, increase duty cycle
   * if dp/dv < 0, decrease duty cycle
   */
  if(delta_power > 0){
    if(delta_voltage > 0){
    	if(new_duty_cycle < DUTY_CYCLE_MAX){
    		new_duty_cycle += duty_cycle_step;
    	}
    }
    else if(delta_voltage < 0){
    	if(new_duty_cycle > DUTY_CYCLE_MIN)
    		new_duty_cycle -= duty_cycle_step;
    }
  }
  else if (delta_power < 0){
    if(delta_voltage > 0){
    	if(new_duty_cycle > DUTY_CYCLE_MIN){
    		new_duty_cycle-= duty_cycle_step;
    	}
    }
    else if(delta_voltage < 0){
    	if(new_duty_cycle < DUTY_CYCLE_MAX){
    		new_duty_cycle += duty_cycle_step;
    	}
    }
  }

  return new_duty_cycle;
}

float map_values(int32_t val, int32_t input_min, int32_t input_max, int32_t output_min, int32_t output_max)
{
	float slope = 1.0 * (output_max - output_min)/(input_max-input_min);
	return (val - input_min)*(slope) + output_min;
}

float map_fvalues(float val, float input_min, float input_max, float output_min, float output_max)
{
	float slope = 1.0 * (output_max - output_min)/(input_max-input_min);
	return (val - input_min)*(slope) + output_min;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
