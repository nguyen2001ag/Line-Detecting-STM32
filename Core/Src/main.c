/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "Read_sensor.h"
#include "string.h"
#include "FB.h"
#include "stdio.h"
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
char Rx_data[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

PID_parameter PID_set_parameters = {.Kp =5,.Ki=0.1,.Kd=0.001,.Ts = 0.01,.PID_Saturation = 30
                                                                              ,.error =0.0,.pre_error =0.0,.pre2_error=0.0,.Out_left=0,.Out_right=0,.pre_Out_left=0,.pre_Out_right=0};
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
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_Base_Start(&htim17);
  HAL_UART_Receive_IT(&huart1, &Rx_data, 1);

//  HAL_TIM_Base_Stop_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	float error;
	float pre_error = 0;
	if(htim->Instance == htim6.Instance)
	{
		error= read_sensor_values(pre_error);
		PID_PROCESS(&PID_set_parameters, error, 0);
		forward();
		HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

		uint8_t right_value = (uint8_t)(PID_ReadValue_right (&PID_set_parameters))+40;
		uint8_t left_value = (uint8_t)(PID_ReadValue_left (&PID_set_parameters))+40;
		__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,right_value);
		__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,left_value);

		pre_error=error;
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	if (huart->Instance == USART1)  //current UART
	{
		if(Rx_data[0] == 'f'){
								HAL_TIM_Base_Stop_IT(&htim6);
								forward();
								HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
								HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
								__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1, 80);
								__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1, 80);
							}

		else if(Rx_data[0] == 'b'){
								HAL_TIM_Base_Stop_IT(&htim6);
								backward();
								HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
								HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
								__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1, 80);
								__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1, 80);
							}

		else if(Rx_data[0] == 'l'){
								HAL_TIM_Base_Stop_IT(&htim6);
								forward();
								HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
								HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
								__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1, 0);
								__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1, 80);
							}
		else if(Rx_data[0] == 'r'){
								HAL_TIM_Base_Stop_IT(&htim6);
								forward();
								HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
								HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
								__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1, 80);
								__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1, 0);
							}
		else if(Rx_data[0] == 'i'){
								HAL_TIM_Base_Stop_IT(&htim6);
								__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,0);
								__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,0);
							}




		else if(Rx_data[0] == 'k'){
						HAL_TIM_Base_Start_IT(&htim6);
	    }
		else if(Rx_data[0] == 'd'){
			HAL_TIM_Base_Stop_IT(&htim6);
		}
	 HAL_UART_Receive_IT(&huart1, (uint8_t *) Rx_data, 1);
}
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
