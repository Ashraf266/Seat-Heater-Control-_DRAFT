/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"


#define HIGH_PRIORITY 		3
#define MEDIUM_PRIORITY 	2
#define LOW_PRIORITY		 	1

void SystemClock_Config(void);


TaskHandle_t Driving_Wheel_Button_Task_Handler;
TaskHandle_t Driver_Button_Task_Handler;
TaskHandle_t Passanger_Button_Task_Handler;
TaskHandle_t Driver_Seat_Temp_Read_Task_Handler;
TaskHandle_t Passanger_Seat_Temp_Read_Task_Handler;
TaskHandle_t Driver_Seat_Heater_Task_Handler;
TaskHandle_t Passanger_Seat_Heater_Task_Handler;
TaskHandle_t Display_Task_Handler;
TaskHandle_t Control_Task_Handler;
TaskHandle_t Failure_Task_Handler;
TaskHandle_t Diagnostics_Task_Handler;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* Initialization */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
	
	
	
	
	xTaskCreate(Task, "Driving Wheel Button Task", 					256, NULL, HIGH_PRIORITY, 	&Driving_Wheel_Button_Task_Handler);
	xTaskCreate(Task, "Driver Seat Console Button Task", 		256, NULL, HIGH_PRIORITY, 	&Driver_Button_Task_Handler);
	xTaskCreate(Task, "Passenger Seat Console Button Task", 256, NULL, HIGH_PRIORITY, 	&Passanger_Button_Task_Handler);
	xTaskCreate(Task, "Driver Seat Temp Read Task", 				256, NULL, MEDIUM_PRIORITY, &Driver_Seat_Temp_Read_Task_Handler);
	xTaskCreate(Task, "Passenger Seat Temp Read Task", 			256, NULL, MEDIUM_PRIORITY, &Passanger_Seat_Temp_Read_Task_Handler);
	xTaskCreate(Task, "Driver Seat Heater Task", 						256, NULL, MEDIUM_PRIORITY, &Driver_Seat_Heater_Task_Handler);
	xTaskCreate(Task, "Passanger Seat Heater Task", 				256, NULL, MEDIUM_PRIORITY, &Passanger_Seat_Heater_Task_Handler);
	xTaskCreate(Task, "Display Task", 											256, NULL, LOW_PRIORITY, 		&Display_Task_Handler);
	xTaskCreate(Task, "Control Task", 											256, NULL, MEDIUM_PRIORITY, &Control_Task_Handler);
	xTaskCreate(Task, "Failure Handling Task", 							256, NULL, HIGH_PRIORITY, 	&Failure_Task_Handler);
	xTaskCreate(Task, "Diagnostics Task", 									256, NULL, HIGH_PRIORITY, 	&Diagnostics_Task_Handler);

	
	
	
	/* Start the scheduler so the created tasks start executing. */
	vTaskStartScheduler();
	
	
	/* The following line should never be reached. */
	for (;;);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

