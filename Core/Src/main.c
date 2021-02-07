/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

#define REF_CLOCK	4000000

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start(&htim4);
  +HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  char buf[100];

  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

  send(0x9F); // Set attenuation channel 0
  send(0xBF); // Set attenuation channel 1
  send(0xDF); // Set attenuation channel 2
  send(0xFF); // Set attenuation noise channel

  send( 0xF0 );

/*
  // White Noise N/512
  send( 0b11100100);
  HAL_Delay(1000);

  // White Noise N/1024
  send( 0b11100101);
  HAL_Delay(1000);

  // White Noise N/2048
  send( 0b11100110);
  HAL_Delay(1000);

  // Periodic Noise N/512
  send( 0b11100000);
  HAL_Delay(1000);

  // Periodic Noise N/1024
  send( 0b11100001);
  HAL_Delay(1000);

  // Periodic Noise N/2048
  send( 0b11100010);
  HAL_Delay(1000);
*/

  // Noise associated with Channel 2 tone
  for ( int i = 1000 ; i < 8000 ; i+= 1000 )
  {

	  // White Noise
	  play( 2, i );
	  send(0xDF); // Set attenuation channel 2
	  send( 0b11100011);
	  HAL_Delay(500);

	  // Periodic Noise
	  play( 2, i );
	  send(0xDF); // Set attenuation channel 2
	  send( 0b11100111);
	  HAL_Delay(500);
  }

  send( 0xFF );



  /*
  for ( int i = 1 ; i < 4 ; i++ )
  {

	  send(0x83);
	  send(i+5);

	  //send(0xA3);
	  //send(i);

	  //send(0xC3);
	  //send(i+10);


	  HAL_Delay(500);

  }
*/


  /*
  play( 0, 261 );
  HAL_Delay(400);

  play( 0, 293 );
  HAL_Delay(400);

  play( 0, 329 );
  HAL_Delay(400);

  play( 0, 349 );
  HAL_Delay(400);

  play( 0, 392 );
  HAL_Delay(400);

  play( 0, 440 );
  HAL_Delay(400);

  play( 0, 493 );
  HAL_Delay(400);

  play( 0, 523 );
  HAL_Delay(400);
*/

  /*

  play( 0, 261 );
  HAL_Delay(400);

  play( 1, 329 );
  HAL_Delay(400);

  play( 2, 392 );
  HAL_Delay(4000);

  stop(0);
  stop(1);
  stop(2);
*/

  while (1)
  {

	  HAL_StatusTypeDef status;

	  status = HAL_UART_Receive( &huart1, (uint8_t*)buf, 1, 1000 );
	  if ( status == HAL_OK )
	  {
		  HAL_GPIO_TogglePin( GPIOC, GPIO_PIN_13 );
		  int i = (int) buf[0] - 'a';

		  int freq = REF_CLOCK / (i << 4) / 32;
		  sprintf( buf, "Frequency: %d\r\n", freq );

		  HAL_UART_Transmit( &huart1, (uint8_t*)buf, strlen(buf), 1000 );

		  send(0x90); // Set attenuation channel 0
		  //send(0xB0); // Set attenuation channel 0

		  send(0x80);
		  send(i);
		  //send(0xA0);
		  //send(i+10);
		  HAL_Delay(500);

		  send(0x9F); // Set attenuation channel 0
		  send(0xBF); // Set attenuation channel 0

	  }

	  //HAL_GPIO_TogglePin( GPIOC, GPIO_PIN_13 );
	  //HAL_Delay(500);


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/* USER CODE BEGIN 4 */


void play( int channel, int freq )
{
	int tenBitValue = REF_CLOCK / ( 32 * freq );

	int lastFourBits = tenBitValue & 0xf;
	int firstSixBit = tenBitValue >> 4;

	int soundReg = 8 + channel * 2;
	int attenuationReg = soundReg + 1;

	send( attenuationReg << 4 );

	send( (soundReg << 4) + lastFourBits );
	send(firstSixBit);
}

stop( int channel )
{
	int attenuationReg = ( 8 + channel*2 + 1 ) << 4;
	send( attenuationReg + 0xf );
}

void send(uint8_t b)
{
	  HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, (b&128)?GPIO_PIN_SET:GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, (b&64)?GPIO_PIN_SET:GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, (b&32)?GPIO_PIN_SET:GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, (b&16)?GPIO_PIN_SET:GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (b&8)?GPIO_PIN_SET:GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (b&4)?GPIO_PIN_SET:GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (b&2)?GPIO_PIN_SET:GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (b&1)?GPIO_PIN_SET:GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1);

	  /*
	  int status = 0;
	  do {
		  status = HAL_GPIO_ReadPin( RE_GPIO_Port, RE_Pin );
	  } while ( status == 0 );
*/

	  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, GPIO_PIN_SET);

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
