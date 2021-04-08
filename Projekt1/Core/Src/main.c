/* USER CODE BEGIN Header */
/*!
 * @file main.c
 *
 * @mainpage Inteligentny dom
 *
 * @section Strona Strona początkowa
 *
 * Dokumentacja projektu inteligentny dom
 *
 *W projekcie używam plytki STM32 F3 Discovery
 *Używam modółów DHT-11 pomiar temperatury i wilgotnosci,HC-05 moduł bluetooh,
 *alarm (buzzer) , czyjnik ruchy, czujnik natężenia swiatła.
 *
 * @section author Author
 *
 * Written by Hubert Leszczyński.
 */
/**
 *
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "DHT.h"
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
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


DHT_DataTypedef DHT;   //Struktura przechowywująca pomiary temperatury i wilgotnoisci
uint8_t znak;			//zmienna typu uint8_t do przechowywania znaku pobranego z uarta
char komunikat[100];	//zapisana wiadomość do wys łania uartem
typedef enum { F, T } boolean;  	//zmianny typu bool
boolean is_led_on=F;				//zmianna pokazujaca czy swiatło jest włączone
boolean alarm_test=F;				//zmienna pokazujacza czy test alarmu jest w łączony
uint8_t temperature_fan_on=20;   //temperatura po wyzej ktorej włącza sie wiatrak
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * Funkcja która włączącza swiatło ustawia stan wyjsciowy wysoki
 */
void led_on(void)
{
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_1,GPIO_PIN_SET); // turn on LED
}
/**
 * Funkcja która wyłączącza swiatło ustawia stan wyjsciowy niski
 */
void led_off(void)
{
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_1,GPIO_PIN_RESET); // turn off LED
}
/**
 * Funkcja obsługująca włączanie\wyłączanie swiatła w zależności czy uzytkownik włączył swiatło z telefonu,
 * lub czy któryś z czujnikow wysłał stan wysoki
 * pin D0 czujnik zmierzchu
 * pin D4 czujnik ruchu
 */
void light_sensor(void)
{
	if(is_led_on==T)
	{
		led_on();
	}
	else if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0)||HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)) // if the pin is HIGH
	{
		led_on();
	}
	else
	{
		led_off();
	}
}
/**
 * Funkcja która włączącza alarm, ustawia stan wyjsciowy niski
 */
void buzzer_on(void)
{
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3,GPIO_PIN_RESET); // turn on buzzer
}
/**
 * Funkcja która wyłączącza alarm, ustawia stan wyjsciowy wysoki
 */
void buzzer_off(void)
{
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3,GPIO_PIN_SET); // turn off buzzer
}
/**
 * Funkcja obsługująca włączanie\wyłączanie alarmu w zależności czy uzytkownik włączył test alarmu,
 * lub czy czujnik dymu nie przysłał stanu niskiego oznaczajacego wykrycie dymu
 * pin D2 czujnik dymu
 */
void gas_sensor(void)
{
	if(alarm_test==F)
	{
		if (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2)) // if the pin is LOW
		{
			buzzer_on();
		}
		else
		{
			buzzer_off();
		}
	}
	else
	{
		buzzer_on();
	}
}
/**
 * Funkcja obsługująca włączanie\wyłączanie wiatraka w zależności jaka temperatura została zmierzona
 * pin D6 wyjscie do tranzystora
 */
void fan_device(void)
{
	if(DHT.Temperature>temperature_fan_on)
	{
		HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6,GPIO_PIN_RESET);
	}
}


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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  //inicjalizacja przerwania uart
  HAL_UART_Receive_IT(&huart2,&znak, 1);
  //inicjalizacja przerwania timer
  HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 light_sensor();
	 gas_sensor();



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 35999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 59999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 17999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD0 PD2 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * Funkcja obsługujaca przerwanie uart
 * Stosowany do obsługi komunikacji bluetooth.
 * @param huart wskaźnik na zmienną która zgłosiła przerwanie
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//sprawdzanie któe przerwanie przyszło z usart2 czyli bluetooth.
	if(huart->Instance==USART2)
	{
		//włącznie swiatła
		if(znak==1)
		{

			is_led_on=T;
			sprintf(komunikat,"-LED_ON.\n");
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)komunikat,strlen(komunikat));
		}
		//wyłącznie swiatła
		else if(znak==0)
		{
			is_led_on=F;
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6,GPIO_PIN_RESET);
			sprintf(komunikat,"-LED_OFF.\n");
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)komunikat,strlen(komunikat));
		}
		//pomiar temperatury i wysłamnie wartosci na telefon
		else if(znak==2)
		{	DHT_GetData(&DHT);
		    fan_device();
			sprintf(komunikat,"-TEMPERATURA:%d.\n",(int)DHT.Temperature);
		    HAL_UART_Transmit_IT(&huart2,(uint8_t*)komunikat,strlen(komunikat));
		}
		//pomiar wilgotnosci i wysłamnie wartosci na telefon
		else if(znak==3)
		{	DHT_GetData(&DHT);
			sprintf(komunikat,"-WILGOTNOSC:%d.\n",(int)DHT.Humidity);
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)komunikat,strlen(komunikat));
		}
		//włącznie alarmu testowego i włączenie timera
		else if(znak==4)
		{
			alarm_test=T;
			HAL_TIM_Base_Start_IT(&htim17);
			sprintf(komunikat,"-TEST_ALARMU.\n");
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)komunikat,strlen(komunikat));
		}


		//oczekiwanie na kolejny przesłaną wiadomosc
		HAL_UART_Receive_IT(&huart2,&znak, 1);
	}
}

/**
 * * Funkcja obsługujaca przerwanie timerów
 * @param htim wskaźnik na zmienną zgłaszającą przerwanie
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//wysyłanie pomiaru temperatury co 1 min
	if(htim->Instance==TIM16)
	{
		DHT_GetData(&DHT);
		sprintf(komunikat,"-TEMPERATURA:%d\n WILGOTNOSC:%d.\n",(int)DHT.Temperature,(int)DHT.Humidity);
		HAL_UART_Transmit_IT(&huart2,(uint8_t*)komunikat,strlen(komunikat));
	}
	//po czasie 5 sekund wyłączanie alarmu
	if(htim->Instance==TIM17)
		{
		alarm_test=F;
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
