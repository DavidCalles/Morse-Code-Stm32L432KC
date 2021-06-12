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
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define MAX_CHAR_LENGTH 4 	// ELements of a single morse character
#define DOT 300				// Duration in miliseconds of a DOT
#define LINE 900			// Duration in miliseconds of a LINE
#define ALPHABET_SIZE 26 	// Number of letters in alphabet
#define PAUSE 300			// Time between letters
#define SPACE 2100			// Time between words
#define WAIT 5000			// Time between conversions

#define GPIO GPIOA			// Output GPIO
#define PIN GPIO_PIN_12		// Output pin (Pin D2 on board) (HAL definitions)

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	char capitalC, lowerC;
	uint8_t length;
	uint16_t morse[MAX_CHAR_LENGTH];
}MORSE_CHAR;

typedef enum {
	NOTSHOW = 0,
	SHOW    = 1
}CHAR_STATE;

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
/*CUBEMX defined*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/*User Defined*/
uint8_t String2Morse(char *, uint8_t, MORSE_CHAR *);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*Morse alphabet*/
 MORSE_CHAR alphabet[ALPHABET_SIZE] = {
			{'A', 'a', 2, {DOT,		LINE,	0,		0}},
			{'B', 'b', 4, {LINE,	DOT,	DOT,	DOT}},
			{'C', 'c', 4, {LINE,	DOT,	LINE,	DOT}},
			{'D', 'd', 3, {LINE,	DOT,	DOT,	0}},
			{'E', 'e', 1, {DOT,		0,		0,  	0}},
			{'F', 'f', 4, {DOT,		DOT,	LINE,  	DOT}},
			{'G', 'g', 3, {LINE,	LINE,	DOT,  	0}},
			{'H', 'h', 4, {DOT,		DOT,	DOT,  	DOT}},
			{'I', 'i', 2, {DOT,		DOT,	0, 		0}},
			{'J', 'j', 4, {DOT,		LINE,	LINE,  	LINE}},
			{'K', 'k', 3, {LINE,	DOT,	LINE,  	0}},
			{'L', 'l', 4, {DOT,		LINE,	DOT,  	DOT}},
			{'M', 'm', 2, {LINE,	LINE,	0,  	0}},
			{'N', 'n', 2, {LINE,	DOT,	0,  	0}},
			{'O', 'o', 3, {LINE,	LINE,	LINE,  	0}},
			{'P', 'p', 4, {DOT,		LINE,	LINE,  	DOT}},
			{'Q', 'q', 4, {LINE,	LINE,	DOT,  	LINE}},
			{'R', 'r', 3, {DOT,		LINE,	DOT, 	0}},
			{'S', 's', 3, {DOT,		DOT,	DOT,  	0}},
			{'T', 't', 1, {LINE,	0,		0,  	0}},
			{'U', 'u', 3, {DOT,		DOT,	LINE, 	0}},
			{'V', 'v', 4, {DOT,		DOT,	DOT,  	LINE}},
			{'W', 'w', 3, {DOT,		LINE, 	LINE,	0}},
			{'X', 'x', 4, {LINE,	DOT,	DOT,	LINE}},
			{'Y', 'y', 4, {LINE,	DOT,	LINE,	LINE}},
			{'Z', 'z', 4, {LINE,	LINE,	DOT,	DOT}}
	};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  GPIO_InitTypeDef GPIO_InitStruct2 = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO, PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct2.Pin = PIN;
  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct2.Pull = GPIO_NOPULL;
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO, &GPIO_InitStruct2);

  /*General purpose variables*/
  unsigned int x = 0;
  char myname[] = "A A B B C C\n"; // Input string

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	printf("Initiate conversion %d \r\n", x);
	String2Morse(myname, 13, alphabet);
	printf("End conversion %d \r\n", x++);
	HAL_Delay(WAIT);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief String2Morse
  * @retval None
  */
uint8_t String2Morse(char *name, uint8_t n, MORSE_CHAR *alphabet){

	//Index calculation and cases evaluation
	uint8_t i = 0;
	uint8_t index = 0;
	CHAR_STATE state = NOTSHOW;
	// Iterate through every character in string
	for(i=0; i<n; i++){
		// Evaluate the type of character
		switch(name[i]){
			// Capital case letters evaluation according to ASCII.
			case 'A' ... 'Z':{
				index = name[i]-'A';
				state = SHOW;
				break;
			}//end case
			// Lower case letters evaluation according to ASCII.
			case 'a' ... 'z':{
				index = name[i]-'a';
				state = SHOW;
				break;
			}//end case
			// Space evaluation
			case ' ':{
				HAL_Delay(SPACE);
				state = NOTSHOW;
				break;
			}//end case
			// Other special characters will just be ignored
			default:{
				state = NOTSHOW;
				break;
			}//end default case
		}//end switch

		// Output of the morse code of the character
		if (state == SHOW){

			printf("%c\n\r", alphabet[index].capitalC); //Print detected char

			for(uint8_t j = 0; j < alphabet[index].length; j++){
				//Turn LED ON,
				HAL_GPIO_WritePin(GPIO, PIN, GPIO_PIN_SET);
				//Keep led on for a specific time
				HAL_Delay(alphabet[index].morse[j]);
				//Turn LED OFF,
				HAL_GPIO_WritePin(GPIO, PIN, GPIO_PIN_RESET);
				HAL_Delay(PAUSE);
			}// end for j
		}//end if (state == SHOW)

	}//end for i
	return i; // Number of characters processed
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
