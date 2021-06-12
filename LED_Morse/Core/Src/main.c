/* USER CODE BEGIN Header */
/****
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
#include "lcd.h"
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define MAX_CHAR_LENGTH 4 	// ELements of a single morse character
#define DOT 300				// Duration in miliseconds of a DOT
#define LINE 900			// Duration in miliseconds of a LINE
#define ALPHABET_SIZE 26 	// Number of letters in alphabet
#define PAUSE 300			// Time between flashes
#define PAUSE_L 600			// Time between letters
#define SPACE 2100			// Time between words
#define WAIT 5000			// Time between conversions

/*LED pin*/
#define LED_GPIO GPIOA			// Output GPIO
#define LED_PIN GPIO_PIN_12		// Output pin (Pin D2 on board)

/*LCD screen pins*/
#define D0_GPIO_Port GPIOB		// Data 0 GPIO port
#define D1_GPIO_Port GPIOB		// Data 1 GPIO port
#define D2_GPIO_Port GPIOB		// Data 2 GPIO port
#define D3_GPIO_Port GPIOB		// Data 3 GPIO port
#define D4_GPIO_Port GPIOA		// Data 4 GPIO port
#define D5_GPIO_Port GPIOA		// Data 5 GPIO port
#define D6_GPIO_Port GPIOB		// Data 6 GPIO port
#define D7_GPIO_Port GPIOB		// Data 7 GPIO port

#define D0_Pin GPIO_PIN_0		// Data 0 pin (D3 on board)
#define D1_Pin GPIO_PIN_7		// Data 0 pin (D4 on board)
#define D2_Pin GPIO_PIN_6		// Data 0 pin (D5 on board)
#define D3_Pin GPIO_PIN_1		// Data 0 pin (D6 on board)
#define D4_Pin GPIO_PIN_8		// Data 0 pin (D9 on board)
#define D5_Pin GPIO_PIN_11		// Data 0 pin (D10 on board)
#define D6_Pin GPIO_PIN_5		// Data 0 pin (D11 on board)
#define D7_Pin GPIO_PIN_4		// Data 0 pin (D12 on board)

#define RS_GPIO_Port GPIOA		 // Register select GPIO port
#define RS_Pin 		 GPIO_PIN_9  // Register select pin (D1)
#define EN_GPIO_Port GPIOA		 // Enable GPIO port
#define EN_Pin		 GPIO_PIN_10 // Register select pin (D0)

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	char capitalC, lowerC;
	uint8_t length;
	uint16_t morse[MAX_CHAR_LENGTH];
}MORSE_CHAR; // Struct for each element of morse code

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
uint8_t String2Morse(char *, uint8_t, MORSE_CHAR *, Lcd_HandleTypeDef);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

	/*General purpose variables*/
	unsigned int x = 0; // Conversions counter
	char myname[] = "David Calles\n"; // Input string

	// GPIO Ports for LCD
	Lcd_PortType ports[] = {
			D0_GPIO_Port, D1_GPIO_Port, D2_GPIO_Port, D3_GPIO_Port,
			D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port
	  };
	// GPIO Pins for LCD
	Lcd_PinType pins[] = {D0_Pin, D1_Pin, D2_Pin, D3_Pin,
						  D4_Pin, D5_Pin, D6_Pin, D7_Pin};
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); // Manually modified to include LED_PIN
  MX_USART2_UART_Init();

  // Create handler for the LCD
  Lcd_HandleTypeDef lcd;
  lcd = Lcd_create(ports, pins,
		  	  	  RS_GPIO_Port, RS_Pin,
		  	  	  EN_GPIO_Port, EN_Pin,
				  LCD_8_BIT_MODE);


  //Lcd_clear(&lcd);

  /* Infinite loop---------------------------------------------------------*/
  while (1){
	// Show Initial message
	Lcd_clear(&lcd);
	Lcd_cursor(&lcd, 0,1);
	Lcd_string(&lcd, "String 2 Morse");
	Lcd_cursor(&lcd, 1,0);
	Lcd_string(&lcd, "by David Calles!");
	HAL_Delay(5000);

	printf("Initiate conversion %d \r\n", x);
	String2Morse(myname, 13, alphabet, lcd); // Actual morse output
	printf("End conversion %d \r\n", x++);




	HAL_Delay(WAIT);
  } // End while(1)
} // End Main

/**
  * @brief String2Morse
  * @retval Characters processed
  */
uint8_t String2Morse(char *name, uint8_t n, MORSE_CHAR *alphabet, Lcd_HandleTypeDef lcd){

	//Index calculation and cases evaluation
	uint8_t i = 0;
	uint8_t index = 0;
	char dispC[] = "  ";
	char dispM[] = "     ";
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
			dispM[0] = ' ', dispM[1] = ' ', dispM[2] = ' ', dispM[3] = ' ';
			for(uint8_t j = 0; j < alphabet[index].length; j++){
				//Turn LED ON,
				HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_SET);
				//Keep led on for a specific time
				HAL_Delay(alphabet[index].morse[j]);
				//Turn LED OFF,
				HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_RESET);
				//Create morse string for lcd
				if(alphabet[index].morse[j]==DOT)
					dispM[j]='.';
				if(alphabet[index].morse[j]==LINE)
					dispM[j]='_';
				HAL_Delay(PAUSE);
			}// end for j

			printf("%c\n\r", alphabet[index].capitalC); //Print detected char
			// Show letter on LCD
			dispC[0]= alphabet[index].capitalC;
			Lcd_clear(&lcd);
			Lcd_cursor(&lcd, 0, 7);
			Lcd_string(&lcd, dispC);

			// Show morse on LCD
			Lcd_cursor(&lcd, 1, 7);
			Lcd_string(&lcd, dispM);
			HAL_Delay(PAUSE_L);
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
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|LD3_Pin|GPIO_PIN_4
						  |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
						  |GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure LED Output level*/
	HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_RESET);

	/*Configure GPIO pin : VCP_TX_Pin */
	GPIO_InitStruct.Pin = VCP_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 LD3_Pin PB4
						   PB5 PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD3_Pin|GPIO_PIN_4
						  |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	if(LED_GPIO == GPIOB) // Add Led pin if part of GPIOB
	  GPIO_InitStruct.Pin |= LED_PIN;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 PA10 PA11
						   PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
						  |GPIO_PIN_12;
	if(LED_GPIO == GPIOA)// Add Led pin if part of GPIOA
	  GPIO_InitStruct.Pin |= LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : VCP_RX_Pin */
	GPIO_InitStruct.Pin = VCP_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
	HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);
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
