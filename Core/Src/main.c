/* USER CODE BEGIN Header */
/**
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
#include <string.h>
#include "../User/DEV_Config/DEV_Config.h"
#include "../User/PAJ7620U2/PAJ7620U2.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char statusAndLast10Gestures[11][4] = {0}; // 0 is master_ready, 1-10 is gestures newest->oldest
char gesture_name[4] = "NON\0";
uint8_t gesture_type = 0;
uint16_t gesture_data = 0;
volatile uint8_t master_ready = 1;
volatile uint8_t uart_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/*
 *@brief PAJ7620U2 Initialization
 */
unsigned char PAJ7620U2_init()
{
	unsigned char i, State, State1;
	DEV_Set_I2CAddress(PAJ7620U2_I2C_ADDRESS);
	DEV_I2C_WriteByte(PAJ_BANK_SELECT, 0);		// Select Bank 0
	DEV_Delay_ms(20);
	State = DEV_I2C_ReadByte(0x00);	// Read State
	State1 = DEV_I2C_ReadByte(0x01);
	if (State != 0x20 || State1 != 0x76)
		return 0;					// Wake up failed

	for (i=0;i< Init_Array;i++)
	{
		 DEV_I2C_WriteByte(Init_Register_Array[i][0], Init_Register_Array[i][1]); //Power up initialize
	}
	DEV_I2C_WriteByte(0x65, 0x12);
	for (i=0; i<Gesture_Array_SIZE; ++i)
	{
	  DEV_I2C_WriteByte(Init_Gesture_Array[i][0], Init_Gesture_Array[i][1]);
	}
	return 1;
}
/*
void uart_read_line(UART_HandleTypeDef* handler, char* buffer, uint16_t buffer_size) {
	HAL_StatusTypeDef status;
	char current_char;
	uint16_t char_counter = 0;
	while(char_counter < buffer_size-1) {
		status = HAL_UART_Receive(handler, &current_char, 1, 100);
		if(status == HAL_OK) {
			if(current_char == '\r' || current_char == '\n')
				if(char_counter == 0) continue;
				else break;
			*(buffer + char_counter++) = current_char;
		}
	}
	*(buffer + char_counter) = '\0';
}

void uart_write_line(UART_HandleTypeDef* handler, char* text) {
	HAL_UART_Transmit(handler, text, strlen(text), 1000);
	HAL_UART_Transmit(handler, "\r\n", 2, 100);

}

uint8_t esp_send_cmd(UART_HandleTypeDef* uart, char* command) {

	char response[30];
	response[0] = '\0';
	uart_write_line(uart, command);
	__HAL_UART_FLUSH_DRREGISTER(&huart1);

	while (strcmp(response, "OK") != 0
		&& strcmp(response, "no change") != 0
		&& strcmp(response, "ERROR") != 0)
		uart_read_line(uart, response, 30);
	if (strcmp(response, "ERROR") == 0) return 0;
	else return 1;

}

unsigned char ESP8266_init()
{
	HAL_Delay(500); // let's wait till module gets up
	if(!esp_send_cmd(&huart1, "AT")) return 0; // ping
	if (!esp_send_cmd(&huart1, "AT+CWMODE=1")) return 0; // configure as WiFi endpoint-device
	if (!esp_send_cmd(&huart1, "AT+CWJAP=\"pennyisafreeloader\",\"BigBangThe0ry\"")) return 0; // connect to guest WiFi at home
	if (!esp_send_cmd(&huart1, "AT+CIPSTART=\"TCP\",\"migu.ovh\",\"5000\"")) return 0; // connect to server hosting python echo script
	return 1;
}*/
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  //if(ESP8266_init()) HAL_UART_Transmit(&huart2, "CANT_START_ESP\0", 15, 100);
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_flag, 1); // wait for input from testing PC
  HAL_SPI_Receive_IT(&hspi2, (uint8_t*)&master_ready, 1); // wait for ACK from NodeMCU
  if(!PAJ7620U2_init()) HAL_UART_Transmit_DMA(&huart2, (uint8_t*)statusAndLast10Gestures, sizeof(statusAndLast10Gestures));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // for testing purposes no IT yet
	  //I2CReadGesture();
	  //HAL_SPI_TransmitReceive(&hspi2, 42, &master_ready, 1, 10); // debug send 42
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*
 *@brief UART Receive Callback -- Used for enable/disable algorithm from PC
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)statusAndLast10Gestures, sizeof(statusAndLast10Gestures));
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_flag, 1); // wait for input from testing PC
}

/*
 *@brief SPI Receive Callback -- Used for triggering gesture read and transfer to NodeMCU
 */

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	master_ready = 0;
	I2CReadGesture();
	HAL_SPI_Transmit(&hspi2, &gesture_type, 1, 10);
	HAL_SPI_Receive_IT(&hspi2, &master_ready, 1); // re-enable wait for ACK from NodeMCU
}

// I2CReadGesture - called by GPIO_PIN_11 EXTI ISR - incoming INT from gesture sensor
void I2CReadGesture()
{
	statusAndLast10Gestures[0][0] = master_ready;
	gesture_data = DEV_I2C_ReadWord(PAJ_INT_FLAG1);
	if (gesture_data)
	{
		gesture_type = gesture_data;
		switch (gesture_data)
		{
			case PAJ_UP:			  	strcpy(gesture_name, "U\0");	break;
			case PAJ_DOWN:				strcpy(gesture_name, "D\0");	break;
			case PAJ_LEFT:				strcpy(gesture_name, "L\0");	break;
			case PAJ_RIGHT:				strcpy(gesture_name, "R\0"); 	break;
			case PAJ_FORWARD:			strcpy(gesture_name, "FW\0");	break;
			case PAJ_BACKWARD:			strcpy(gesture_name, "BW\0"); 	break;
			case PAJ_CLOCKWISE:			strcpy(gesture_name, "CW\0"); 	break;
			case PAJ_COUNT_CLOCKWISE:	strcpy(gesture_name, "CCW\0"); 	break;
			default:
				 gesture_data=DEV_I2C_ReadByte(PAJ_INT_FLAG2);
				 if(gesture_data == PAJ_WAVE) strcpy(gesture_name, "WAV\0");
				 break;
		}
		memmove(&statusAndLast10Gestures[2], &statusAndLast10Gestures[1], 4*8*sizeof(char)); // last element gets overwritten
		//memmove(&statusAndLast10Gestures[2], &statusAndLast10Gestures[1], (sizeof(statusAndLast10Gestures)-2*4*sizeof(char))); // last element gets overwritten
		strcpy(statusAndLast10Gestures[1], gesture_name); // save new gesture
		HAL_Delay(50);
		HAL_UART_Transmit(&huart2, (uint8_t*)&gesture_name, sizeof(gesture_name), 0xFFFF);
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
