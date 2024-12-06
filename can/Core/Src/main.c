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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI1_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define SPI1_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  mcp2515init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //mcp2515messageAvailable();
	  HAL_Delay(500);
	  mcp2515readMessage();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_INT_Pin */
  GPIO_InitStruct.Pin = CAN_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CAN_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print(const char* buffer) {
	 // Correctly calculate the string length
	    size_t length = strlen(buffer);

	    // Transmit the string over UART
	    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, length, 100);
}

void mcp2515writeRegister(uint8_t address, uint8_t data){

	uint8_t txBuffer[3] = {0x02, address, data};

	SPI1_CS_LOW();
	HAL_SPI_Transmit(&hspi1, txBuffer, sizeof(txBuffer), 100);
	SPI1_CS_HIGH();
}


uint8_t mcp2515readRegister(uint8_t address){

	uint8_t txBuffer[3] = {0x03, address, 0xFF};
	uint8_t rxBuffer[3] = {0};

	HAL_StatusTypeDef status;

	SPI1_CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, sizeof(txBuffer), 100);
	SPI1_CS_HIGH();

	if(status != HAL_OK){

		Error_Handler();
	}
	return rxBuffer[2];
}

void mcp2515setTiming(void){
	// Example configuration for 500 kbps with 8 MHz oscillator
	// Calculate CNF1, CNF2, CNF3 using the MCP2515 datasheet
	mcp2515writeRegister(0x2A, 0x00); // CNF1: SJW=1, BRP=0
	mcp2515writeRegister(0x29, 0x90); // CNF2: BTLMODE=1, SAM=0, PHSEG1=3, PRSEG=1
	mcp2515writeRegister(0x28, 0x02); // CNF3: SOF=0, WAKFIL=0, PHSEG2=3
}

void mcp2515normalMode(void){

	mcp2515readRegister(0x0F);


}

void mcp2515init(void){

	uint8_t resetOP[1] = {0xC0};
	uint8_t status = 0;
	SPI1_CS_HIGH();
	HAL_Delay(10);

	mcp2515setTiming();

	//Transmission to reset device
	SPI1_CS_LOW();
	HAL_SPI_Transmit(&hspi1, resetOP, sizeof(resetOP), 100);
	SPI1_CS_HIGH();

	HAL_Delay(1);
	//Set to configuration Mode
	mcp2515writeRegister(0x0F,0x80);

	//Read back to confirm config mode
	uint8_t configResult = mcp2515readRegister(0x0E);

	if (configResult != 0x80){
		Error_Handler();

	}
	//Write into the receive 0 buffer to receive any message coming through
	mcp2515writeRegister(0x60, 0x60);

	uint8_t	buffer0ConfigResult = mcp2515readRegister(0x60);

	if (buffer0ConfigResult != 0x60){
		Error_Handler();
	}
//	if (configResult != 0x80){
//
//			Error_Handler();
//		}

	//Set register to accept any message

	HAL_Delay(10);
	//Write CAN status register into Normal Operation mode
	mcp2515writeRegister(0x0F,0x00);
	HAL_Delay(10);

	//Read CAN status register to confirm normal operation mode
	uint8_t resultAfter = mcp2515readRegister(0x0E);

	if (resultAfter != 0x00){
		Error_Handler();
	}

	HAL_Delay (10);
}

void mcp2515messageAvailable(void){

	/*Function to trigger the interrupt on the MCP2515 module
	 * when a message is available in the receive buffer 0 (RXB0)
	 */

	GPIO_PinState status;
	GPIO_PinState status1;


	status = HAL_GPIO_ReadPin(CAN_INT_GPIO_Port, CAN_INT_Pin);
	//Set the Interrupt flag from the RX0IF
	mcp2515writeRegister(0x2B, 0x01);



	//read the result from the Interrupt enable register at RX0IE
	uint8_t result = mcp2515readRegister(0x2B);
	result = mcp2515readRegister(0x0C);
	mcp2515writeRegister(0x2C, 0x01);

	if (result != 0x01){
		Error_Handler;
	}

	status1 = HAL_GPIO_ReadPin(CAN_INT_GPIO_Port, CAN_INT_Pin);

}


uint8_t mcp2515readMessage(bool random, uint8_t fixedData){

	//Use this function to take the message and transform it into a readable CAN message packet to be read

	//Use the recevice function and return a random number in place of of it. Clears register as well
	uint8_t readRXB0[1] = {0x90};
	uint8_t RXB0Buffer[14] = {0};
	uint16_t RXB0Data[1] = {0};
	const char sensorName[20] = "Temp Sensor";

	char outputBuffer[2];

	//Clear RXB0
	mcp2515writeRegister(0x2C, 0x00);

	SPI1_CS_LOW();
	HAL_SPI_Transmit(&hspi1, readRXB0, 1, 100);
	HAL_SPI_Receive(&hspi1, RXB0Buffer, 14, 100);
	SPI1_CS_HIGH();

	// Convert the received byte to a null-terminated string
	outputBuffer[0] = (char)RXB0Buffer[1];
	outputBuffer[1] = '\0'; // Null-terminator

	// Use the print function
	print(outputBuffer);

	MessageCAN canMessage = {
		.canID = 0x35,
		.data = RXB0Data[0],
		.sensorName = sensorName,
		.timeStamp = 0
	};


	//Returns data of the message, can be random or fixed for testing
	if (random == True){
		RXB0Data[0] = rand() % (255);
	}
	else{

		RXB0Data[0] = fixedData;
	}

	print(RXB0Data);
	//Add message to mail queue
	return RXB0Data;
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
