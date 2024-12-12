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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "event_groups.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI1_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define SPI1_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define EVENT_BIT_0 (0 << 0)
#define EVENT_BIT_1 (1 << 0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
/* USER CODE BEGIN PV */

EventGroupHandle_t messageToRead;
static StaticTask_t xTimerTaskTCB;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/* USER CODE BEGIN PFP */

//No idea what this doe, but needed for xEventGroupSetBitsFromISR
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;              // Provide TCB memory
    *ppxTimerTaskStackBuffer = xTimerStack;               // Provide stack memory
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH; // Provide stack size
}

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
  print("Program Started");

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  messageToRead = xEventGroupCreate();
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityAboveNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //mcp2515messageAvailable();
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

  /*Configure GPIO pin : Btn_Int_Pin */
  GPIO_InitStruct.Pin = Btn_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Btn_Int_GPIO_Port, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print(const char* buffer) {
    // Calculate the string length
    size_t length = strlen(buffer);

    // Add space for the new line and carriage return
    char tempBuffer[length + 3]; // Original string + '\r' + '\n' + null terminator

    // Copy the original string into the temporary buffer
    strcpy(tempBuffer, buffer);

    // Append the new line and carriage return
    tempBuffer[length] = '\r';     // Carriage return
    tempBuffer[length + 1] = '\n'; // New line
    tempBuffer[length + 2] = '\0'; // Null terminator

    // Transmit the modified string over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)tempBuffer, strlen(tempBuffer), 100);
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


void mcp2515readMessage(bool random, uint8_t fixedData){

	//Use this function to take the message and transform it into a readable CAN message packet to be read

	//Use the recevice function and return a random number in place of of it. Clears register as well

	GPIO_PinState status = HAL_GPIO_ReadPin(CAN_INT_GPIO_Port, CAN_INT_Pin);

	if (status == GPIO_PIN_RESET){
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

		//Returns data of the message, can be random or fixed for testing
		if (random == true){
			RXB0Data[0] = rand() % (255);
		}
		else{

			RXB0Data[0] = fixedData;
		}


		MessageCAN canMessage = {
			.canID = 0x35,
			.data = RXB0Data[0],
			.sensorName = sensorName,
			.timeStamp = 0
		};

		char buffer[10];
		sprintf(buffer, "%d", canMessage.data);
		print("message received, data:");
		print(buffer);
	}
	else{
		print("no message in buffer");
	}


	//Add message to mail queue
	//return RXB0Data;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	print("Int Activated");
	UNUSED(GPIO_Pin);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	mcp2515messageAvailable();
	xEventGroupSetBitsFromISR(messageToRead, EVENT_BIT_1, &xHigherPriorityTaskWoken);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    print("Idle Task");
    osDelay(500);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  print("Task Entered");
	  EventBits_t uxBits = xEventGroupWaitBits(messageToRead, EVENT_BIT_1, pdTRUE, pdTRUE, portMAX_DELAY);
	  print("Task Started");
      mcp2515readMessage(false, 100);
      osDelay(500);

  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
