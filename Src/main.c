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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "userdefine.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osThreadId myLEDHandle;
osThreadId mySg90Handle;
osThreadId myIrLedHandle;
osThreadId myWaiteTimerHandle;
/* USER CODE BEGIN PV */
void smForward(uint16_t times);
void smRetreat(uint16_t times);
uint8_t checkIrSens(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void StartmyLED(void const * argument);
void StartSg90(void const * argument);
void StartIrLed(void const * argument);
void StartWaitTimer(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void smForward(uint16_t times) {
	  uint16_t i;
	  for(i=0;i<times;i++){
		HAL_GPIO_WritePin(SM_IN1,  GPIO_PIN_SET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN4,  GPIO_PIN_RESET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN2,  GPIO_PIN_SET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN1,  GPIO_PIN_RESET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN3,  GPIO_PIN_SET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN2,  GPIO_PIN_RESET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN4,  GPIO_PIN_SET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN3,  GPIO_PIN_RESET);
		osDelay(SM_STEP);
	  }
	  HAL_GPIO_WritePin(SM_IN4,  GPIO_PIN_RESET);

}
void smRetreat(uint16_t times) {
	  uint16_t i;
	  for(i=0;i<times;i++){
		HAL_GPIO_WritePin(SM_IN4,  GPIO_PIN_SET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN1,  GPIO_PIN_RESET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN3,  GPIO_PIN_SET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN4,  GPIO_PIN_RESET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN2,  GPIO_PIN_SET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN3,  GPIO_PIN_RESET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN1,  GPIO_PIN_SET);
		osDelay(SM_STEP);
		HAL_GPIO_WritePin(SM_IN2,  GPIO_PIN_RESET);
		osDelay(SM_STEP);
	  }
	  HAL_GPIO_WritePin(SM_IN1,  GPIO_PIN_RESET);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myLED */
  osThreadDef(myLED, StartmyLED, osPriorityIdle, 0, 128);
  myLEDHandle = osThreadCreate(osThread(myLED), NULL);

  /* definition and creation of mySg90 */
  osThreadDef(mySg90, StartSg90, osPriorityNormal, 0, 128);
  mySg90Handle = osThreadCreate(osThread(mySg90), NULL);

  /* definition and creation of myIrLed */
  osThreadDef(myIrLed, StartIrLed, osPriorityBelowNormal, 0, 128);
  myIrLedHandle = osThreadCreate(osThread(myIrLed), NULL);

  /* definition and creation of myWaiteTimer */
  osThreadDef(myWaiteTimer, StartWaitTimer, osPriorityLow, 0, 128);
  myWaiteTimerHandle = osThreadCreate(osThread(myWaiteTimer), NULL);

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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 104;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_R_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_R_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_SENS_Pin */
  GPIO_InitStruct.Pin = IR_SENS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_SENS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


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
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartmyLED */
/**
* @brief Function implementing the myLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmyLED */
void StartmyLED(void const * argument)
{
  /* USER CODE BEGIN StartmyLED */
  /* Infinite loop */
  for(;;)
  {
	//HAL_GPIO_TogglePin(LED_G);

	  osSignalWait(0x01,osWaitForever);
	  HAL_GPIO_WritePin(LED_G,GPIO_PIN_SET);
	  osDelay(80);
	  HAL_GPIO_WritePin(LED_G,GPIO_PIN_RESET);
  }
  /* USER CODE END StartmyLED */
}

/* USER CODE BEGIN Header_StartSg90 */
/**
* @brief Function implementing the mySg90 thread.
* @param argument: Not used
* @retval None
*/
void armSwing(void){
	  TIM2->CCR1 = RADDOWN2;
	  osDelay(500);
	  TIM2->CCR1 = RADUP2;
	  osDelay(200);
}


/* USER CODE END Header_StartSg90 */
void StartSg90(void const * argument)
{
  /* USER CODE BEGIN StartSg90 */
	uint8_t iteration;
	uint8_t i;
	  TIM2->CCMR1|=0x808;	//プリロードON
	  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	  TIM2->CCR1 = RADUP2;
	  osDelay(100);
	  HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
	  osEvent event = osSignalWait(0x0fU,osWaitForever);
	  iteration = (uint8_t)event.value.signals;

	  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	  osDelay(100);
	  for(i=0;i<(iteration*2);i++){
		  armSwing();
		  if(checkIrSens()!=IR_SENS){
			  break;
		  }
	  }
	  HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	  osDelay(500);
	  osSignalSet(myIrLedHandle,0x02U);


  }
  /* USER CODE END StartSg90 */
}

/* USER CODE BEGIN Header_StartIrLed */
/**
* @brief Function implementing the myIrLed thread.
* @param argument: Not used
* @retval None
*/
uint8_t checkIrSens(void){
	//赤外線パルスを打って反応を見る
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    osDelay(IR_DUTY);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	TIM3->CCMR1|=0xD808;	//LEDを落とす

	return (uint8_t)HAL_GPIO_ReadPin(IR_INPUT);
}
/* USER CODE END Header_StartIrLed */
void StartIrLed(void const * argument)
{
  /* USER CODE BEGIN StartIrLed */
	uint8_t irCount = 0;
	TIM3->CCMR1|=0x8808;	//プリロードON
  /* Infinite loop */
  for(;;)
  {
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	    osDelay(IR_DUTY);
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
		TIM3->CCMR1|=0xD808;	//LEDを落とす

		if(checkIrSens()==IR_SENS){
			irCount++;
		}else{
			irCount = 0;
		}

		if(irCount==4){
			HAL_GPIO_WritePin(LED_G,GPIO_PIN_SET);
			osSignalSet(myWaiteTimerHandle,0x01U);
			osSignalWait(0x02U,osWaitForever);

		}else if(irCount >4){
			//一旦センスが消えるまで待つ
			//センサ故障に対する対処
			HAL_GPIO_TogglePin(LED_G);
			irCount = 9;
		}else{
			HAL_GPIO_WritePin(LED_G,GPIO_PIN_RESET);
		}


	    osDelay(IR_CYCLE-IR_DUTY);

  }
  /* USER CODE END StartIrLed */
}

/* USER CODE BEGIN Header_StartWaitTimer */
/**
* @brief Function implementing the myWaiteTimer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWaitTimer */
void StartWaitTimer(void const * argument)
{
  /* USER CODE BEGIN StartWaitTimer */
	uint8_t countTime=1;
  /* Infinite loop */
  for(;;)
  {
	  osEvent event = osSignalWait(0x01U,1800U * ONE_SEC);
	  if(event.status == osEventTimeout){
		  if(countTime<4){
			  countTime++;
			  osSignalSet(myLEDHandle,0x1);
		  }
	  }else if(event.status == osEventSignal){
		  osSignalSet(mySg90Handle,countTime);
		  countTime = 1;
	  }
    osDelay(1);
  }
  /* USER CODE END StartWaitTimer */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM15 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM15) {
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
