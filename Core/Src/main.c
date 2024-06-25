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
#include <stdio.h>
#include "LCD1602.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

osThreadId ReadRightTaskHandle;
osThreadId ReadSensorDataHandle;
osThreadId choosePathHandle;
osMutexId myMutex01Handle;
/* USER CODE BEGIN PV */
#define TRIGCENTER_PIN GPIO_PIN_9
#define TRIGCENTER_PORT GPIOA
#define ECHOCENTER_PIN GPIO_PIN_8
#define ECHOCENTER_PORT GPIOA
uint32_t centerpMillis;
uint32_t centerValue1 = 0;
uint32_t centerValue2 = 0;
uint16_t centerDistance  = 0;  // cm

#define TRIGLEFT_PIN GPIO_PIN_11
#define TRIGLEFT_PORT GPIOA
#define ECHOLEFT_PIN GPIO_PIN_10
#define ECHOLEFT_PORT GPIOA
uint32_t leftpMillis;
uint32_t leftValue1 = 0;
uint32_t leftValue2 = 0;
uint16_t leftDistance  = 0;  // cm


#define TRIGRIGHT_PIN GPIO_PIN_15
#define TRIGRIGHT_PORT GPIOB
#define ECHORIGHT_PIN GPIO_PIN_14
#define ECHORIGHT_PORT GPIOB
uint32_t rightpMillis;
uint32_t rightValue1 = 0;
uint32_t rightValue2 = 0;
uint16_t rightDistance  = 0;  // cm

uint16_t desiredDistance=5;
uint16_t error;

int preverror=0;
#define kp 500
#define ki 0



#define kd 0.3
int pwmC=0;
int pwmR=0;
int pwmL=0;
int sum=0;
int SampleTime=500; // 1khz
int propotional=0;
int integral=0;
int derivative =0;
char buf[10];
uint16_t x;

int PID_Apply(int error){
	int output=0;

	sum+=error;
	 propotional= kp*error;
	 integral=(ki*sum)/SampleTime;
	 derivative =kd*(error-preverror)*SampleTime;

	output=propotional+integral+derivative;
	preverror=error;
	if(output>12000)
		output=12000;
	else if(output<0)
	{
		output=0;
	}
	else if(output<=4500 && derivative>0)
	{
		output=4500;
	}
	return output;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void ReadSensor(void const * argument);
void path(void const * argument);

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_GPIO_WritePin(TRIGCENTER_PORT, TRIGCENTER_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END 2 */
  lcd_init ();
    HAL_Delay(100);
    lcd_clear();
    lcd_put_cur(0, 0);
    lcd_send_string("HELLO ");
    HAL_Delay(100);
    lcd_clear();
  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* definition and creation of ReadSensorData */
  osThreadDef(ReadSensorData, ReadSensor, osPriorityNormal, 0, 128);
  ReadSensorDataHandle = osThreadCreate(osThread(ReadSensorData), NULL);

  /* definition and creation of choosePath */
  osThreadDef(choosePath, path, osPriorityNormal, 0, 128);
  choosePathHandle = osThreadCreate(osThread(choosePath), NULL);

  osKernelStart();

  while (1){}  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim2.Init.Period = 12000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA6 PA7 PA9 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ta */
/**
  * @brief  Function implementing the ReadRightTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ta */

void ReadSensor(void const * argument)
{
  /* USER CODE BEGIN ReadSensor */
  /* Infinite loop */
  for(;;)
  {
	 // xSemaphoreTake(myMutex01Handle,0);
	  vTaskSuspend(NULL);
	  HAL_GPIO_WritePin(TRIGCENTER_PORT, TRIGCENTER_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	      __HAL_TIM_SET_COUNTER(&htim1, 0);
	      while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
	      HAL_GPIO_WritePin(TRIGCENTER_PORT, TRIGCENTER_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	      centerpMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	      // wait for the echo pin to go high
	      while (!(HAL_GPIO_ReadPin (ECHOCENTER_PORT, ECHOCENTER_PIN)) && centerpMillis + 10 >  HAL_GetTick());
	      centerValue1 = __HAL_TIM_GET_COUNTER (&htim1);

	      centerpMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	      // wait for the echo pin to go low
	      while ((HAL_GPIO_ReadPin (ECHOCENTER_PORT, ECHOCENTER_PIN)) && centerpMillis + 50 > HAL_GetTick());
	      centerValue2 = __HAL_TIM_GET_COUNTER (&htim1);

	      centerDistance = (centerValue2-centerValue1)* 0.034/2;



	      //left ultrasonic
	      HAL_GPIO_WritePin(TRIGLEFT_PORT, TRIGLEFT_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	      __HAL_TIM_SET_COUNTER(&htim3, 0);
	      while (__HAL_TIM_GET_COUNTER (&htim3) < 10);  // wait for 10 us
	      HAL_GPIO_WritePin(TRIGLEFT_PORT, TRIGLEFT_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	      leftpMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	      // wait for the echo pin to go high
	      while (!(HAL_GPIO_ReadPin (ECHOLEFT_PORT, ECHOLEFT_PIN)) && leftpMillis + 10 >  HAL_GetTick());
	      leftValue1 = __HAL_TIM_GET_COUNTER (&htim3);

	      leftpMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	      // wait for the echo pin to go low
	      while ((HAL_GPIO_ReadPin (ECHOLEFT_PORT, ECHOLEFT_PIN)) && leftpMillis + 50 > HAL_GetTick());
	      leftValue2 = __HAL_TIM_GET_COUNTER (&htim3);

	      leftDistance = (leftValue2-leftValue1)* 0.034/2;


	      //right ultrasonic
	      HAL_GPIO_WritePin(TRIGRIGHT_PORT, TRIGRIGHT_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	      __HAL_TIM_SET_COUNTER(&htim4, 0);
	      while (__HAL_TIM_GET_COUNTER (&htim4) < 10);  // wait for 10 us
	      HAL_GPIO_WritePin(TRIGRIGHT_PORT, TRIGRIGHT_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	      rightpMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	      // wait for the echo pin to go high
	      while (!(HAL_GPIO_ReadPin (ECHORIGHT_PORT, ECHORIGHT_PIN)) && rightpMillis + 10 >  HAL_GetTick());
	      rightValue1 = __HAL_TIM_GET_COUNTER (&htim4);

	      rightpMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	      // wait for the echo pin to go low
	      while ((HAL_GPIO_ReadPin (ECHORIGHT_PORT, ECHORIGHT_PIN)) && rightpMillis + 50 > HAL_GetTick());
	      rightValue2 = __HAL_TIM_GET_COUNTER (&htim4);

	      rightDistance = (rightValue2-rightValue1)* 0.034/2;

	      //xSemaphoreGive(myMutex01Handle);
	      vTaskResume(NULL);
    //osDelay(1);
  }
  /* USER CODE END ReadSensor */
}

/* USER CODE BEGIN Header_path */
/**
* @brief Function implementing the choosePath thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_path */
void path(void const * argument)
{
  /* USER CODE BEGIN path */
  /* Infinite loop */
  for(;;)
  {
	  vTaskSuspend(NULL);
	  if(leftDistance>200 && rightDistance>200 && centerDistance>200){
	     	TIM2->CCR1=0;
	     	TIM2->CCR2=0;
	     	 lcd_send_string("No object detected! ");
	          HAL_Delay(100);
	          lcd_clear();
	     }
	     else if(centerDistance<leftDistance && centerDistance<rightDistance && centerDistance<300 && centerDistance>desiredDistance){
	     	 error=(centerDistance-desiredDistance);
	     	pwmC=PID_Apply(error);
	     	// pwmL=PID_Apply(error);
	     	if(error<=0)
	     	{
	     		pwmC=0;
	     		TIM2->CCR1=pwmC;
	     		TIM2->CCR2=pwmC;
	     		lcd_put_cur(0, 0);
	     		lcd_send_string("desired distance reached");


	     	}
	     	else
	     	{
	     	TIM2->CCR1=pwmC;
	     	TIM2->CCR2=pwmC;
	     	lcd_put_cur(0, 0);
	 		lcd_send_string("Move Forward ");
	 		lcd_put_cur(1, 0);
	 		sprintf(buf,"%d",error); // @suppress("Float formatting support"
	 		lcd_send_string(buf);
	 		lcd_send_string("  ");
	 		sprintf(buf,"%d",pwmC); // @suppress("Float formatting support"
	 		lcd_send_string(buf);
	   	    lcd_send_string("  ");
	 	 	sprintf(buf,"%d",centerDistance); // @suppress("Float formatting support"
	 	  lcd_send_string(buf);
	 		 HAL_Delay(100);
	 		 lcd_clear();
	     	}
	     }

	     else if(leftDistance>rightDistance && centerDistance>rightDistance &&rightDistance<300 && rightDistance>desiredDistance){
	      	error=(rightDistance-desiredDistance);
	      	if(error<=0)
	      	{
	      		pwmR=0;
	      		TIM2->CCR1=pwmR;
	      		TIM2->CCR2=pwmR;
	      		lcd_put_cur(0, 0);
	      		lcd_send_string("desired distance reached");


	      	}
	      	else
	      	{
	      	pwmR=PID_Apply(error);
	      	//pwmR=error*1000;
	      	TIM2->CCR1=pwmR;
	      	TIM2->CCR2=0;
	      	lcd_put_cur(0, 0);
	 		lcd_send_string("Move Right ");
	 		lcd_put_cur(1, 0);
	 		sprintf(buf,"%d",error); // @suppress("Float formatting support"
	 		lcd_send_string(buf);
	 		lcd_send_string("  ");
	 		sprintf(buf,"%d",pwmR); // @suppress("Float formatting support"
	 		lcd_send_string(buf);
	   	    lcd_send_string("  ");
	 	 	sprintf(buf,"%d",rightDistance); // @suppress("Float formatting support"
	 	  lcd_send_string(buf);
	 		 HAL_Delay(100);
	 		 lcd_clear();
	      	}

	     }

	     else if(leftDistance<rightDistance &&leftDistance<centerDistance && leftDistance<300 && leftDistance>desiredDistance){
	      	error=(leftDistance-desiredDistance);
	     	pwmL=PID_Apply(error);
	     	if(error<=0)
	     	{
	 		pwmL=0;
	 		TIM2->CCR1=pwmL;
	 		TIM2->CCR2=pwmL;
	 		lcd_put_cur(0, 0);
	 		lcd_send_string("desired distance reached");

	 		}
	 		else
	 		{
	 		TIM2->CCR2=pwmL;
	 		TIM2->CCR1=0;
	 		lcd_put_cur(0, 0);
	 		lcd_send_string("Move Left ");
	 		lcd_put_cur(1, 0);
	 		sprintf(buf,"%d",error); // @suppress("Float formatting support"
	 		lcd_send_string(buf);
	 		lcd_send_string("  ");
	 		sprintf(buf,"%d",pwmL); // @suppress("Float formatting support"
	 		lcd_send_string(buf);
	   	    lcd_send_string("  ");
	 	 	sprintf(buf,"%d",leftDistance); // @suppress("Float formatting support"
	 	  lcd_send_string(buf);
	 		 HAL_Delay(100);
	 		 lcd_clear();
	 		}

	     }

	    vTaskResume(NULL);
    //osDelay(1);
  }
  /* USER CODE END path */


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
