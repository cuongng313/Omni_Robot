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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
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
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// UART variable
uint8_t rx_data;
char rx_index;
char rx_buffer[20];
char temp[20];
int check_var = 0;
//encoder variable
int pulse_counter1 = 0, pulse_counter2 = 0, pulse_counter3 = 0, pulse_counter4 = 0;

//velocity variable
double k = sqrt(2)/2;
float vx = 0, vy = 0, w_angle = 0;
float w_reference1_temp = 0;
float w_reference2_temp = 0;
float w_reference3_temp = 0;
float w_reference4_temp = 0;
float w_reference1 = 0, w_reference2 = 0, w_reference3 = 0, w_reference4 = 0;
float w1 = 0, w2 = 0, w3 = 0, w4 = 0;
float e1 = 0, e2 = 0, e3 = 0, e4 = 0;
float pre_e1 = 0, pre_e2 = 0, pre_e3 = 0, pre_e4 = 0;

//robot variable
float d_robot = 0.3;	//robot radius
float r_wheel = 0.05;

//PID parameters
float kP = 0.0, kI = 0.0, kD = 0.0;
float pPart1 = 0, pPart2 = 0, pPart3 = 0, pPart4 = 0;
float iPart1 = 0, iPart2 = 0, iPart3 = 0, iPart4 = 0;
float dPart1 = 0, dPart2 = 0, dPart3 = 0, dPart4 = 0;
float PID_signal1 = 0, PID_signal2 = 0, PID_signal3 = 0, PID_signal4 = 0;
float signal_pos1 = 0, signal_neg1 = 0;
float signal_pos2 = 0, signal_neg2 = 0;
float signal_pos3 = 0, signal_neg3 = 0;
float signal_pos4 = 0, signal_neg4 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Init_Motor1()
{
	////Enable built-in chip H-bridge
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);			//bridge1 3.3V
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);			//bridge1 GND
	////Enable H-bridge
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, 1);			//bridge1 EN_R
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, 1);			//bridge1 EN_L
	////Start PWM
	//PWM Motor 1
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);			//Motor1 +
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);			//Motor1 -
}

void Init_Motor2()
{
	////Enable built-in chip H-bridge
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 1);			//bridge1 3.3V
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);			//bridge1 GND
	////Enable H-bridge
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);			//bridge1 EN_R
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);			//bridge1 EN_L
	////Start PWM
	//PWM Motor 2
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);			//Motor2 +
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);			//Motor2 -
}

void Init_Motor3()
{
	////Enable built-in chip H-bridge
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);			//bridge1 3.3V
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);			//bridge1 GND
	////Enable H-bridge
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);			//bridge1 EN_R
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);			//bridge1 EN_L
	////Start PWM
	//PWM Motor 3
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);		//Motor3 +
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);		//Motor3 -
}

void Init_Motor4()
{
	////Enable built-in chip H-bridge
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);			//bridge1 3.3V
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, 0);			//bridge1 GND
	////Enable H-bridge
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);			//bridge1 EN_R
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, 1);			//bridge1 EN_L
	////Start PWM
	//PWM Motor 4
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);			//Motor4 +
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);			//Motor4 -
}

void check_w_ref()
{
	sprintf(temp, "vx_ref = %.2f\n", vx);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "vy_ref = %.2f\n", vy);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "w_ref = %.2f\n", w_angle);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);

	sprintf(temp, "w1_temp = %.2f, w1_ref = %.2f\n", w_reference1_temp ,w_reference1);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "w2_temp = %.2f, w2_ref = %.2f\n", w_reference2_temp ,w_reference2);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "w3_temp = %.2f, w3_ref = %.2f\n", w_reference3_temp ,w_reference3);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "w4_temp = %.2f, w4_ref = %.2f\n", w_reference4_temp ,w_reference4);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	HAL_Delay(2000);
}

void signal_feedback_PID_parameter()
{
	sprintf(temp, "P%.2fp\n", kP);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "I%.2fi\n", kI);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "D%.2fd\n", kD);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(2000);
}

void signal_feedback_w_wheel()
{
	sprintf(temp, "w1 = %.2f rpm r\n", w1);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "w2 = %.2f rpm r\n", w2);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "w3 = %.2f rpm r\n", w3);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "w4 = %.2f rpm r\n", w4);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(2000);
}

void signal_feedback_error()
{
	sprintf(temp, "e1 = %.2f rpm r\n", e1);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "e2 = %.2f rpm r\n", e2);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "e3 = %.2f rpm r\n", e3);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(100);
	sprintf(temp, "e4 = %.2f rpm r\n", e4);
	HAL_UART_Transmit(&huart3, temp, strlen(temp), 10);
	HAL_Delay(2000);
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  ////Start Timer Interrupt for sampling time!!!
  HAL_TIM_Base_Start_IT(&htim2);
  ////Start UART Receive interupt
  HAL_UART_Receive_IT(&huart3, &rx_data, 1);

  Init_Motor1();
  Init_Motor2();
  //Init_Motor3();
  //Init_Motor4();
 //__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 200);
  //__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 0);
  //__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2, 0);
  //__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3, 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(check_var == 0)
	  {
		  check_w_ref();
	  }
	  if(check_var == 1)
	  {
		  signal_feedback_w_wheel();
	  }
	  if(check_var == 2)
	  {
		  signal_feedback_error();
	  }
	  if(check_var == 3)
	  {
		  signal_feedback_PID_parameter();
	  }
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 16;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 16;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M01_EN_R_Pin|M01_EN_L_Pin|M02_EN_L_Pin|M02_EN_R_Pin 
                          |M02_3_3V_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M01_3_3V_GPIO_Port, M01_3_3V_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M01_GND_GPIO_Port, M01_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M02_GND_GPIO_Port, M02_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, M03_EN_R_Pin|M03_EN_L_Pin|M03_3_3V_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M03_GND_GPIO_Port, M03_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M04_GND_GPIO_Port, M04_GND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, M04_3_3V_Pin|M04_EN_L_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M04_EN_R_GPIO_Port, M04_EN_R_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M01_EN_R_Pin M01_EN_L_Pin M02_EN_L_Pin M02_EN_R_Pin 
                           M02_3_3V_Pin M02_GND_Pin */
  GPIO_InitStruct.Pin = M01_EN_R_Pin|M01_EN_L_Pin|M02_EN_L_Pin|M02_EN_R_Pin 
                          |M02_3_3V_Pin|M02_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : M01_3_3V_Pin M01_GND_Pin */
  GPIO_InitStruct.Pin = M01_3_3V_Pin|M01_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : EN02_A_Pin EN01_A_Pin */
  GPIO_InitStruct.Pin = EN02_A_Pin|EN01_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EN02_B_Pin EN01_B_Pin */
  GPIO_InitStruct.Pin = EN02_B_Pin|EN01_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : M03_EN_R_Pin M03_EN_L_Pin M03_3_3V_Pin M03_GND_Pin */
  GPIO_InitStruct.Pin = M03_EN_R_Pin|M03_EN_L_Pin|M03_3_3V_Pin|M03_GND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : EN03_A_Pin */
  GPIO_InitStruct.Pin = EN03_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EN03_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN03_B_Pin */
  GPIO_InitStruct.Pin = EN03_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN03_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN04_B_Pin */
  GPIO_InitStruct.Pin = EN04_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN04_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN04_A_Pin */
  GPIO_InitStruct.Pin = EN04_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EN04_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M04_GND_Pin M04_3_3V_Pin M04_EN_L_Pin */
  GPIO_InitStruct.Pin = M04_GND_Pin|M04_3_3V_Pin|M04_EN_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : M04_EN_R_Pin */
  GPIO_InitStruct.Pin = M04_EN_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M04_EN_R_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void cal_vel_ref(float vx, float vy, float w_angle)
{
	//calculate ref for each wheel
	w_reference1_temp = (1/r_wheel)*(vx*(-k) + vy*k + w_angle*d_robot);		//(rad/s)
	w_reference2_temp = (1/r_wheel)*(vx*(-k) + vy*(-k) + w_angle*d_robot);		//(rad/s)
	w_reference3_temp = (1/r_wheel)*(vx*k + vy*(-k) + w_angle*d_robot);		//(rad/s)
	w_reference4_temp = (1/r_wheel)*(vx*k + vy*k + w_angle*d_robot);		//(rad/s)

	//convert to RPM
	w_reference1 = (w_reference1_temp)*60/(2*M_PI);
	w_reference2 = (w_reference2_temp)*60/(2*M_PI);
	w_reference3 = (w_reference3_temp)*60/(2*M_PI);
	w_reference4 = (w_reference4_temp)*60/(2*M_PI);
}

void PID_controller_wheel1(float w_reference1)
{
	w1 = (float)(pulse_counter1)*1.2;
	pulse_counter1 = 0;
	if(w_reference1 == 0)
	{
		signal_pos1 = 0;
		signal_neg1 = 0;
	}
	else
	{
		e1 = w_reference1 - w1;
		pPart1 = kP*e1;
		iPart1 += kI*e1*0.1;
		dPart1 = kD*(e1 - pre_e1)*10;
		PID_signal1 += pPart1 + iPart1 + dPart1;
		if (PID_signal1 >= 0)
		{	//quay thuan
			signal_pos1 = PID_signal1;
			signal_neg1 = 0;
		}
		else
		{	//quay nguoc
			signal_pos1 = 0;
			signal_neg1 = -PID_signal1;
		}
		if(signal_pos1 > 1000) signal_pos1 = 999;
		if(signal_neg1 > 1000) signal_neg1 = 999;
		pre_e1 = e1;
	}
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, signal_pos1);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, signal_neg1);
	//__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, signal_neg1);
	//__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, signal_pos1);
}
void PID_controller_wheel2(float w_reference2)
{
	w2 = (float)(pulse_counter2)*1.2;
	pulse_counter2 = 0;
	if(w_reference2 == 0)
	{
		signal_pos2 = 0;
		signal_neg2 = 0;
	}
	else
	{
		e2 = w_reference2 - w2;
		pPart2 = kP*e2;
		iPart2 += kI*e2*0.1;
		dPart2 = kD*(e2 - pre_e2)*10;
		PID_signal2 += pPart2 + iPart2 + dPart2;
		if (PID_signal2 >= 0)
		{	//quay thuan
			signal_pos2 = PID_signal2;
			signal_neg2 = 0;
		}
		else
		{	//quay nguoc
			signal_pos2 = 0;
			signal_neg2 = -PID_signal2;
		}
		if(signal_pos2 > 1000) signal_pos2 = 999;
		if(signal_neg2 > 1000) signal_neg2 = 999;
		pre_e2 = e2;
	}
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, signal_pos2);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, signal_neg2);
}
void PID_controller_wheel3(float w_reference3)
{
	w3 = (float)(pulse_counter3)*1.2;
	pulse_counter3 = 0;
	if(w_reference3 == 0)
	{
		signal_pos3 = 0;
		signal_neg3 = 0;
	}
	else
	{
		e3 = w_reference3 - w3;
		pPart3 = kP*e3;
		iPart3 += kI*e3*0.1;
		dPart3 = kD*(e3 - pre_e3)*10;
		PID_signal3 += pPart3 + iPart3 + dPart3;
		if (PID_signal3 >= 0)
		{	//quay thuan
			signal_pos3 = PID_signal3;
			signal_neg3 = 0;
		}
		else
		{	//quay nguoc
			signal_pos3 = 0;
			signal_neg3 = -PID_signal3;
		}
		if(signal_pos3 > 1000) signal_pos3 = 999;
		if(signal_neg3 > 1000) signal_neg3 = 999;
		pre_e3 = e3;
	}
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, signal_pos3);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, signal_neg3);
}
void PID_controller_wheel4(float w_reference4)
{
	w4 = (float)(pulse_counter4)*1.2;
	pulse_counter4 = 0;
	if(w_reference4 == 0)
	{
		signal_pos4 = 0;
		signal_neg4 = 0;
	}
	else
	{
		e4 = w_reference4 - w4;
		pPart4 = kP*e4;
		iPart4 += kI*e4*0.1;
		dPart4 = kD*(e4 - pre_e4)*10;
		PID_signal4 += pPart4 + iPart4 + dPart4;
		if (PID_signal4 >= 0)
		{	//quay thuan
			signal_pos4 = PID_signal4;
			signal_neg4 = 0;
		}
		else
		{	//quay nguoc
			signal_pos4 = 0;
			signal_neg4 = -PID_signal4;
		}
		if(signal_pos4 > 1000) signal_pos4 = 999;
		if(signal_neg4 > 1000) signal_neg4 = 999;
		pre_e4 = e4;
	}
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, signal_pos4);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, signal_neg4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim2.Instance)
	{
		cal_vel_ref(vx, vy, w_angle);			//calculate ref for eachwheel
		PID_controller_wheel1(w_reference1);
		//PID_controller_wheel2(w_reference2);
		//PID_controller_wheel3(w_reference3);
		//PID_controller_wheel4(w_reference4);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Encoder moto1
	if(GPIO_Pin == GPIO_PIN_0)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == 0)
		{
			pulse_counter1++;
		}
		else
		{
			pulse_counter1--;
		}
	}
	//Encoder moto2
	if(GPIO_Pin == GPIO_PIN_14)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == 0)
		{
			pulse_counter2++;
		}
		else
		{
			pulse_counter2--;
		}
	}
	//Encoder moto3
	if(GPIO_Pin == GPIO_PIN_12)
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == 0)
		{
			pulse_counter3++;
		}
		else
		{
			pulse_counter3--;
		}
	}
	//Encoder moto3
	if(GPIO_Pin == GPIO_PIN_10)
	{
		if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9) == 0)
		{
			pulse_counter4++;
		}
		else
		{
			pulse_counter4--;
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart3.Instance)
	{
		if(rx_index == 0)
		{
			for(int i=0;i<=20;i++)
			{
				rx_buffer[i] = NULL;
			}
		}
		rx_buffer[rx_index] = rx_data;
		rx_index++;

		if(rx_data=='p')
		{
			kP=atof(rx_buffer);
			rx_index=0;
		}
		if(rx_data=='i')
		{
			kI=atof(rx_buffer);
			rx_index=0;
		}
		if(rx_data=='d')
		{
			kD=atof(rx_buffer);
			rx_index=0;
		}
		if(rx_data=='x')
		{
			vx=atof(rx_buffer);
			rx_index=0;
		}
		if(rx_data=='y')
		{
			vy=atof(rx_buffer);
			rx_index=0;
		}
		if(rx_data=='w')
		{
			w_angle=atof(rx_buffer);
			rx_index=0;
		}
		if(rx_data=='c')
		{
			check_var=atof(rx_buffer);
			rx_index=0;
		}
		HAL_UART_Receive_IT(&huart3, &rx_data,1);
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
