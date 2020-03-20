/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M01_EN_R_Pin GPIO_PIN_2
#define M01_EN_R_GPIO_Port GPIOE
#define M01_EN_L_Pin GPIO_PIN_3
#define M01_EN_L_GPIO_Port GPIOE
#define M01_PWM__Pin GPIO_PIN_5
#define M01_PWM__GPIO_Port GPIOE
#define M01_PWM_E6_Pin GPIO_PIN_6
#define M01_PWM_E6_GPIO_Port GPIOE
#define M01_3_3V_Pin GPIO_PIN_0
#define M01_3_3V_GPIO_Port GPIOF
#define M01_GND_Pin GPIO_PIN_1
#define M01_GND_GPIO_Port GPIOF
#define M02_PWM__Pin GPIO_PIN_0
#define M02_PWM__GPIO_Port GPIOB
#define M02_PWM_B1_Pin GPIO_PIN_1
#define M02_PWM_B1_GPIO_Port GPIOB
#define M02_EN_L_Pin GPIO_PIN_10
#define M02_EN_L_GPIO_Port GPIOE
#define M02_EN_R_Pin GPIO_PIN_11
#define M02_EN_R_GPIO_Port GPIOE
#define M02_3_3V_Pin GPIO_PIN_12
#define M02_3_3V_GPIO_Port GPIOE
#define M02_GND_Pin GPIO_PIN_13
#define M02_GND_GPIO_Port GPIOE
#define EN02_A_Pin GPIO_PIN_14
#define EN02_A_GPIO_Port GPIOE
#define EN02_A_EXTI_IRQn EXTI15_10_IRQn
#define EN02_B_Pin GPIO_PIN_15
#define EN02_B_GPIO_Port GPIOE
#define M03_PWM__Pin GPIO_PIN_14
#define M03_PWM__GPIO_Port GPIOB
#define M03_PWM_B15_Pin GPIO_PIN_15
#define M03_PWM_B15_GPIO_Port GPIOB
#define M03_EN_R_Pin GPIO_PIN_8
#define M03_EN_R_GPIO_Port GPIOD
#define M03_EN_L_Pin GPIO_PIN_9
#define M03_EN_L_GPIO_Port GPIOD
#define M03_3_3V_Pin GPIO_PIN_10
#define M03_3_3V_GPIO_Port GPIOD
#define M03_GND_Pin GPIO_PIN_11
#define M03_GND_GPIO_Port GPIOD
#define EN03_A_Pin GPIO_PIN_12
#define EN03_A_GPIO_Port GPIOD
#define EN03_A_EXTI_IRQn EXTI15_10_IRQn
#define EN03_B_Pin GPIO_PIN_13
#define EN03_B_GPIO_Port GPIOD
#define EN04_B_Pin GPIO_PIN_9
#define EN04_B_GPIO_Port GPIOG
#define EN04_A_Pin GPIO_PIN_10
#define EN04_A_GPIO_Port GPIOG
#define EN04_A_EXTI_IRQn EXTI15_10_IRQn
#define M04_GND_Pin GPIO_PIN_13
#define M04_GND_GPIO_Port GPIOG
#define M04_3_3V_Pin GPIO_PIN_14
#define M04_3_3V_GPIO_Port GPIOG
#define M04_EN_L_Pin GPIO_PIN_15
#define M04_EN_L_GPIO_Port GPIOG
#define M04_EN_R_Pin GPIO_PIN_3
#define M04_EN_R_GPIO_Port GPIOB
#define M04_PWM__Pin GPIO_PIN_4
#define M04_PWM__GPIO_Port GPIOB
#define M04_PWM_B5_Pin GPIO_PIN_5
#define M04_PWM_B5_GPIO_Port GPIOB
#define EN01_A_Pin GPIO_PIN_0
#define EN01_A_GPIO_Port GPIOE
#define EN01_A_EXTI_IRQn EXTI0_IRQn
#define EN01_B_Pin GPIO_PIN_1
#define EN01_B_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
