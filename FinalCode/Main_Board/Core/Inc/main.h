/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define Ultrasound_Echo_Pin GPIO_PIN_5
#define Ultrasound_Echo_GPIO_Port GPIOE
#define Ultrasound_Trigger_Pin GPIO_PIN_6
#define Ultrasound_Trigger_GPIO_Port GPIOE
#define camer_pin_Pin GPIO_PIN_3
#define camer_pin_GPIO_Port GPIOC
#define camer_pin_EXTI_IRQn EXTI3_IRQn
#define camera_pin_Pin GPIO_PIN_2
#define camera_pin_GPIO_Port GPIOA
#define camera_pinA3_Pin GPIO_PIN_3
#define camera_pinA3_GPIO_Port GPIOA
#define Motor_PWM_Pin GPIO_PIN_6
#define Motor_PWM_GPIO_Port GPIOA
#define VR_controller_Pin GPIO_PIN_7
#define VR_controller_GPIO_Port GPIOA
#define camera_pinC4_Pin GPIO_PIN_4
#define camera_pinC4_GPIO_Port GPIOC
#define camera_pinC5_Pin GPIO_PIN_5
#define camera_pinC5_GPIO_Port GPIOC
#define camera_pinB10_Pin GPIO_PIN_10
#define camera_pinB10_GPIO_Port GPIOB
#define camera_pinB11_Pin GPIO_PIN_11
#define camera_pinB11_GPIO_Port GPIOB
#define camera_pinB12_Pin GPIO_PIN_12
#define camera_pinB12_GPIO_Port GPIOB
#define camera_pinB13_Pin GPIO_PIN_13
#define camera_pinB13_GPIO_Port GPIOB
#define camera_pinB14_Pin GPIO_PIN_14
#define camera_pinB14_GPIO_Port GPIOB
#define camera_pinB15_Pin GPIO_PIN_15
#define camera_pinB15_GPIO_Port GPIOB
#define camera_pi_Pin GPIO_PIN_12
#define camera_pi_GPIO_Port GPIOD
#define camera_pinC6_Pin GPIO_PIN_6
#define camera_pinC6_GPIO_Port GPIOC
#define camera_pinC7_Pin GPIO_PIN_7
#define camera_pinC7_GPIO_Port GPIOC
#define PC_Serial_Pin GPIO_PIN_9
#define PC_Serial_GPIO_Port GPIOA
#define PC_SerialA10_Pin GPIO_PIN_10
#define PC_SerialA10_GPIO_Port GPIOA
#define LoRa_RX_Pin GPIO_PIN_10
#define LoRa_RX_GPIO_Port GPIOC
#define LoRa_TX_Pin GPIO_PIN_11
#define LoRa_TX_GPIO_Port GPIOC
#define Weight_DT_Pin GPIO_PIN_12
#define Weight_DT_GPIO_Port GPIOC
#define Weight_CLK_Pin GPIO_PIN_2
#define Weight_CLK_GPIO_Port GPIOD
#define camera_pinD3_Pin GPIO_PIN_3
#define camera_pinD3_GPIO_Port GPIOD
#define camera_pinB8_Pin GPIO_PIN_8
#define camera_pinB8_GPIO_Port GPIOB
#define camera_pinB9_Pin GPIO_PIN_9
#define camera_pinB9_GPIO_Port GPIOB
#define camera_pinE1_Pin GPIO_PIN_1
#define camera_pinE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
