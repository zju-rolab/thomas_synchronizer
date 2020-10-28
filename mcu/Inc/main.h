/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIDAR_BAUD 115200
#define IMU_BAUD 460800
#define CMD_BAUD 9600
#define CMD_USART_OUT_Pin GPIO_PIN_2
#define CMD_USART_OUT_GPIO_Port GPIOA
#define CMD_USART_IN_Pin GPIO_PIN_3
#define CMD_USART_IN_GPIO_Port GPIOA
#define IMU_SYNC_IN_Pin GPIO_PIN_5
#define IMU_SYNC_IN_GPIO_Port GPIOA
#define IMU_SYNC_IN_EXTI_IRQn EXTI9_5_IRQn
#define CAM_SYNC_OUT1_Pin GPIO_PIN_0
#define CAM_SYNC_OUT1_GPIO_Port GPIOB
#define CAM_SYNC_OUT2_Pin GPIO_PIN_1
#define CAM_SYNC_OUT2_GPIO_Port GPIOB
#define IMU_IN_Pin GPIO_PIN_11
#define IMU_IN_GPIO_Port GPIOB
#define LIDAR_SYNC_OUT_Pin GPIO_PIN_15
#define LIDAR_SYNC_OUT_GPIO_Port GPIOB
#define LIDAR_MSG_OUT_Pin GPIO_PIN_9
#define LIDAR_MSG_OUT_GPIO_Port GPIOA
#define CAM_SYNC_OUT7_Pin GPIO_PIN_5
#define CAM_SYNC_OUT7_GPIO_Port GPIOB
#define CAM_SYNC_OUT6_Pin GPIO_PIN_6
#define CAM_SYNC_OUT6_GPIO_Port GPIOB
#define CAM_SYNC_OUT5_Pin GPIO_PIN_7
#define CAM_SYNC_OUT5_GPIO_Port GPIOB
#define CAM_SYNC_OUT4_Pin GPIO_PIN_8
#define CAM_SYNC_OUT4_GPIO_Port GPIOB
#define CAM_SYNC_OUT3_Pin GPIO_PIN_9
#define CAM_SYNC_OUT3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
