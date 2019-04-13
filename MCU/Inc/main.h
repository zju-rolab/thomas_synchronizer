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
#define IMU_SyncIn_Pin GPIO_PIN_5
#define IMU_SyncIn_GPIO_Port GPIOA
#define IMU_SyncIn_EXTI_IRQn EXTI9_5_IRQn
#define Cam_SyncOut1_Pin GPIO_PIN_0
#define Cam_SyncOut1_GPIO_Port GPIOB
#define Cam_SyncOut2_Pin GPIO_PIN_1
#define Cam_SyncOut2_GPIO_Port GPIOB
#define Laser_SyncOut_Pin GPIO_PIN_15
#define Laser_SyncOut_GPIO_Port GPIOB
#define Cam_SyncOut7_Pin GPIO_PIN_5
#define Cam_SyncOut7_GPIO_Port GPIOB
#define Cam_SyncOut6_Pin GPIO_PIN_6
#define Cam_SyncOut6_GPIO_Port GPIOB
#define Cam_SyncOut5_Pin GPIO_PIN_7
#define Cam_SyncOut5_GPIO_Port GPIOB
#define Cam_SyncOut4_Pin GPIO_PIN_8
#define Cam_SyncOut4_GPIO_Port GPIOB
#define Cam_SyncOut3_Pin GPIO_PIN_9
#define Cam_SyncOut3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
