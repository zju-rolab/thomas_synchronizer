/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include "stm32f1xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IMU_UART huart3
#define LASER_UART huart1
#define uputs(buf, len) HAL_UART_Transmit(&LASER_UART, (uint8_t *)buf, len, 0xFFFF)
#define uputs2(buf) uputs(buf, strlen(buf))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t counter = 0;
int send_gps = 0;
int gps_pulse_sent = 0;
uint8_t imu_buf_single[1];
uint16_t Cam_SyncOut_Pin[7] = {
    Cam_SyncOut1_Pin, Cam_SyncOut2_Pin, Cam_SyncOut3_Pin, Cam_SyncOut4_Pin,
    Cam_SyncOut5_Pin, Cam_SyncOut6_Pin, Cam_SyncOut7_Pin};
GPIO_TypeDef* Cam_SyncOut_GPIO_Port[7] = {
    Cam_SyncOut1_GPIO_Port, Cam_SyncOut2_GPIO_Port, Cam_SyncOut3_GPIO_Port, Cam_SyncOut4_GPIO_Port,
    Cam_SyncOut5_GPIO_Port, Cam_SyncOut6_GPIO_Port, Cam_SyncOut7_GPIO_Port};
const int CAM_CNT = 7;
const int CAM_FREQ_DIV[7] = {25, 25, 25, 25, 25, 25, 25};
const int LASER_DIV = 400;
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&IMU_UART, (uint8_t *)imu_buf_single, 1);
  char content[] = "$GPRMC,000000,A,hh.mm,N,ss.00,W,3.3,4.4,010117,004.2,W*";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HAL_GPIO_WritePin(Laser_SyncOut_GPIO_Port, Laser_SyncOut_Pin, GPIO_PIN_SET);
  printf("tmp\n");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (send_gps)
    {
      send_gps = 0;

      int hh, mm, ss;
      ss = counter / 400;
      mm = ss / 60;
      ss = ss % 60;
      hh = mm / 60;
      mm = mm % 60;

      content[7] = content[16] = '0' + hh / 10;
      content[8] = content[17] = '0' + hh % 10;
      content[9] = content[19] = '0' + mm / 10;
      content[10] = content[20] = '0' + mm % 10;
      content[11] = content[24] = '0' + ss / 10;
      content[12] = content[25] = '0' + ss % 10;

      char cs;
      cs = 0x00;
      for (int i = 1; i < strlen(content) - 1; ++i)
      {
        cs ^= content[i];
      }

      char cs_str[2];
      itoa(cs, cs_str, 16);

      uputs(content, strlen(content));
      uputs(cs_str, 2);
      uputs("\r\n", 2);
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Cam_SyncOut1_Pin|Cam_SyncOut2_Pin|Laser_SyncOut_Pin|Cam_SyncOut7_Pin 
                          |Cam_SyncOut6_Pin|Cam_SyncOut5_Pin|Cam_SyncOut4_Pin|Cam_SyncOut3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IMU_SyncIn_Pin */
  GPIO_InitStruct.Pin = IMU_SyncIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_SyncIn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Cam_SyncOut1_Pin Cam_SyncOut2_Pin Laser_SyncOut_Pin Cam_SyncOut7_Pin 
                           Cam_SyncOut6_Pin Cam_SyncOut5_Pin Cam_SyncOut4_Pin Cam_SyncOut3_Pin */
  GPIO_InitStruct.Pin = Cam_SyncOut1_Pin|Cam_SyncOut2_Pin|Laser_SyncOut_Pin|Cam_SyncOut7_Pin 
                          |Cam_SyncOut6_Pin|Cam_SyncOut5_Pin|Cam_SyncOut4_Pin|Cam_SyncOut3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t buf[300];
uint16_t buf_p = 0;
uint16_t data_len = 0;
uint8_t data = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  data = imu_buf_single[0];

  if (buf_p >= 300)
  {
    buf_p = 0;
    return;
  }
  else
  {
    buf[buf_p++] = data;
  }

  if (data == 0xfa)
  {
    // HAL_GPIO_TogglePin(Laser_SyncOut_GPIO_Port, Laser_SyncOut_Pin);
  }

  if (buf_p == 1)
  {
    if (data != 0xfa)
    {
      buf_p = 0;
    }
  }
  else if (buf_p == 2)
  {
    if (data != 0xff)
    {
      buf_p = 0;
    }
  }
  else if (buf_p == 4)
  {
    data_len = data;
  }
  else if (buf_p == (5 + data_len))
  {
    // Check it
    uint16_t sum = 0;
    for (int i = 1; i < buf_p; ++i)
      sum += buf[i];
    if ((sum & 0x00ff) == 0)
    {
      // checksum pass
      if (buf[2] == 0x36)
      {
        if (data_len >= 5)
        {
          if (buf[4] == 0x10 && buf[5] == 0x20 && buf[6] == 2)
          {
            static uint32_t short_counter = 0;
            static uint32_t loop = 0;
            uint32_t new_short_counter = buf[7] << 8 | buf[8];
            if (new_short_counter < short_counter)
              loop++;
            short_counter = new_short_counter;

            counter = loop << 16 | short_counter;

            if (gps_pulse_sent)
            {
              gps_pulse_sent = 0;
              send_gps = 1;
            }
          }
          else
          {
            uputs2("No TS?\r\n");
          }
        }
        else
        {
          uputs2("Too short MTData2\r\n");
        }
      }
    }
    else
    {
      uputs2("Checksum failed\r\n");
    }
    buf_p = 0;
  }
  else if (buf_p > (5 + data_len))
  {
    buf_p = 0;
    uputs2("WTF?\r\n");
  }
  HAL_UART_Receive_IT(&IMU_UART, (uint8_t *)imu_buf_single, 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_5)
  {
    /* For camera */
    for (int i = 0; i < CAM_CNT; ++i)
    {
      if (((counter + 1) % CAM_FREQ_DIV[i]) == 0)
      {
        HAL_GPIO_WritePin(Cam_SyncOut_GPIO_Port[i], Cam_SyncOut_Pin[i],
            GPIO_PIN_SET);
      }
      else if (((counter + 1) % CAM_FREQ_DIV[i]) == 1)
      {
        HAL_GPIO_WritePin(Cam_SyncOut_GPIO_Port[i], Cam_SyncOut_Pin[i],
            GPIO_PIN_RESET);
      }
    }

    /* For LiDAR */
    if (((counter + 1) % LASER_DIV) == 0)
    {
      HAL_GPIO_WritePin(Laser_SyncOut_GPIO_Port, Laser_SyncOut_Pin, GPIO_PIN_RESET);
      gps_pulse_sent = 1;
    }
    else if (((counter + 1) % LASER_DIV) == 10)
    {
      HAL_GPIO_WritePin(Laser_SyncOut_GPIO_Port, Laser_SyncOut_Pin, GPIO_PIN_SET);
    }
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
