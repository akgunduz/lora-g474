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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_nucleo.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define TRUE 1
#define FALSE 0
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLUE_BTN_Pin GPIO_PIN_13
#define BLUE_BTN_GPIO_Port GPIOC
#define BLUE_BTN_EXTI_IRQn EXTI15_10_IRQn
#define RSIC_2_Pin GPIO_PIN_2
#define RSIC_2_GPIO_Port GPIOC
#define SICAKLIK_2_Pin GPIO_PIN_3
#define SICAKLIK_2_GPIO_Port GPIOC
#define LORA_LED_Pin GPIO_PIN_0
#define LORA_LED_GPIO_Port GPIOA
#define MIK_1_Pin GPIO_PIN_1
#define MIK_1_GPIO_Port GPIOA
#define DEBUG_UART_TX_Pin GPIO_PIN_2
#define DEBUG_UART_TX_GPIO_Port GPIOA
#define DEBUG_UART_RX_Pin GPIO_PIN_3
#define DEBUG_UART_RX_GPIO_Port GPIOA
#define AUX_3_Pin GPIO_PIN_4
#define AUX_3_GPIO_Port GPIOA
#define NUCLEO_LED_Pin GPIO_PIN_5
#define NUCLEO_LED_GPIO_Port GPIOA
#define MIK_2_Pin GPIO_PIN_6
#define MIK_2_GPIO_Port GPIOA
#define RSIC_1_Pin GPIO_PIN_7
#define RSIC_1_GPIO_Port GPIOA
#define LORA_UART_TX_Pin GPIO_PIN_4
#define LORA_UART_TX_GPIO_Port GPIOC
#define LORA_UART_RX_Pin GPIO_PIN_5
#define LORA_UART_RX_GPIO_Port GPIOC
#define AKIM_3_Pin GPIO_PIN_0
#define AKIM_3_GPIO_Port GPIOB
#define AKIM_2_Pin GPIO_PIN_1
#define AKIM_2_GPIO_Port GPIOB
#define OUTPUT_3_Pin GPIO_PIN_2
#define OUTPUT_3_GPIO_Port GPIOB
#define AUX_1_Pin GPIO_PIN_11
#define AUX_1_GPIO_Port GPIOB
#define AKIM_1_Pin GPIO_PIN_12
#define AKIM_1_GPIO_Port GPIOB
#define OUTPUT_1_Pin GPIO_PIN_13
#define OUTPUT_1_GPIO_Port GPIOB
#define SICAKLIK_1_Pin GPIO_PIN_14
#define SICAKLIK_1_GPIO_Port GPIOB
#define AUX_2_Pin GPIO_PIN_15
#define AUX_2_GPIO_Port GPIOB
#define OUTPUT_11_Pin GPIO_PIN_6
#define OUTPUT_11_GPIO_Port GPIOC
#define OUTPUT_10_Pin GPIO_PIN_7
#define OUTPUT_10_GPIO_Port GPIOC
#define OUTPUT_2_Pin GPIO_PIN_8
#define OUTPUT_2_GPIO_Port GPIOC
#define OUTPUT_8_Pin GPIO_PIN_9
#define OUTPUT_8_GPIO_Port GPIOC
#define OUTPUT_7_Pin GPIO_PIN_9
#define OUTPUT_7_GPIO_Port GPIOA
#define OUTPUT_5_Pin GPIO_PIN_10
#define OUTPUT_5_GPIO_Port GPIOA
#define OUTPUT_6_Pin GPIO_PIN_11
#define OUTPUT_6_GPIO_Port GPIOA
#define OUTPUT_4_Pin GPIO_PIN_12
#define OUTPUT_4_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define DEVIR_Pin GPIO_PIN_2
#define DEVIR_GPIO_Port GPIOD
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define LSM6DS_INT2_Pin GPIO_PIN_4
#define LSM6DS_INT2_GPIO_Port GPIOB
#define LSM6DS_INT2_EXTI_IRQn EXTI4_IRQn
#define LSM6DS_INT1_Pin GPIO_PIN_5
#define LSM6DS_INT1_GPIO_Port GPIOB
#define LSM6DS_INT1_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
