/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

//#include "component.h"

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

extern uint16_t adc1[6];
extern uint16_t adc2[8];

extern uint16_t calibrated_adc1[6];
extern uint16_t calibrated_adc2[8];

extern uint16_t processed_adc1[6];
extern uint16_t processed_adc2[8];

extern TIM_HandleTypeDef *hADC1Timer;
extern TIM_HandleTypeDef *hADC2Timer;

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */

unsigned char AnalogDevicesInit();
unsigned char CollectAnalogData();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
