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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "retarget.h"
#include "sensor.h"
#include "MotionSP.h"
#include "lora_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LPP_DATATYPE_HUMIDITY 0x68
#define LPP_DATATYPE_TEMPERATURE 0x67
#define LPP_DATATYPE_BAROMETER 0x73
#define LPP_DATATYPE_ADC 0x74
#define LPP_DATATYPE_PULSE 0x75

//#define ENABLE_LORA
#define ENABLE_SENSOR

#ifdef ENABLE_LORA
#define LORAWAN_APP_PORT           99;            /*LoRaWAN application port*/
#define LORAWAN_CONFIRMED_MSG      ENABLE         /*LoRaWAN confirmed messages*/
#define JOIN_MODE                  OTAA_JOIN_MODE   /*ABP_JOIN_MODE */ /*LoRaWan join methode*/
#endif

#define PULSE_COUNTER_COUNT 		1
#define PULSE_COEF                  4
#define PULSE_MINUTE                60
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t pulse[PULSE_COUNTER_COUNT];
uint16_t calibrated_pulse[PULSE_COUNTER_COUNT];

TIM_HandleTypeDef *hAccellCycleTimer; //200ms

TIM_HandleTypeDef *hPulseCycleTimer; //1s
TIM_HandleTypeDef *hPulseCounter[PULSE_COUNTER_COUNT];
TIM_HandleTypeDef *hCollectCycleTimer; //5s

#ifdef ENABLE_LORA
static LoRaDriverParam_t LoRaDriverParam = {0, JOIN_MODE};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#ifdef ENABLE_LORA
static void PrepareSensorData(sSendDataBinary_t *SendDataBinary);
static LoRaDriverCallback_t LoRaDriverCallbacks = { PrepareSensorData, NULL };
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define min(a,b) (a) < (b) ? (a) : (b)
#define max(a,b) (a) > (b) ? (a) : (b)
#define abs(a) ((a) > 0 ? (a) : -(a))
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&hlpuart1);

  hmodem_uart = &huart1;

  hAccellCycleTimer = &htim4; //200ms
  hSendCycleTimer = &htim17; //15s
  hPulseCycleTimer = &htim7; //1s
  hPulseCounter[0] = &htim3;
  hCollectCycleTimer = &htim16; //5s
  hProtectCycleTimer = &htim6; //18s
  hADC1Timer = &htim1;
  hADC2Timer = &htim2;

  /* Context Initialization following the LoRa device modem Used*/
#ifdef ENABLE_LORA
  Lora_Ctx_Init(&LoRaDriverCallbacks, &LoRaDriverParam);
#endif

  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

#ifdef ENABLE_SENSOR

  if (Init_Accelerator() != 0)
  {
	  printf("Can not init Accelerator! \r\n");

  } else {

	  printf("Accelerator is initialized! \r\n");
  }
#endif

  if (AnalogDevicesInit() != TRUE) {
	  printf("ADCs can not initialized! \r\n");

  } else {
	  printf("ADCs initialized! \r\n");
  }

  //TIMERS
  start_timer(hPulseCycleTimer);
  start_timer(hCollectCycleTimer);
  start_timer(hAccellCycleTimer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  printf("Starting \r\n");

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#ifdef ENABLE_LORA
    Lora_fsm();
#endif

#ifdef ENABLE_SENSOR
/* Data processing started */
    if (Process_Accelerator_Data() != TRUE)
    {
    	printf("Can not process accelerator data! \r\n");
    }
#endif
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == hAccellCycleTimer) {

//		if (CollectAcceleratorData() != TRUE) {
//			printf("Can not get Accelerator data! \r\n");
//
//		} else {
//			printf("Collected Accelerator data :  x = %d, y = %d, z = %d \r\n",
//					accelero[0], accelero[1], accelero[2]);
//		}
		return;
	}

	if (htim == hPulseCycleTimer) {
		
		for (int i = 0; i < PULSE_COUNTER_COUNT; i++) {
			HAL_TIM_Base_Stop(hPulseCounter[i]);
			pulse[i] = __HAL_TIM_GetCounter(hPulseCounter[i]);
			HAL_TIM_GenerateEvent(hPulseCounter[i], TIM_EVENTSOURCE_UPDATE);
			HAL_TIM_Base_Start(hPulseCounter[i]);
			//calibrated_pulse[i] = pulse[i] * (PULSE_MINUTE / PULSE_COEF);
			calibrated_pulse[i] = 0x1250;
		}

		return;
	}

	if (htim == hCollectCycleTimer) {

/*
		if (CollectSensorData() != COMPONENT_OK) {
			printf("Can not get sensor data! \r\n");

		} else {
			printf("Collected sensor data :  temperature = %d, pressure = %d, humidity = %d \r\n",
					temperature, pressure, humidity);
		}
*/

		if (CollectAnalogData() != TRUE) {
			printf("Can not get analog data! \r\n");

		} else {

			printf("Collected analog data :  adc1[0] = %d, adc1[1] = %d, adc2[0] = %d \r\n",
			processed_adc1[0], processed_adc1[1], processed_adc2[0] );
		}

		return;
	} 

	if (htim == hSendCycleTimer) {
#ifdef ENABLE_LORA
		Lora_OnNextSensorMeasureTimerEvt(NULL);
#endif
		return;
	}

	if (htim == hProtectCycleTimer) {
#ifdef ENABLE_LORA
	//	Lora_OnJoinProtectTimerEvt(NULL); //disabled for now, should be reenabled later
#endif
	}
}

#ifdef ENABLE_LORA
static void PrepareSensorData(sSendDataBinary_t *SendDataBinary)
{
	int index = 0;

//	SendDataBinary->Buffer[index++] = cchannel++;
/*	SendDataBinary->Buffer[index++] = LPP_DATATYPE_BAROMETER;
	SendDataBinary->Buffer[index++] = (pressure >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = pressure & 0xFF;
	SendDataBinary->Buffer[index++] = cchannel++;
	SendDataBinary->Buffer[index++] = LPP_DATATYPE_TEMPERATURE;
	SendDataBinary->Buffer[index++] = (temperature >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = temperature & 0xFF;
	SendDataBinary->Buffer[index++] = cchannel++;
	SendDataBinary->Buffer[index++] = LPP_DATATYPE_HUMIDITY;
	SendDataBinary->Buffer[index++] = humidity & 0xFF;
*/
	SendDataBinary->Buffer[index++] = LPP_DATATYPE_ADC;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[0] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[0] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[1] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[1] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[2] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[2] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[3] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[3] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[4] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[4] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc1[5] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc1[5] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[0] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[0] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[1] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[1] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[2] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[2] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[3] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[3] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[4] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[4] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[5] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[5] & 0xFF;
	SendDataBinary->Buffer[index++] = (calibrated_adc2[6] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_adc2[6] & 0xFF;

	SendDataBinary->Buffer[index++] = LPP_DATATYPE_PULSE;
	SendDataBinary->Buffer[index++] = (calibrated_pulse[0] >> 8) & 0xFF;
	SendDataBinary->Buffer[index++] = calibrated_pulse[0] & 0xFF;


	SendDataBinary->DataSize = index;
	SendDataBinary->Port = LORAWAN_APP_PORT;
	SendDataBinary->Ack = !LORAWAN_CONFIRMED_MSG;
}
#endif

#if 0
/**
  * @brief  Get message
  * @param  Msg the pointer to the message to be handled
  * @param  AxisActive currently active axis index
  * @retval None
  */
void Get_Msg(TMsg *Msg, ACTIVE_AXIS_t AxisActive)
{
  uint32_t k = 0;
  float err = -1.0f;
  float temp;

  memcpy(&Msg->Data[3], (void *) &AcceleroODR.Frequency, sizeof(float));
  Msg->Len = 3 + sizeof(float);

  /* Store FFT values to Msg */
  /* Store TimeDomain parameter values to Msg */
  switch (AxisActive)
  {
    case X_AXIS:
      for (k = 0; k < MotionSP_Parameters.FftSize / 2; k++)
      {
        temp = AccAxesAvgMagBuff.AXIS_X[k] * 1000; // * 1000 for Unicleo-GUI
        memcpy(&Msg->Data[Msg->Len + k * sizeof(float)], (void *) &temp, sizeof(float));
      }
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.AccRms.AXIS_X, sizeof(float));
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.AccPeak.AXIS_X, sizeof(float));
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.SpeedRms.AXIS_X, sizeof(float));
      break;

    case Y_AXIS:
      for (k = 0; k < MotionSP_Parameters.FftSize / 2; k++)
      {
        temp = AccAxesAvgMagBuff.AXIS_Y[k] * 1000; // * 1000 for Unicleo-GUI
        memcpy(&Msg->Data[Msg->Len + k * sizeof(float)], (void *) &temp, sizeof(float));
      }
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.AccRms.AXIS_Y, sizeof(float));
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.AccPeak.AXIS_Y, sizeof(float));
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.SpeedRms.AXIS_Y, sizeof(float));
      break;

    case Z_AXIS:
      for (k = 0; k < MotionSP_Parameters.FftSize / 2; k++)
      {
        temp = AccAxesAvgMagBuff.AXIS_Z[k] * 1000; // * 1000 for Unicleo-GUI
        memcpy(&Msg->Data[Msg->Len + k * sizeof(float)], (void *) &temp, sizeof(float));
      }
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.AccRms.AXIS_Z, sizeof(float));
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.AccPeak.AXIS_Z, sizeof(float));
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &sTimeDomain.SpeedRms.AXIS_Z, sizeof(float));
      break;

    default:
      /* Store -1 to Msg to indicate error */
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &err, sizeof(float));
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &err, sizeof(float));
      memcpy(&Msg->Data[Msg->Len + k++ * sizeof(float)], (void *) &err, sizeof(float));
      break;
  }
  Msg->Len += k * sizeof(float);

  /* Store active axis index to Msg */
  Msg->Data[Msg->Len] = (uint8_t)AxisActive;
  Msg->Len += sizeof(uint8_t);
}
#endif

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
