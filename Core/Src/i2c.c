/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#define TIMEOUT_DURATION 1000
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x108031A0;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8-BOOT0     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB8);

    __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB9);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8-BOOT0     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Initialize a bus
  * @param None
  * @retval BSP status
  */
int32_t BSP_I2C1_Init(void)
{

  int32_t ret = BSP_ERROR_NONE;

  hi2c1.Instance  = I2C1;

  if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET)
  {
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
    /* Init the I2C Msp */
	  HAL_I2C_MspInit(&hi2c1);
#else
    if (IsI2C1MspCbValid == 0U)
    {
      if (BSP_I2C1_RegisterDefaultMspCallbacks() != BSP_ERROR_NONE)
      {
        return BSP_ERROR_MSP_FAILURE;
      }
    }
#endif

    /* Init the I2C */
    MX_I2C1_Init();

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  return ret;
}

/**
  * @brief  DeInitialize a bus
  * @param None
  * @retval BSP status
  */
int32_t BSP_I2C1_DeInit(void)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 0)
  /* DeInit the I2C */
  HAL_I2C_MspDeInit(&hi2c1);
#endif

  if (HAL_I2C_DeInit(&hi2c1) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Return the status of the Bus
  * @retval bool
  */
int32_t BSP_I2C1_IsReady(void)
{
  return (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY);
}

/**
  * @brief  Write registers through bus (8 bits)
  * @param  Addr: Device address on Bus.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  * @retval BSP status
  */
int32_t BSP_I2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  if (HAL_I2C_Mem_Write(&hi2c1, (uint8_t)DevAddr,
                        (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t *)pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Read registers through a bus (8 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @retval BSP status
  */
int32_t  BSP_I2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  if (HAL_I2C_Mem_Read(&hi2c1, DevAddr, (uint16_t)Reg,
                       I2C_MEMADD_SIZE_8BIT, pData,
                       len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = HAL_OK;
  }

  return ret;
}

/**
  * @brief  Write registers through bus (16 bits)
  * @param  Addr: Device address on Bus.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  * @retval BSP status
  */
int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  if (HAL_I2C_Mem_Write(&hi2c1, (uint8_t)DevAddr,
                        (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT,
                        (uint8_t *)pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Read registers through a bus (16 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @retval BSP status
  */
int32_t  BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  if (HAL_I2C_Mem_Read(&hi2c1, DevAddr, (uint16_t)Reg,
                       I2C_MEMADD_SIZE_16BIT, pData,
                       len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Send an amount width data through bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  if (HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = len;
  }

  return ret;
}

/**
  * @brief  Receive an amount of data through a bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  if (HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = len;
  }

  return ret;
}

/**
  * @brief  Send and receive an amount of data through bus (Full duplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pTxdata: Transmit data pointer
  * @param  pRxdata: Receive data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C1_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  /*
   * Send and receive an amount of data through bus (Full duplex)
   * I2C is Half-Duplex protocol
   */
  if ((BSP_I2C1_Send(DevAddr, pTxdata, len) == len) && \
      (BSP_I2C1_Recv(DevAddr, pRxdata, len) == len))
  {
    ret = len;
  }

  return ret;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
