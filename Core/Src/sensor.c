#include "iks01a2_motion_sensors.h"
#include "iks01a2_motion_sensors_ex.h"
#include "MotionSP.h"
#include "sensor_def.h"
#include "sensor.h"

#define LSM6DSL_HP_DISABLE        0xFBU  /* Disable HP filter */
#define LSM6DSL_HP_ENABLE_DIV400  0x34U  /* Enable HP filter, DIV/400 */
#define LSM6DSL_DEFAULT_ODR       417.0f /* Default output/batch data rate */
#define LSM6DSL_DEFAULT_FS        2      /* Default full scale */
#define NUM_SENSORS  1

static volatile uint32_t IntCurrentTime1 = 0;
static volatile uint32_t IntCurrentTime2 = 0;
static volatile uint8_t AccIntReceived = 0;
static uint8_t RestartFlag = 1;
static uint32_t StartTick = 0;

uint8_t SensorList[NUM_SENSORS + 1];

sensor_conf_t sensorConf = {.hp_filter = 0, .switch_HP_to_DC_null = 0};

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
void Init_Sensors(void)
{
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);

  /* Set accelerometer:
   *   - ODR >= 416Hz
   *   - FS   = <-2g, 2g>
   */
  (void)IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, LSM6DSL_DEFAULT_ODR);
  (void)IKS01A2_MOTION_SENSOR_SetFullScale(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, LSM6DSL_DEFAULT_FS);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	BSP_LED_Toggle(LED_GREEN);
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

//	if( GPIO_Pin == LSM303AGR_INT_Pin )
//	{
//	//  AXL_INT_received = 1;
//	  printf("AXL_INT_received! \r\n");
//	}

  if (GPIO_Pin == GPIO_PIN_5)
  {
	AccIntReceived = 1;
  }
}

/**
  * @brief  Collect data from accelerometer
  * @param  None
  * @retval 1 in case of success
  * @retval 0 otherwise
  */
uint8_t Collect_Data(void)
{
  uint16_t pattern;
  uint16_t samples_in_fifo = 0;
  int32_t acceleration;
  uint32_t start = HAL_GetTick();
  SensorVal_f_t single_data;
  SensorVal_f_t single_data_no_dc;
  uint8_t exit_cond = 0;

  if (FinishAvgFlag == 0 && fftIsEnabled == 0 && AccIntReceived == 1)
  {
    AccIntReceived = 0;

    IKS01A2_MOTION_SENSOR_FIFO_Get_Num_Samples(IKS01A2_LSM6DSL_0, &samples_in_fifo);

    if ((samples_in_fifo / 3U) < MotionSP_Parameters.FftSize)
    {
      Restart_FIFO();
      return 0;
    }

    while (fftIsEnabled == 0)
    {
      if (((HAL_GetTick() - start) > 6000))
      {
        Restart_FIFO();
        return 0;
      }

      IKS01A2_MOTION_SENSOR_FIFO_Get_Pattern(IKS01A2_LSM6DSL_0, &pattern);
      IKS01A2_MOTION_SENSOR_FIFO_Get_Axis(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, &acceleration);

      switch (pattern)
      {
        case X_AXIS:
          single_data.AXIS_X = (float)acceleration;
          break;

        case Y_AXIS:
          single_data.AXIS_Y = (float)acceleration;
          break;

        case Z_AXIS:
          single_data.AXIS_Z = (float)acceleration;

          /* Remove DC offset */
          MotionSP_accDelOffset(&single_data_no_dc, &single_data, DC_SMOOTH, RestartFlag);

          /* Fill the accelero circular buffer */
          MotionSP_CreateAccCircBuffer(&AccCircBuffer, single_data_no_dc);

          if (AccCircBuffer.Ovf == 1)
          {
            fftIsEnabled = 1;
            AccCircBuffer.Ovf = 0;
          }

          MotionSP_TimeDomainProcess(&sTimeDomain, (Td_Type_t)MotionSP_Parameters.td_type, RestartFlag);
          RestartFlag = 0;
          break;

        default:
          exit_cond = 1;
          break;
      }

      if (exit_cond)
      {
        return 0;
      }
    }

    if (!Restart_FIFO())
    {
      return 0;
    }
  }

  return 1;
}

/**
 * @brief  Measure accelerometer real ODR
 * @param  None
 * @retval 1 in case of success 0 otherwise
 */
uint8_t Meas_Odr(void)
{
  uint8_t  odr_meas_enable = 1;
  uint16_t odr_meas_iter = 0;
  uint16_t odr_meas_start_time = 0;
  uint16_t odr_meas_stop_time = 0;
  uint16_t odr_meas_samples = 150; /* number of measured samples for calculating ODR */
  uint32_t start = 0;

  if (!Disable_FIFO())
  {
    return 0;
  }

  /* Set DRDY pulsed mode */
  if (IKS01A2_MOTION_SENSOR_DRDY_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_DRDY_PULSED) != BSP_ERROR_NONE)
  {
    return 0;
  }

  if (!Enable_DRDY())
  {
    return 0;
  }

  start = HAL_GetTick();

  while (odr_meas_enable)
  {
    if (((HAL_GetTick() - start) > 1000))
    {
      return 0;
    }

    if (AccIntReceived)
    {
      AccIntReceived = 0;

      /* Get start time */
      if (odr_meas_iter == 0)
      {
        IntCurrentTime1 = HAL_GetTick();
        odr_meas_start_time = IntCurrentTime1;
      }

      /* Get stop time */
      if (odr_meas_iter == odr_meas_samples - 1)
      {
        IntCurrentTime2 = HAL_GetTick();
        odr_meas_stop_time = IntCurrentTime2;
        odr_meas_enable = 0;
      }

      /* Stop after measuring "odr_meas_samples" values */
      if (odr_meas_iter < odr_meas_samples)
      {
        odr_meas_iter++;
      }
    }
  }

  /* Calculate measured ODR */
  AcceleroODR.Frequency = ((float)(1000 * odr_meas_samples) / (odr_meas_stop_time - odr_meas_start_time));

  if (AcceleroODR.Frequency != 0)
  {
    AcceleroODR.Period = 1 / AcceleroODR.Frequency;
  }

  AcceleroODR.Tau = exp(-(float)(1000 * AcceleroODR.Period) / MotionSP_Parameters.tau);

  /* Disable accelerometer to avoid interrupt conflicts on highest ODRs */
  if (IKS01A2_MOTION_SENSOR_Disable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO) != BSP_ERROR_NONE)
  {
    return 0;
  }

  if (!Disable_DRDY())
  {
    return 0;
  }

  /* Set DRDY latched mode */
  if (IKS01A2_MOTION_SENSOR_DRDY_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_DRDY_LATCHED) != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* Enable accelerometer */
  if (IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO) != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* Enable FIFO full flag interrupt */
  if (!Enable_FIFO())
  {
    return 0;
  }

  if (!Restart_FIFO())
  {
    return 0;
  }

  return 1;
}

/**
 * @brief  Changes HP to DCnull and vice versa in main menu options
 * @param  None
 * @retval void
 */
void HP_DC_Changer(void)
{
  uint8_t ret_err = 0;
  uint8_t data;

  if (sensorConf.switch_HP_to_DC_null)
  {
	  sensorConf.switch_HP_to_DC_null = 0;
	  sensorConf.hp_filter = 0;
  }
  else
  {
    /* Disable HP filter */
    if (SensorSetting.hp_filter_available)
    {
      if (IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM6DSL_0, LSM6DSL_CTRL8_XL, &data) != BSP_ERROR_NONE)
      {
        ret_err = 1;
      }

      data &= LSM6DSL_HP_DISABLE;

      if (IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM6DSL_0, LSM6DSL_CTRL8_XL, data) != BSP_ERROR_NONE)
      {
        ret_err = 1;
      }
    }

    if (ret_err == 0)
    {
    	sensorConf.switch_HP_to_DC_null = 1;
    	sensorConf.hp_filter = 0;
      fftIsEnabled = 0;
    }
  }
}

/**
  * @brief  Enable DRDY
  * @param  None
  * @retval 1 in case of success
  * @retval 0 in case of failure
  */
uint8_t Enable_DRDY(void)
{
  IKS01A2_MOTION_SENSOR_AxesRaw_t axes;

  AccIntReceived = 0;

  /* Enable DRDY */
  if (IKS01A2_MOTION_SENSOR_Set_INT1_DRDY(IKS01A2_LSM6DSL_0, PROPERTY_ENABLE) != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* Clear DRDY */
  if (IKS01A2_MOTION_SENSOR_GetAxesRaw(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, &axes) != BSP_ERROR_NONE)
  {
    return 0;
  }

  return 1;
}

/**
  * @brief  Disable DRDY
  * @param  None
  * @retval 1 in case of success
  * @retval 0 in case of failure
  */
uint8_t Disable_DRDY(void)
{
  /* Disable DRDY */
  if (IKS01A2_MOTION_SENSOR_Set_INT1_DRDY(IKS01A2_LSM6DSL_0, PROPERTY_DISABLE) != BSP_ERROR_NONE)
  {
    return 0;
  }

  return 1;
}

/**
  * @brief  Enable FIFO measuring
  * @param  None
  * @retval 1 in case of success
  * @retval 0 in case of failure
  */
uint8_t Enable_FIFO(void)
{
  /* Enable FIFO full flag interrupt */
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_INT1_FIFO_Full(IKS01A2_LSM6DSL_0, PROPERTY_ENABLE) != BSP_ERROR_NONE)
  {
    return 0;
  }

  return 1;
}

/**
  * @brief  Disable FIFO measuring
  * @param  None
  * @retval 1 in case of success
  * @retval 0 in case of failure
  */
uint8_t Disable_FIFO(void)
{
  /* Set FIFO to bypass mode */
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_BYPASS_MODE) != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* Disable FIFO full flag interrupt */
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_INT1_FIFO_Full(IKS01A2_LSM6DSL_0, PROPERTY_DISABLE) != BSP_ERROR_NONE)
  {
    return 0;
  }

  return 1;
}

/**
  * @brief  Find available accelerometers that can be used for FFT demo
  * @param  None
  * @retval Number of found sensors
  */
uint8_t Get_Available_Sensors(void)
{
  uint8_t sensors_available = 1;

  /* NOTE: Currently done for just one sensor but possible to add mechanism for available sensors detection here */

  SensorList[sensors_available] = IKS01A2_LSM6DSL_0;

  SensorList[0] = sensors_available;

  return SensorList[0];
}

/**
  * @brief  Enable/Disable HP or DCnull
  * @param  void
  * @retval void
  */
void En_Dis_HP_Or_DCnull(void)
{
  uint8_t ret_err = 0;
  uint8_t data;

  if (sensorConf.switch_HP_to_DC_null)
  {
    if (sensorConf.hp_filter)
    {
    	sensorConf.hp_filter = 0;
    }
    else
    {
    	sensorConf.hp_filter = 1;
    }

    fftIsEnabled = 0;
  }
  else
  {
    if (sensorConf.hp_filter)
    {
      /* Disable HP filter */
      if (IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM6DSL_0, LSM6DSL_CTRL8_XL, &data) != BSP_ERROR_NONE)
      {
        ret_err = 1;
      }

      data &= LSM6DSL_HP_DISABLE;

      if (IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM6DSL_0, LSM6DSL_CTRL8_XL, data) != BSP_ERROR_NONE)
      {
        ret_err = 1;
      }

      if (ret_err == 0)
      {
    	  sensorConf.hp_filter = 0;
        fftIsEnabled = 0;
      }
    }
    else
    {
      /* Enable HP filter */
      if (IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM6DSL_0, LSM6DSL_CTRL8_XL, &data) != BSP_ERROR_NONE)
      {
        ret_err = 1;
      }

      data |= LSM6DSL_HP_ENABLE_DIV400;

      if (IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM6DSL_0, LSM6DSL_CTRL8_XL, data) != BSP_ERROR_NONE)
      {
        ret_err = 1;
      }

      if (ret_err == 0)
      {
    	  sensorConf.hp_filter = 1;
        fftIsEnabled = 0;
      }
    }
    HAL_Delay(40);
    Restart_FIFO();
  }
}

/**
  * @brief  Initialize demo
  * @param  None
  * @retval 1 in case of success
  * @retval 0 in case of failure
  */
uint8_t Init_Demo(void)
{
  uint8_t data;

  /* Disable HP filter if needed */
  if (SensorSetting.hp_filter_available == 1)
  {
    if (IKS01A2_MOTION_SENSOR_Read_Register(IKS01A2_LSM6DSL_0, LSM6DSL_CTRL8_XL, &data) != BSP_ERROR_NONE)
    {
      return 0;
    }

    data &= LSM6DSL_HP_DISABLE;

    if (IKS01A2_MOTION_SENSOR_Write_Register(IKS01A2_LSM6DSL_0, LSM6DSL_CTRL8_XL, data) != BSP_ERROR_NONE)
    {
      return 0;
    }
  }

  /* Turn-on time delay */
  HAL_Delay(40);

  if (Enable_DRDY() == 0)
  {
    return 0;
  }

  sensorConf.switch_HP_to_DC_null = 0;
  sensorConf.hp_filter = 0;
  fftIsEnabled = 0;

  /* Set parameters for MotionSP library */
  MotionSP_Parameters.FftSize = FFT_SIZE_DEFAULT;
  MotionSP_Parameters.tau = TAU_DEFAULT;
  MotionSP_Parameters.window = WINDOW_DEFAULT;
  MotionSP_Parameters.td_type = TD_DEFAULT;
  MotionSP_Parameters.tacq = TACQ_DEFAULT;

  /* Create circular buffer and initialize result variables */
  AccCircBuffer.Size = MotionSP_Parameters.FftSize;
  AccCircBuffer.IdPos = 0;
  AccCircBuffer.Ovf = 0;

  magSize = MotionSP_Parameters.FftSize / 2;

  /* Reset circular buffer for storing accelerometer values */
  memset(AccCircBuffer.Data.AXIS_X, 0x00, (AccCircBuffer.Size) * (sizeof(float)));
  memset(AccCircBuffer.Data.AXIS_Y, 0x00, (AccCircBuffer.Size) * (sizeof(float)));
  memset(AccCircBuffer.Data.AXIS_Z, 0x00, (AccCircBuffer.Size) * (sizeof(float)));

  /* Reset the TimeDomain parameter values */
  sTimeDomain.AccRms.AXIS_X = 0.0f;
  sTimeDomain.AccRms.AXIS_Y = 0.0f;
  sTimeDomain.AccRms.AXIS_Z = 0.0f;
  sTimeDomain.AccPeak.AXIS_X = 0.0f;
  sTimeDomain.AccPeak.AXIS_Y = 0.0f;
  sTimeDomain.AccPeak.AXIS_Z = 0.0f;
  sTimeDomain.SpeedRms.AXIS_X = 0.0f;
  sTimeDomain.SpeedRms.AXIS_Y = 0.0f;
  sTimeDomain.SpeedRms.AXIS_Z = 0.0f;

  /* Reset the counters of the number of sums about the calculation of the average */
  AccSumCnt.AXIS_X = 0;
  AccSumCnt.AXIS_Y = 0;
  AccSumCnt.AXIS_Z = 0;

  MotionSP_SetWindFiltArray(Filter_Params, MotionSP_Parameters.FftSize, (Filt_Type_t)MotionSP_Parameters.window);

  /* Reset the flag to enable FFT computation */
  fftIsEnabled = 0;

  arm_rfft_fast_init_f32(&fftS, MotionSP_Parameters.FftSize);

  accCircBuffIndexForFft = MotionSP_Parameters.FftSize - 1; /* It is the minimum value to do the first FFT */

  /* Enable AXL data to FIFO with no decimation */
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_Decimation(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, LSM6DSL_FIFO_XL_NO_DEC) != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* Set FIFO ODR to highest value */
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_ODR_Value(IKS01A2_LSM6DSL_0, 6660.0f) != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* Set FIFO watermark level */
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_Watermark_Level(IKS01A2_LSM6DSL_0, (MotionSP_Parameters.FftSize + 1) * 3) != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* Set FIFO to stop on FIFO threshold */
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_Stop_On_Fth(IKS01A2_LSM6DSL_0, PROPERTY_ENABLE) != BSP_ERROR_NONE)
  {
    return 0;
  }

  if (SensorSetting.hp_filter_available == 0)
  {
    HP_DC_Changer();
  }

  /* Measure and calculate ODR */
  if (Meas_Odr() == 0)
  {
    return 0;
  }

  return 1;
}


/**
  * @brief  Restart FIFO
  * @param  None
  * @retval 1 in case of success
  * @retval 0 in case of failure
  */
uint8_t Restart_FIFO(void)
{
  AccIntReceived = 0;

  /* FIFO Bypass Mode */
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_BYPASS_MODE) != BSP_ERROR_NONE)
  {
    return 0;
  }

  /* FIFO Mode*/
  if (IKS01A2_MOTION_SENSOR_FIFO_Set_Mode(IKS01A2_LSM6DSL_0, LSM6DSL_FIFO_MODE) != BSP_ERROR_NONE)
  {
    return 0;
  }

  return 1;
}

/**
  * @brief  Set accelerometer ODR
  * @param  value the index of ODR to be set
  * @retval 1 in case of success
  * @retval 0 in case of failure
  */
uint8_t Set_ODR(uint8_t value)
{
  /* Set chosen ODR */
  if ((value <= SensorSetting.odr_list[0]) && value != 0)
  {
    if (IKS01A2_MOTION_SENSOR_SetOutputDataRate(IKS01A2_LSM6DSL_0, MOTION_ACCELERO, SensorSetting.odr_list[value]) != BSP_ERROR_NONE)
    {
      return 0;
    }

    Meas_Odr();
    fftIsEnabled = 0;

    if (sensorConf.hp_filter == 1)
    {
      HAL_Delay((uint32_t)(320 / AcceleroODR.Frequency));
      Restart_FIFO();
    }
  }

  return 1;
}

/**
  * @brief  Get HP filter status
  * @param  None
  * @retval HP filter status
  */
uint8_t Get_HP_Filter(void)
{
  return sensorConf.hp_filter;
}

/**
  * @brief  Get number of FFT samples
  * @param  None
  * @retval FFT samples
  */
uint16_t Get_Samples(void)
{
  return MotionSP_Parameters.FftSize;
}

/**
  * @brief  Get list of available sensors
  * @param  None
  * @retval Sensor list
  */
uint8_t *Get_Sensor_List(void)
{
  Get_Available_Sensors();

  return SensorList;
}

uint8_t Process_Sensor_Data(void)
{
	ACTIVE_AXIS_t axis_active;

  if (Collect_Data() != 1)
  {
	return FALSE;
  }

  /* Perform Frequency Domain analysis if buffer is full */
  if (fftIsEnabled == 1)
  {
	fftIsEnabled = 0;

	if ((HAL_GetTick() - StartTick) >= MotionSP_Parameters.tacq)
	{
	  FinishAvgFlag = 1;
	  StartTick = HAL_GetTick();
	}

	MotionSP_FrequencyDomainProcess();
  }

  /* Send data to GUI if total acquisition time is reached */
  if (FinishAvgFlag == 1)
  {
	FinishAvgFlag = 0;

	/* Send all 3 axes data to Unicleo-GUI */
	for (axis_active = X_AXIS; axis_active < NUM_AXES; axis_active++)
	{
//	  INIT_STREAMING_HEADER(&msg_dat);
//	  Get_Msg(&msg_dat, axis_active);
//	  UART_SendMsg(&msg_dat);
	}
	RestartFlag = 1;
  }

  return TRUE;
}

