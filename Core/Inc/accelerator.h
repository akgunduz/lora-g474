/*
 * sensor.h
 *
 *  Created on: Nov 20, 2020
 *      Author: akgun
 */

#ifndef INC_ACCELERATOR_H_
#define INC_ACCELERATOR_H_

#include <stdio.h>

#define PRESSURE_SENSOR                         0x00000001U
#define TEMPERATURE_SENSOR                      0x00000002U
#define HUMIDITY_SENSOR                         0x00000004U
#define UV_SENSOR                               0x00000008U /* for future use */
#define ACCELEROMETER_SENSOR                    0x00000010U
#define GYROSCOPE_SENSOR                        0x00000020U
#define MAGNETIC_SENSOR                         0x00000040U

/* Typedefs ------------------------------------------------------------------*/
typedef enum
{
  X_AXIS = 0,
  Y_AXIS,
  Z_AXIS,
  ALL_ACTIVE
} ACTIVE_AXIS_t;

typedef struct
{
  const char *name;
  const float *odr_list;
  const int32_t *fs_list;
  const ACTIVE_AXIS_t *axis_list;
  const uint32_t *samples_list;
  const uint8_t hp_filter_available;
} accelerator_setting_t;

/* Exported variables --------------------------------------------------------*/
extern const accelerator_setting_t acceleratorSetting;

typedef struct
{
  uint8_t hp_filter;
  uint8_t switch_HP_to_DC_null;
} accelerator_conf_t;

extern accelerator_conf_t acceleratorConf;
//extern float OdrMeasured;

uint8_t Collect_Data(void);
uint8_t Meas_Odr(void);
void HP_DC_Changer(void);
uint8_t Enable_DRDY(void);
uint8_t Disable_DRDY(void);
uint8_t Enable_FIFO(void);
uint8_t Disable_FIFO(void);
uint8_t Get_Available_Sensors(void);
void En_Dis_HP_Or_DCnull(void);

uint8_t Init_Accelerator(void);
uint8_t Restart_FIFO(void);
uint8_t Get_HP_Filter(void);
uint8_t Set_HP_Filter(uint8_t value);
uint8_t *Get_Sensor_List(void);
uint8_t Set_ODR(uint8_t value);
uint8_t Set_Full_Scale(uint8_t value);
uint16_t Get_Samples(void);
uint8_t Set_Samples(uint8_t value);

uint8_t Collect_Accelerator_Data(void);

#endif /* INC_ACCELERATOR_H_ */
