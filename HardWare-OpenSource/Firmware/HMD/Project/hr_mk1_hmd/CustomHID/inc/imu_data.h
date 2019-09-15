/*
* The MIT License (MIT)
* Copyright (c) 2017 Shanghai Chai Ming Huang Info&Tech Co£¬Ltd
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/
/**
  ******************************************************************************
  * @file    imu_data.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for imu_data.c.
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "hr_mk1_config.h"

/* Exported types ------------------------------------------------------------*/
#pragma pack (1)
typedef struct
{
    int16_t gyro[3];                            //0: X,1: Y,2:Z 
    int16_t acce[3];                            //0: X,1: Y,2:Z 
} imu_data;

#pragma pack (1)
typedef struct
{
    uint32_t	Timestamp;
    imu_data    Samples[2];                     //0:first time,1: second time
    int16_t     MagneticField[3];               //0: X,1: Y,2:Z
    int16_t     Temperature;
    uint16_t    InterpupillaryDistance;
    uint8_t     ProximitySwitch;
    uint8_t     Reserve;
} imu_report_type;

#pragma pack (1)
typedef struct temp_correction_struct {
    float slope[3];                             //0: X,1: Y,2:Z
    float intercept[3];                         //0: X,1: Y,2:Z
} correction_s, *correction_p;

#pragma pack (1)
typedef struct calibrate_struct {
    correction_s gyro_correction;
    correction_s acc_correction;
} calibrate_s, *calibrate_p;

typedef calibrate_s imu_calib_type;

/* Exported constants --------------------------------------------------------*/
#define IMU_FIRST       0
#define IMU_SECOND      1

#define IMU_GYRO_SCALE	(65536.0/60000)
#define IMU_ACCE_SCALE	(65536.0/60000)

#define MAX_FLOAT       32767
#define MIN_FLOAT       -32768

/* Exported functions ------------------------------------------------------- */
extern imu_report_type imu_report_buf;
extern uint8_t systick_2ms_first_flag;
extern uint8_t systick_2ms_second_flag;
extern uint16_t InterpupillaryDistance;

extern uint32_t systick_count_l;
extern uint32_t systick_count_h;

void imu_data_init();
void imu_data_process(uint8_t ping_pang);
void imu_data_poll();

#endif /* _IMU_DATA_H_ */

/*****************************END OF FILE****/