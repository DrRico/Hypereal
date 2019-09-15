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
  * @file    calibrate.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides calibrate data about accel and gyroscope.
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "calibrate.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct averaging_struct {
    uint32_t sample_count;
    int32_t gyro[3];                    //0: X-axis,1: Y-axis,2: Z-axis
    int32_t acc[3];                     //0: X-axis,1: Y-axis,2: Z-axis
    int32_t temp;
} averaging_s, *averaging_p;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static calibrate_s g_cal;
static averaging_s avg = {0};
float dcorrection[3];                   //0: X-axis,1: Y-axis,2: Z-axis

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  clear remainders from IMU offset.
  * @param  None
  * @retval None
  */
void averaging_reset()
{
    memset(&avg, 0, sizeof(averaging_s));

    dcorrection[0] = 0;
    dcorrection[1] = 0;
    dcorrection[2] = 0;
    
}

/**
  * @brief  init IMU offset.
  * @param  None
  * @retval None
  */
void calibrate_init(void)
{
    memset(&g_cal, 0, sizeof(g_cal));
    averaging_reset();
}

/**
  * @brief  calibrate IMU data.
  * @param  None
  * @retval None
  */
bool calibrate_apply(imu_data *data)
{	
    for(int i=0;i<3;i++)
    {
        dcorrection[i] += (float)data->gyro[i] - (g_cal.gyro_correction.intercept[i]);
        data->gyro[i] = dcorrection[i];
		dcorrection[i] -= data->gyro[i];
    }
    
    for(int i=0;i<3;i++)
    {
        data->acce[i] = (float)data->acce[i] - (g_cal.acc_correction.intercept[i]);
    }

    return 0; 
}

/*****************************END OF FILE****/