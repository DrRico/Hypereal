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
  * @file    imu_data.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides firmware functions to calibrate data about accel and gyroscope.
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "imu_data.h"
#include "calibrate.h"
    
/* Private variables ---------------------------------------------------------*/
imu_report_type imu_report_buf;
imu_calib_type imu_calib_data;    
unsigned char system_status;



/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Reset IMU offset
  * @param  None
  * @retval None
  */
void imu_data_init()
{
	
    imu_report_buf.InterpupillaryDistance = 0;
    imu_report_buf.ProximitySwitch = 0;
    
    imu_report_buf.MagneticField[0] = 0;
    imu_report_buf.MagneticField[1] = 0;
    imu_report_buf.MagneticField[2] = 0;
	
    calibrate_init();
		
    return;
}

/**
  * @brief  calibrate IMU data
  * @param  None
  * @retval None
  */
void imu_rescale(imu_data *acc_gyro_data)
{
	float calc_tmp;
	for (int i = 0;i<3;i++)
	{
		calc_tmp = acc_gyro_data->acce[i] * IMU_ACCE_SCALE;
		if (calc_tmp > MAX_FLOAT) 
                  calc_tmp = MAX_FLOAT;
		else if (calc_tmp < MIN_FLOAT) 
                  calc_tmp = MIN_FLOAT;
		
		acc_gyro_data->acce[i] = (int16_t)calc_tmp;
	}
	for (int i = 0;i<3;i++)
	{
		calc_tmp = acc_gyro_data->gyro[i] * IMU_GYRO_SCALE;
		if (calc_tmp > MAX_FLOAT) 
                  calc_tmp = MAX_FLOAT;
		else if (calc_tmp < MIN_FLOAT) 
                  calc_tmp = MIN_FLOAT;
		
		acc_gyro_data->gyro[i] = (int16_t)calc_tmp;
	}
}

/**
  * @brief  Put measure data from max21100 to report_buf(put 2 times in 2ms)
  * @param  None
  * @retval None
  */
void imu_data_process(uint8_t ping_pang)
{
    int16_t change_tmp;	
    
    
    if (ping_pang == IMU_FIRST)/* first time */
    {
        imu_data *p_sample = &(imu_report_buf.Samples[0]);
        max21100_read_out((uint8_t *)(void *)(p_sample));
        max21100_read_register_n(0x36,2,(uint8_t *)(void *)(&(imu_report_buf.Temperature)));
        imu_report_buf.Timestamp = systick_count_l;
        
        change_tmp = p_sample->acce[0];
        p_sample->acce[0] = -p_sample->acce[1];
        p_sample->acce[1] = -change_tmp;
        p_sample->acce[2] = -p_sample->acce[2];
        
        change_tmp = p_sample->gyro[0];
        p_sample->gyro[0] = -p_sample->gyro[1];
        p_sample->gyro[1] = -change_tmp;
        p_sample->gyro[2] = -p_sample->gyro[2];

        imu_rescale(p_sample);
        calibrate_apply(p_sample);
                
        /* todo magnetic field */
        if (hmc5883_flush_flag)
        {
            memcpy((uint8_t *)(void *)(&(imu_report_buf.MagneticField[0])),hmc5883_buf,6);
            hmc5883_flush_flag = 0;
            
            imu_report_buf.MagneticField[1] = -imu_report_buf.MagneticField[1];
            imu_report_buf.MagneticField[2] = -imu_report_buf.MagneticField[2];
        }
    }
    else/* second time */
    {
        imu_data *p_sample = &(imu_report_buf.Samples[1]);
        max21100_read_out((uint8_t *)(void *)(p_sample));

        change_tmp = p_sample->acce[0];
        p_sample->acce[0] = -p_sample->acce[1];
        p_sample->acce[1] = -change_tmp;
        p_sample->acce[2] = -p_sample->acce[2];


        change_tmp = p_sample->gyro[0];
        p_sample->gyro[0] = -p_sample->gyro[1];
        p_sample->gyro[1] = -change_tmp;
        p_sample->gyro[2] = -p_sample->gyro[2];
            
        imu_rescale(p_sample);    
        calibrate_apply(p_sample);
    
    }	
    return;
}

/**
  * @brief  Put report_buf to send stack 
  * @param  None
  * @retval None
  */
void imu_data_poll()
{
    if (systick_2ms_first_flag)
    {
        imu_data_process(IMU_FIRST);
        
        imu_report_buf.InterpupillaryDistance = InterpupillaryDistance;
        
        systick_2ms_first_flag = 0;
    }
    
    if (systick_2ms_second_flag)
    {
        imu_data_process(IMU_SECOND);				

        TX_push((uint8_t *)(void *)(&imu_report_buf),sizeof(imu_report_buf),0);	

        systick_2ms_second_flag = 0;
    }

}


/*****************************END OF FILE****/
