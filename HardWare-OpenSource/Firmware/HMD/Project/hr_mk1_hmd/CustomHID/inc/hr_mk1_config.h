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
  * @file    hr_mk1_config.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for system.
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HR_MK1_CONFIG_H_
#define _HR_MK1_CONFIG_H_

/* Includes ------------------------------------------------------------------*/
#include "usbd_customhid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "string.h"
#include "max21100.h"
#include "fpga_coprocesser.h"
#include "stm32f4xx_gpio.h"
#include "si4432.h"
#include "usbd_trans.h"
#include "imu_data.h"
#include "wireless_phase_control.h"
#include "led.h"			
#include "hmc5883l.h"

/* Exported constants --------------------------------------------------------*/
#define SYS_CLK                                 168000
#define WIRELESS_LOWPASS                        0x2B
    
#define MAIN_VERSION                            0X01
#define SUB_VERSION                             0X00

#define DEFAULT_WIRELESS_CHANNEL 		0


#define NV_ADDR_BASE	                        0X08040000
#define NV_BYTE0_WIRELESS_PHASE_LOWPASS	        (0X00 + NV_ADDR_BASE)
#define NV_BYTE1_WIRELESS_CHANNEL               (0X01 + NV_ADDR_BASE)
#define IMU_CALIB_DATA_ADDR                     (0X20 + NV_ADDR_BASE)

#define DFU_FLAG_ADDR                           0x08060000
#define DFU_FLAG_DATA                           0x55555555

#define USB_PID_HR                              0x5751
#define USB_VID_HR                              0x0484
#define USB_PRODUCT_STRING_HR                   "HR-MK1-HMD"

#endif /* _HR_MK1_CONFIG_H_ */

/*****************************END OF FILE****/