/*
* The MIT License (MIT)
* Copyright (c) 2017 Shanghai Chai Ming Huang Info&Tech Co，Ltd
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
  * @file    usb_trans.h
  * @author  Hypereal Team
  * @version V2.2.0
  * @date    19-November-2015
  * @brief   Header for usb_trans.c.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _USBD_TRANS_H_
#define _USBD_TRANS_H_

/* Includes ------------------------------------------------------------------*/
#include  "usbd_customhid_core.h"
#include  "usbd_usr.h"
#include  "usbd_desc.h"
#include  "usb_bsp.h"
#include  "string.h"
#include  "stm32f4xx_flash.h"
#include  "imu_data.h"
/* Exported constants --------------------------------------------------------*/ 
#define USB_START       0
#define USB_START1      1
#define USB_LEN         2
#define USB_LEN1        3
#define USB_TYPE        4
#define USB_TYPE1       5
#define USB_BODY        6
#define USB_CMD         6
#define temp_rage       256     //数据堆栈大小

/* Exported types ------------------------------------------------------------*/
#pragma pack (1)
typedef struct
{
    int8_t START; 
    int8_t START1;
    int16_t LEN;                            
    int16_t TYPE;
    int8_t  BODY[512];
} USB_DATA;

typedef struct {
		uint8_t lighthouse_id;
		uint8_t axis_sync;
		uint8_t version;
		uint32_t error;
} lighthouse_version_report_type;

extern unsigned int systick_flag ;

uint8_t TX_push(uint8_t *buf,uint16_t len,uint16_t type);
uint16_t trans_RX(uint8_t *buf);
uint16_t deal_RX(void);
void lighthouse_version_report_flush(void);
void sync_report_routine(void);
void trans_poll(void); 
void LV_poll(void);
  
#endif /*__USBD_TRANS_H__*/

/*****************************END OF FILE****/