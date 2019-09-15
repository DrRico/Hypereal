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
  * @file    wireless_phase_control.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for wireless_phase_control.c.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _WIRELESS_PHASE_CONTROL_
#define _WIRELESS_PHASE_CONTROL_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported constants --------------------------------------------------------*/
#define WP_PHASE_STAGE_NUM              11
#define WP_IDLE				0
#define WP_LIGHT_A			1
#define WP_HUB_TX_0			2
#define WP_REMOTE_A_A		        3
#define WP_HUB_RX_0			4
#define WP_REMOTE_B_A		        5
#define WP_LIGHT_B			6
#define WP_HUB_TX_1			7
#define WP_REMOTE_A_B		        8
#define WP_HUB_RX_1			9
#define WP_REMOTE_B_B		        10

/* Exported macro ------------------------------------------------------------*/
#define WP_TIME_LINE {5,10,10,70,10,80,10,10,70,10,75}

#define WP_TIME_LINE_SCALE ((((168.0/16)/30)/360)*1000000)//定时器168M时钟下16分频

#define WP_SYNC_DELAY  ((uint16_t)(8.64*WP_TIME_LINE_SCALE))//8.64对应灯塔同步包的空中时间

#define HZ_30_TIMMER_DEFAULT_ARR (43740)

/* Private variables ---------------------------------------------------------*/
extern uint8_t currunt_wireless_phase;
extern uint8_t wireless_phase_flag;
extern void (*wp_callback[WP_PHASE_STAGE_NUM])();

extern signed int tim5_diff;

extern uint16_t scan_num;

extern uint8_t light_a_sync_flag;
extern uint8_t light_b_sync_flag;
extern uint8_t light_a_sync_flag_last;
extern uint8_t light_b_sync_flag_last;

void wireless_phase_control_init();
void wireless_phase_control_sync();	//在无线中断后立即调用
void wireless_phase_control_run();
void wireless_phase_control_run_from_light_b();

void wireless_phase_control_scan_num_push(uint8_t num_in);

void hz_30_timmer_sync();

#endif /* _WIRELESS_PHASE_CONTROL_ */

/*****************************END OF FILE****/