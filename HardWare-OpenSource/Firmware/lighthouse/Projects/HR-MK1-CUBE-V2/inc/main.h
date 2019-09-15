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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported constants --------------------------------------------------------*/
#define SYS_CLK                                         72000000 / 1000
#define WIRELESS_CHANNEL				0
#define MAX_CHANNEL_NUM					32
#define IS_SLAVE						0

#define DEBUG_MOTOR_FIX_PERIOD_TEST		        0
#define DEBUG_WIRELESS_NOISE_SOURCE		        0

#define PWM_PERIOD					40000
#define TIM1_PERIOD					36
#define TIM2_PERIOD					60000
#define TIM2_UPDATE_COUNT				4
#define MOTOR_TARGET_FREQ 				60
#define WIRELESS_NOT_SYNC_THRESHOLD		        10
#define MOTOR_LOCK_THR					50

/* Exported functions ------------------------------------------------------- */
static const uint32_t VER = 0x31;

extern GPIO_TypeDef* GPIO_LED[2];
extern const uint16_t PIN_LED[2];
extern float SCAN_PHASE_OFFSET[2] ;
extern uint8_t tim2UpdateCount;

/** bsp_init.c **/
void gpioInit();
void tim1Init();
void tim2Init();
void tim3Init();
void tim4Init();
void systickInit();
void uart1Init();

/** main_df.c **/
void onPeriodUpdate(int p1, int p2, uint32_t c1, uint32_t c2, uint32_t c4);
void sendSyncPackage(int8_t dphase1, int8_t dphase2,int16_t phaseOffset[]);

#endif /* __MAIN_H__ */

/****END OF FILE****/
