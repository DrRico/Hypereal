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
  * @file    led.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for led.c.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LED_H_
#define _LED_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported macro ------------------------------------------------------------*/
#define LED0(x) {if(x) GPIO_SetBits(GPIOB,GPIO_Pin_4); else GPIO_ResetBits(GPIOB,GPIO_Pin_4);}
#define LED1(x) {if(x) GPIO_SetBits(GPIOB,GPIO_Pin_5); else GPIO_ResetBits(GPIOB,GPIO_Pin_5);}

#define LED0_TOGGLE() {GPIO_ToggleBits(GPIOB,GPIO_Pin_4);}
#define LED1_TOGGLE() {GPIO_ToggleBits(GPIOB,GPIO_Pin_5);}

/* Exported functions ------------------------------------------------------- */
void led_init();

#endif