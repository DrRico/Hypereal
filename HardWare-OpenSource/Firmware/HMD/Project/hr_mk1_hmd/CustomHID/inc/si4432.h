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
  * @file    SI4432.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for SI4432.c.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SI4432_H__
#define __SI4432_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "hr_mk1_config.h"
#include "usbd_trans.h"
/* Exported constants --------------------------------------------------------*/
#define light_sync_num          256

/* Exported macro ------------------------------------------------------------*/
#define NRF_CSN_GPIO	        GPIOA
#define	NRF_IRQ_GPIO	        GPIOA
#define NRF_SD_GPIO		GPIOB

#define NRF_CSN_PIN		GPIO_Pin_4
#define NRF_IRQ_PIN		GPIO_Pin_3
#define NRF_SD_PIN		GPIO_Pin_2

#define SI4432_NIRQ 	        (NRF_IRQ_GPIO->IDR & NRF_IRQ_PIN)

#define sd0()			NRF_SD_GPIO->BRR = NRF_SD_PIN
#define sd1()			NRF_SD_GPIO->BSRR = NRF_SD_PIN

/* Exported functions ------------------------------------------------------- */
extern uint8_t buf_4432[light_sync_num];

void si4432_init();
uint8_t si4432_read(uint8_t* buff, uint8_t *rssi);
void si4432_send(uint8_t* buff,uint8_t count);

void si4432_send_no_wait(uint8_t* buff,uint8_t count);
void si4432_poll();

#endif /* _SI4432_H_ */

/*****************************END OF FILE****/
