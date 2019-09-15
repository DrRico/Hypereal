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
  * @file    hmc5883l.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for hmc5883.c.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HMC5883L_H
#define _HMC5883L_H

/* Includes ------------------------------------------------------------------*/
#include  "stm32f4xx.h"
#include  "usbd_customhid_core.h"
#include  "usbd_usr.h"
#include  "usbd_desc.h"
#include  "string.h"
#include  "usbd_trans.h"
#include  "hr_mk1_config.h"
    
/* Exported constants --------------------------------------------------------*/
#define I2Cx_Rx_len                 6           // (mx_L,mx_H),(my_L,my_H),(mz_L,mz_H)
#define I2Cx_Tx_len                 2           //addr,data
    
/* Exported macro ------------------------------------------------------------*/
#define HMC5883_I2Cx                I2C2
#define	SlaveAddress                0x3C        //address for HMC5883
#define Ownaddr                     0x30        //MCU address
#define hmc5883l_speed              40000       // I2C ClockSpeed   
/**
  * @brief  I2C DMA Interface 
  */  

#define I2C_DMA                      DMA1   
#define I2C_DMA_CHANNEL              DMA_Channel_7
#define I2C_DMA_STREAM_TX            DMA1_Stream7
#define I2C_DMA_STREAM_RX            DMA1_Stream3   
#define I2C_DMA_CLK                  RCC_AHB1Periph_DMA1
#define I2C_DR_Address               (uint32_t)0x40005810//I2C1((uint32_t)0x40005410)
#define USE_DMA
   
#define I2C_DMA_TX_IRQn              DMA1_Stream7_IRQn
#define I2C_DMA_RX_IRQn              DMA1_Stream3_IRQn
#define I2C_DMA_TX_IRQHandler        DMA1_Stream7_IRQHandler
#define I2C_DMA_RX_IRQHandler        DMA1_Stream3_IRQHandler   
#define I2C_DMA_PREPRIO              0
#define I2C_DMA_SUBPRIO              0   
   
#define I2C_TX_DMA_FLAG_FEIF         DMA_FLAG_FEIF7
#define I2C_TX_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF7
#define I2C_TX_DMA_FLAG_TEIF         DMA_FLAG_TEIF7
#define I2C_TX_DMA_FLAG_HTIF         DMA_FLAG_HTIF7
#define I2C_TX_DMA_FLAG_TCIF         DMA_FLAG_TCIF7
#define I2C_RX_DMA_FLAG_FEIF         DMA_FLAG_FEIF3
#define I2C_RX_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF3
#define I2C_RX_DMA_FLAG_TEIF         DMA_FLAG_TEIF3
#define I2C_RX_DMA_FLAG_HTIF         DMA_FLAG_HTIF3
#define I2C_RX_DMA_FLAG_TCIF         DMA_FLAG_TCIF3

#define I2C_DMA_IT_TX_TC             DMA_IT_TCIF7
#define I2C_DMA_IT_RX_TC             DMA_IT_TCIF3
   
#define I2C_DIRECTION_TX             0
#define I2C_DIRECTION_RX             1 

/* Exported functions ------------------------------------------------------- */
extern uint8_t hmc5883_buf[];
extern uint8_t hmc5883_busy;
extern uint8_t hmc5883_flush_flag;

void wp_callback_hmc5883();
void wp_callback_hmc5883_ptr_flush();

void hmc5883_poll();
void HMC5883L_init(void);

uint8_t Multiple_Read_HMC5883L(void);
uint8_t HMC5883_TimeoutUserCallback(void);
uint8_t Single_WriteI2C(u8 REG_Address,u8 REG_data);

#endif  /* _HMC5883L_H_ */

/*****************************END OF FILE****/
