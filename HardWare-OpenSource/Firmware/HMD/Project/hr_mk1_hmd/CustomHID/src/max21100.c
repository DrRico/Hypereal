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
  * @file    fpga_coprocesser.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the SPI3 peripheral:
  *            + SPI3 Interface configuration
  *            + configure max21100
  *            + measure acce and gyro  
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "max21100.h"
/* Private define ------------------------------------------------------------*/
#define SPIX SPI3

/* Private macro -------------------------------------------------------------*/
#define MAX21100_CS_HIGH()      GPIO_SetBits(GPIOA,GPIO_Pin_15);
#define MAX21100_CS_LOW()       GPIO_ResetBits(GPIOA,GPIO_Pin_15);

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Configure delay time for SPI3 .
  * @param  None
  * @retval None
  */
void spi_delay(uint32_t nCount)
{
  while(nCount)
    nCount--;
}

/**
  * @brief  send and receive a byte by SPI3 .
  * @param  None
  * @retval None
  */
unsigned char SPIx_ReadWriteByte(unsigned char data)
{
  while (SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPIX, data); 
  while (SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_RXNE) == RESET); 
    return SPI_I2S_ReceiveData(SPIX); 
}

/**
  * @brief  Configure MAX21100 .
  * @param  None
  * @retval None
  */
void max21100_init()
{
  max21100_hardware_init();

  max21100_write_register(0x3f, 0xff); 
  
  max21100_write_register(0x22,0x00);
  max21100_write_register(0x00,0x7f);
  max21100_write_register(0x16,(0x01<<1)|0x01); //endian
  
  max21100_write_register(0x02,0x03); // gyro 1khz
  max21100_write_register(0x04,0x07); // set pwr_acc_cfg
  max21100_write_register(0x05,0x01); // acc 1khz
  
  
  max21100_write_register(0x3f,0xff); // RST_REG
  return;
}

/**
  * @brief  Configure SPI3 Interface configuration.
  * @param  None
  * @retval None
  */
void max21100_hardware_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
  
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI3, ENABLE );
  
  GPIO_SetBits(GPIOA,GPIO_Pin_15);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10|GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

	
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3); 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3); 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3); 
  
  SPI_Cmd(SPIX, DISABLE);
	
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
  SPI_InitStructure.SPI_CRCPolynomial = 7; 
  SPI_Init(SPIX, &SPI_InitStructure);  
 
  SPI_Cmd(SPIX, ENABLE); 
  
  return;
}

/**
  * @brief  Read a byte from max21100.
  * @param  None
  * @retval None
  */
unsigned char max21100_read_register(unsigned char addr)
{
  unsigned char data;
  
  MAX21100_CS_LOW();
  
  spi_delay(10);
  
  SPIx_ReadWriteByte(0x80 | addr);

  data = SPIx_ReadWriteByte(0x00);
  
  spi_delay(10);
  
  MAX21100_CS_HIGH();
  
  return data;
}

/**
  * @brief  Read Indefinite length byte from max21100.
  * @param  None
  * @retval None
  */
void max21100_read_register_n(unsigned char addr,unsigned char length,unsigned char *dist)
{
  MAX21100_CS_LOW();
  
  spi_delay(10);
  
  SPIx_ReadWriteByte(0x80 | addr);
  
  while(length)
  {
    dist[0] = SPIx_ReadWriteByte(0x00);
    dist++;
    length--;
  }
  
  spi_delay(10);
  
  MAX21100_CS_HIGH();

  return;
}

/**
  * @brief  Write a byte from max21100.
  * @param  None
  * @retval None
  */
void max21100_write_register(unsigned char addr,unsigned char data)
{
  MAX21100_CS_LOW();
  
  spi_delay(10);
  
  SPIx_ReadWriteByte(addr);
  
  spi_delay(10);
  
  SPIx_ReadWriteByte(data);
  
  spi_delay(10);
  
  MAX21100_CS_HIGH();
  
  return;
}

/**
  * @brief  measure acce and gyro from max21100.
  * @param  None
  * @retval None
  */
void max21100_read_out(unsigned char *dst)
{
    max21100_read_register_n(0x24, 12, dst);	// (gx_L,gx_H),(gy_L,gy_H),(gz_L,gz_H),(ax_L,ax_H),(ay_L,ay_H),(az_L,az_H)
    
    return;
}

/*****************************END OF FILE****/

