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
  * @file    hmc5883l.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the I2C peripheral:
  *            + I2C Interface configuration
  *            + configure hmc5883l
  *            + measure Magnetic 
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include  "hmc5883l.h"
    
/* Private variables ---------------------------------------------------------*/
static uint8_t ptr_reset_flag = 0;

uint32_t HMC5883_Timeout = 0xffffffff;
uint8_t  HMC5883_RX_BUF[I2Cx_Rx_len];
uint8_t  HMC5883_TX_BUF[I2Cx_Tx_len];
uint8_t  hmc5883_buf[I2Cx_Rx_len];

uint8_t hmc5883_busy = 0;
uint8_t hmc5883_flush_flag = 0;

uint8_t hmc5883_flush_request = 0;
uint8_t hmc5883_ptr_flush_request = 0;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Configure Delay(us).
  * @param  None
  * @retval None
  */
void delay_us(unsigned int t)
{
    unsigned int i;
    
    while(t--)
    {
      for (i=0;i<5;i++)
        asm("nop");
    }  
}

/**
  * @brief  Generate Start
  * @param  None
  * @retval None
  */
uint8_t I2C_Start()
{
    I2C_GenerateSTART(HMC5883_I2Cx, ENABLE);
    while (!I2C_CheckEvent(HMC5883_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    
    return 0;
}

/**
  * @brief  Generate Stop
  * @param  None
  * @retval None
  */
void I2C_Stop()
{
    I2C_GenerateSTOP(HMC5883_I2Cx, ENABLE);              
}

/**
  * @brief  I2C write a byte
  * @param  None
  * @retval None
  */
uint8_t Single_WriteI2C(uint8_t REG_Address,uint8_t REG_data)
{
    HMC5883_TX_BUF[0] = REG_Address;
    HMC5883_TX_BUF[1] = REG_data; 
    
    while (I2C_GetFlagStatus(HMC5883_I2Cx, I2C_FLAG_BUSY));
  
    /* Enable DMA TX Channel */
    DMA_Cmd(I2C_DMA_STREAM_TX, ENABLE);
  
    I2C_Start();     
  
    I2C_Send7bitAddress(HMC5883_I2Cx,SlaveAddress, I2C_Direction_Transmitter); 
    while (!I2C_CheckEvent(HMC5883_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));     

    return 0;
}

/**
  * @brief  set range of mag
  * @param  None
  * @retval None
  */
uint8_t HMC5883L_Range_Set(void)
{   
    Single_WriteI2C(0x00,0x14);
    asm("nop");
    Single_WriteI2C(0x01,0x60);  //range 2.5Ga
    asm("nop");
    Single_WriteI2C(0x02,0x01);  
    asm("nop");
    
    return 0;
        
}

/**
  * @brief  Configure Delay(us).
  * @param  None
  * @retval None
  */
uint8_t Multiple_Read_HMC5883L(void)
{   
    while (I2C_GetFlagStatus(HMC5883_I2Cx, I2C_FLAG_BUSY));    
  
    /* Enable DMA RX Channel */
    DMA_Cmd(I2C_DMA_STREAM_RX, ENABLE); 
    /* Enable I2C ACK */
    I2C_AcknowledgeConfig(HMC5883_I2Cx, ENABLE);
      
    I2C_Start();
    //EV5
    I2C_Send7bitAddress(HMC5883_I2Cx,SlaveAddress+1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(HMC5883_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    return 0;
}

/**
  * @brief  Set wireless callback flag of hmc5883 receive
  * @param  None
  * @retval None
  */
void wp_callback_hmc5883()
{
    hmc5883_flush_request = 1;
}

/**
  * @brief  Set wireless callback flag of hmc5883 transfer
  * @param  None
  * @retval None
  */
void wp_callback_hmc5883_ptr_flush()
{		
    hmc5883_ptr_flush_request = 1;	
}

/**
  * @brief  The board sends a byte and expects to receive 6 bytes back. 
  * @param  None
  * @retval None
  */
void hmc5883_poll()
{
    if (hmc5883_flush_request == 1)
    {
        if (ptr_reset_flag)
        {		
            hmc5883_busy = 1;
            Multiple_Read_HMC5883L();
            ptr_reset_flag = 0;			
            hmc5883_flush_request = 0;
        }
    }

    if (hmc5883_ptr_flush_request == 1)
    {
            hmc5883_busy = 1;
            Single_WriteI2C(0x02,0x01);
            ptr_reset_flag = 1;				
            hmc5883_ptr_flush_request = 0;
    }
}

/**
  * @brief  Configure the DMA Peripheral used to handle communication via I2C.
  * @param  None
  * @retval None
  */
static void HMC5883_DMA_Config(uint8_t Direction, uint8_t* buffer)
{
    DMA_InitTypeDef     DMA_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;

    /* Enable DMA1 clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* Initialize the DMA_Channel member */
    DMA_InitStructure.DMA_Channel = I2C_DMA_CHANNEL;

    /* Initialize the DMA_PeripheralBaseAddr member */
    DMA_InitStructure.DMA_PeripheralBaseAddr = I2C_DR_Address;

    /* Initialize the DMA_Memory0BaseAddr member */
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;

    /* Initialize the DMA_PeripheralInc member */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    /* Initialize the DMA_MemoryInc member */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /* Initialize the DMA_PeripheralDataSize member */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    /* Initialize the DMA_MemoryDataSize member */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    /* Initialize the DMA_Mode member */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

    /* Initialize the DMA_Priority member */
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

    /* Initialize the DMA_FIFOMode member */
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;

    /* Initialize the DMA_FIFOThreshold member */
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;

    /* Initialize the DMA_MemoryBurst member */
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

    /* Initialize the DMA_PeripheralBurst member */
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    /* If using DMA for Reception */
    if (Direction == I2C_DIRECTION_RX)
    {    
      /* Initialize the DMA_DIR member */
      DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
      
      /* Initialize the DMA_BufferSize member */
      DMA_InitStructure.DMA_BufferSize = I2Cx_Rx_len;
      
      DMA_DeInit(I2C_DMA_STREAM_RX);
      
      /* Initialize the DMA stream */
      DMA_Init(I2C_DMA_STREAM_RX, &DMA_InitStructure);
      
       /* Configure and enable I2C DMA TX Stream interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);      
      DMA_ITConfig(I2C_DMA_STREAM_RX, DMA_IT_TC, ENABLE); 
    }
    /* If using DMA for Transmission */
    else if (Direction == I2C_DIRECTION_TX)
    { 
      /* Initialize the DMA_DIR member */
      DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
      
      /* Initialize the DMA_BufferSize member */
      DMA_InitStructure.DMA_BufferSize = I2Cx_Tx_len;
      
      DMA_DeInit(I2C_DMA_STREAM_TX);
      
      /* Initialize the DMA stream */
      DMA_Init(I2C_DMA_STREAM_TX, &DMA_InitStructure);
      
      /* Configure and enable I2C DMA RX Stream interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
      NVIC_Init(&NVIC_InitStructure);  
      DMA_ITConfig(I2C_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
    }
}

/**
  * @brief  Configure the I2C2 peripheral for hmc5883 
  * @param  None
  * @retval None
  */
void HMC5883L_init()
{
   	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

        /*!< I2C_SCL_GPIO_CLK and I2C_SDA_GPIO_CLK Periph clock enable */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        
        /*!< I2C Periph clock enable */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
       
        /*!< GPIO configuration */
        /* Connect PXx to I2C_SCL*/
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
        /* Connect PXx to I2C_SDA*/
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);  

	/* Configure IO connected to IIC */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
        /* Configure DMA Peripheral */
        HMC5883_DMA_Config(I2C_DIRECTION_TX, (uint8_t*)HMC5883_TX_BUF);       
        HMC5883_DMA_Config(I2C_DIRECTION_RX, (uint8_t*)HMC5883_RX_BUF);        
        
        /* Configure the I2C peripheral */
        I2C_DeInit(HMC5883_I2Cx);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = Ownaddr;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = hmc5883l_speed;

	I2C_Cmd(HMC5883_I2Cx, ENABLE);
        
	I2C_Init(HMC5883_I2Cx, &I2C_InitStructure);
        
	I2C_AcknowledgeConfig(HMC5883_I2Cx, ENABLE);
        
        I2C_DMACmd(HMC5883_I2Cx,ENABLE);
        
        /* set hmc range */
        HMC5883L_Range_Set();
             
}

/**
  * @brief  Reconfigure the I2C.
  * @param  None
  * @retval None
  */
uint8_t HMC5883_TimeoutUserCallback(void)
{
  I2C_InitTypeDef I2C_InitStructure;
  
  /* IOE_I2C peripheral configuration */
  I2C_DeInit(HMC5883_I2Cx);
  
  
  I2C_GenerateSTOP(HMC5883_I2Cx, ENABLE);
  I2C_SoftwareResetCmd(HMC5883_I2Cx, ENABLE);
  I2C_SoftwareResetCmd(HMC5883_I2Cx, DISABLE);

  /* Configure the I2C peripheral */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = Ownaddr;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = hmc5883l_speed;
  
  /* Enable the I2C peripheral */
  I2C_Cmd(HMC5883_I2Cx, ENABLE);  
  I2C_Init(HMC5883_I2Cx, &I2C_InitStructure);
  
  return 1;
}

/*****************************END OF FILE****/
