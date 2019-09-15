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
  * @file    SI4432.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the SPI1 peripheral:
  *            + SPI1 Interface configuration
  *            + configure SI4432
  *            + transfer by SI4432 module 
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "si4432.h"

/* Private macro -------------------------------------------------------------*/
#define SI4432_NIRQ 	(NRF_IRQ_GPIO->IDR & NRF_IRQ_PIN)
#define cs0()		GPIO_ResetBits(NRF_CSN_GPIO, NRF_CSN_PIN)
#define cs1()		GPIO_SetBits(NRF_CSN_GPIO, NRF_CSN_PIN)

/* Private variables ---------------------------------------------------------*/
u8 ItStatus1, ItStatus2;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Configure delay time for SPI1 .
  * @param  None
  * @retval None
  */
static void delay(uint32_t us) 
{
      while (us--)
              asm("nop");
}

/**
  * @brief  send and receive a byte by SPI2 .
  * @param  None
  * @retval None
  */
u8 SPI_Byte(u8 da) 
{
      while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
      SPI2->DR = da;
      while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
      return SPI2->DR;
}

/**
  * @brief  read a byte from SI4432 .
  * @param  None
  * @retval None
  */
u8 SI4432_ReadReg(u8 addr) 
{
	u8 temp = 0;
        
	cs0();

	SPI_Byte(addr);    
	temp = SPI_Byte(0Xff);

	cs1();
        
	return temp;
}

/**
  * @brief  write a byte from SI4432 .
  * @param  None
  * @retval None
  */
void SI4432_WriteReg(u8 addr, u8 value) 
{
	cs0();
        
	SPI_Byte(addr | 0x80);
	SPI_Byte(value);
        
	cs1();
}

/**
  * @brief  configure GPIO peripheral of SI4432 .
  * @param  None
  * @retval None
  */
static void gpioInit(uint8_t isOut, uint8_t isAF, uint16_t pin,GPIO_TypeDef* gpio) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = pin;

	if (isOut)
		GPIO_InitStructure.GPIO_Mode =isAF ? GPIO_Mode_AF_PP : GPIO_Mode_Out_PP;
	else
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(gpio, &GPIO_InitStructure);
}

/**
  * @brief  configure SPI2 peripheral .
  * @param  None
  * @retval None
  */
void si4432_bsp_init() 
{
	SPI_InitTypeDef SPI_InitStructure;

	GPIO_SetBits(NRF_CSN_GPIO, NRF_CSN_PIN);
	gpioInit(1, 0, NRF_CSN_PIN, NRF_CSN_GPIO);
	gpioInit(1, 0, NRF_SD_PIN, NRF_SD_GPIO);
	gpioInit(0, 0, NRF_IRQ_PIN, NRF_IRQ_GPIO);	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	gpioInit(1, 1, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIOB);

	SPI_Cmd(SPI2, DISABLE);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI2, &SPI_InitStructure);  

	SPI_Cmd(SPI2, ENABLE); 
	return;
}

/**
  * @brief  configure SI4432 module .
  * @param  None
  * @retval None
  */
void si4432_init(int8_t ch) 
{
	si4432_bsp_init();
        
	GPIO_SetBits(NRF_SD_GPIO, NRF_SD_PIN);
	delay(1000000);
	GPIO_ResetBits(NRF_SD_GPIO, NRF_SD_PIN);
	delay(3000000);

	ItStatus1 = SI4432_ReadReg(0x03);
	ItStatus2 = SI4432_ReadReg(0x04);
        /* Reset SI4432 */
	SI4432_WriteReg(0x07, 0x80); 
        
	while (SI4432_NIRQ);
	do 
        {
            SI4432_WriteReg(0x40, 0xaa);
            ItStatus1 = SI4432_ReadReg(0x40);
	} while (ItStatus1 != 0xaa);

	ItStatus1 = SI4432_ReadReg(0x03);
	ItStatus2 = SI4432_ReadReg(0x04);

        /* Set frequency 433M  */		
	SI4432_WriteReg(0x75, 0x51);
	SI4432_WriteReg(0x76, 0x7D);  
	SI4432_WriteReg(0x77, 0x00);
	SI4432_WriteReg(0x79, ch*10);
	SI4432_WriteReg(0x7A, 16);

	SI4432_WriteReg(0x6e, 0x41);
	SI4432_WriteReg(0x6f, 0x89);
	SI4432_WriteReg(0x70, 0x0c);  
	SI4432_WriteReg(0x58, 0xED);	

	SI4432_WriteReg(0x72, 0xCD);	                //frequency offset
	SI4432_WriteReg(0x71, 0x23);                    //Disable CLK£¬FiFo £¬in FSK mode

	SI4432_WriteReg(0x1C, 0x8C);                    //write 0x24 to the Clock Recovery Timing Loop Gain 0 register
	SI4432_WriteReg(0x20, 0x2f);                    //write 0xD0 to the Clock Recovery Oversampling Ratio register
	SI4432_WriteReg(0x21, 0x02);                    //write 0x00 to the Clock Recovery Offset 2 register
	SI4432_WriteReg(0x22, 0xbb);                    //write 0x9D to the Clock Recovery Offset 1 register
	SI4432_WriteReg(0x23, 0x0d);                    //write 0x49 to the Clock Recovery Offset 0 register
	SI4432_WriteReg(0x24, 0x07);                    //write 0x00 to the Clock Recovery Timing Loop Gain 1 register
	SI4432_WriteReg(0x25, 0xff);                    //write 0x0A to the AFC Timing Control register
	SI4432_WriteReg(0x1D, 0x40);                    //write 0x40 to the AFC Loop Gearshift Override register
	SI4432_WriteReg(0x1E, 0x02);                    //write 0x1E to the IF Filter Bandwidth register
	SI4432_WriteReg(0x2A, 0x50);	                //write 0x20 to the AFC Limiter register
	SI4432_WriteReg(0x1F, 0x03);	                //write 0x20 to the AFC Limiter register
	SI4432_WriteReg(0x69, 0x60);                    //AGC overload	 

	/* Preamble and synchronization word */
	SI4432_WriteReg(0x30, 0xAC);                    //enable PH+ FIFO mode £¬enable CRC
	SI4432_WriteReg(0x32, 0x8C);                    //disable frame head
	SI4432_WriteReg(0x33, 0x00);   
	SI4432_WriteReg(0x34, 0X08);                    // send 5 bytes as Preamble
	SI4432_WriteReg(0x35, 0x22);                    // enable detect Preamble
	SI4432_WriteReg(0x36, ch*10+ch);                // Í¬²½×ÖÎª 0x2dd4
	SI4432_WriteReg(0x37, ch*10+ch);
        
        /* set GPIO from SI4432 */
	SI4432_WriteReg(0x0b, 0x12); 
	SI4432_WriteReg(0x0c, 0x15); 
	SI4432_WriteReg(0x0d, 0xc7); 
        /* set other function from SI4432 */ 
	SI4432_WriteReg(0x6d, 0x1e);	
	
	SI4432_WriteReg(0x07, 0x05);
	SI4432_WriteReg(0x0a, 0x02); 
	/* open receive intertupt */
	SI4432_WriteReg(0x05, 0x02);
	SI4432_WriteReg(0x06, 0x00);	                //preamble detect -RSSI 
	/* clear interrupt */
	ItStatus1 = SI4432_ReadReg(0x03);	        //read the Interrupt Status1 register
	ItStatus2 = SI4432_ReadReg(0x04);               //read the Interrupt Status2 register
	SI4432_WriteReg(0x07, 0x05);	          

	SI4432_WriteReg(0x08, 0x12);
	SI4432_WriteReg(0x08, 0x10);	                //clear fifo

}
uint16_t c = 0;
void si4432_send(u8* buff, u8 count) 
{
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
        
	SI4432_WriteReg(0x3e, count);                   // count = 8
	for (u8 i = 0; i < count; i++)
		SI4432_WriteReg(0x7F, buff[i]);

	SI4432_WriteReg(0x05, 0x04);	                	
	SI4432_WriteReg(0x06, 0x00);

	SI4432_ReadReg(0x03);	                        //clear interrupt
	SI4432_ReadReg(0x04);

	SI4432_WriteReg(0x07, 0x09);	                //open send
	while (SI4432_NIRQ);

	SI4432_ReadReg(0x03);
	SI4432_ReadReg(0x04);

	SI4432_WriteReg(0x08, 0x12);
	SI4432_WriteReg(0x08, 0x10);	                //clear fifo
	SI4432_WriteReg(0x05, 0x02);
	SI4432_WriteReg(0x07, 0x05);	                //open receive
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/**
  * @brief  control SI4432 receive data .
  * @param  None
  * @retval None
  */
void si4432_startRx() 
{
	SI4432_WriteReg(0x08, 0x12);
	SI4432_WriteReg(0x08, 0x10);	                //clear fifo
	SI4432_WriteReg(0x07, 0x05);	                //open send
	SI4432_WriteReg(0x05, 0x02);
}

/**
  * @brief  get IRQ from SI4432 .
  * @param  None
  * @retval None
  */
u8 si4432_getIRQ() 
{
	return SI4432_NIRQ;
}

/**
  * @brief  receive data from SI4432 .
  * @param  None
  * @retval None
  */
u8 si4432_read(u8* buff) 
{
    u8 SI4432_RxLenth = 0;
	
        if (SI4432_NIRQ)
		return 0;
	ItStatus1 = SI4432_ReadReg(0x03);	        //read the Interrupt Status1 register
	ItStatus2 = SI4432_ReadReg(0x04);	        //read the Interrupt Status2 register
	if ((ItStatus1 & 0x02))                         //RXed
        {		
		SI4432_RxLenth = SI4432_ReadReg(0x4B);
		if (SI4432_RxLenth != SI4432_ReadReg(0x7F) || SI4432_RxLenth > 20) 
                {
			SI4432_WriteReg(0x08, 0x12);
			SI4432_WriteReg(0x08, 0x10);	  
			return 0;
		}
		for (u8 SI4432_RxCount = 0; SI4432_RxCount < SI4432_RxLenth;
				SI4432_RxCount++) 
                {
			buff[SI4432_RxCount] = SI4432_ReadReg(0x7F);
		}
		SI4432_WriteReg(0x08, 0x12);
		SI4432_WriteReg(0x08, 0x10);
		SI4432_WriteReg(0x07, 0x05); 
	}
	return SI4432_RxLenth;
}

/****END OF FILE****/

