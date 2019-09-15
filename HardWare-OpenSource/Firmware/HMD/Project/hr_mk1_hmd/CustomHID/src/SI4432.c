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
  *            + SPI1 peripheral configuration
  *            + configure SI4432
  *            + transfer by SI4432 module 
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "si4432.h"

/* Private define ------------------------------------------------------------*/
#define SPIX 	        SPI1
#define USE_1HZ         0

/* Private macro -------------------------------------------------------------*/
#define SI4432_NIRQ 	(NRF_IRQ_GPIO->IDR & NRF_IRQ_PIN)
#define cs0()		GPIO_ResetBits(NRF_CSN_GPIO, NRF_CSN_PIN)
#define cs1()		GPIO_SetBits(NRF_CSN_GPIO, NRF_CSN_PIN)

/* Private variables ---------------------------------------------------------*/
extern uint8_t measure_ch;
extern uint8_t measure_busy_flag;
extern uint16_t measure_fix;
extern uint8_t exti3_flag;

uint8_t si4432_read_count = 0;

uint8_t ItStatus1,ItStatus2;
uint8_t buf_4432[light_sync_num];
static uint8_t latchedRssi = 0;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Configure delay time for SPI1 .
  * @param  None
  * @retval None
  */
void delay(uint32_t us){
	while(us--)asm("nop");
}

/**
  * @brief  send and receive a byte by SPI1 .
  * @param  None
  * @retval None
  */
u8 SPI_Byte(u8 da)
{
    while(SPI_I2S_GetFlagStatus(SPIX,SPI_I2S_FLAG_TXE)==RESET);
      SPI_SendData(SPIX,da);
    while(SPI_I2S_GetFlagStatus(SPIX,SPI_I2S_FLAG_RXNE)==RESET);
      return SPI_ReceiveData(SPIX);
}

/**
  * @brief  read a byte from SI4432 .
  * @param  None
  * @retval None
  */
u8  SI4432_ReadReg(u8  addr){
	u8 temp=0;
	cs0();
        
	SPI_Byte(addr);    
        
	temp=SPI_Byte(0Xff);
        
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
        
	SPI_Byte(addr|0x80); 
        
	SPI_Byte(value); 
        
	cs1();
}


/**
  * @brief  configure PA3 as SI4432 IRQ .
  * @param  None
  * @retval None
  */
void si4432_irq_init()
{
      GPIO_InitTypeDef  GPIO_InitStructure;
              
      GPIO_InitStructure.GPIO_Pin = NRF_IRQ_PIN;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
      GPIO_Init(NRF_IRQ_GPIO, &GPIO_InitStructure);
      
      EXTI_InitTypeDef EXTI_InitStructure;
      EXTI_InitStructure.EXTI_Line = EXTI_Line3;
      EXTI_InitStructure.EXTI_LineCmd = ENABLE;
      EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
      EXTI_Init(&EXTI_InitStructure);

      NVIC_InitTypeDef NVIC_InitStructure;

      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
      NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  configure SPI1 peripheral .
  * @param  None
  * @retval None
  */
void si4432_bsp_init()
{
      GPIO_InitTypeDef GPIO_InitStructure;
      SPI_InitTypeDef  SPI_InitStructure;
      
      GPIO_SetBits(NRF_CSN_GPIO,NRF_CSN_PIN);
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
      
      /* configure PA4 as SPI1 NSS */
      GPIO_InitStructure.GPIO_Pin =  NRF_CSN_PIN;
      GPIO_Init(NRF_CSN_GPIO, &GPIO_InitStructure);
      
      /* configure PB2 as SI4432 CE */ 
      GPIO_InitStructure.GPIO_Pin =  NRF_SD_PIN;
      GPIO_Init(NRF_SD_GPIO, &GPIO_InitStructure);
           
      RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );
      
      GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5|GPIO_Pin_6 | GPIO_Pin_7;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
      
      GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); 
      GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); 
      GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); 

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

      si4432_irq_init();         
      
      return;
}

/**
  * @brief  configure SI4432 module .
  * @param  None
  * @retval None
  */
void si4432_init()
{   
  int8_t ch;
 
  si4432_bsp_init();
  
  ch = DEFAULT_WIRELESS_CHANNEL;
  
  GPIO_SetBits(NRF_SD_GPIO, NRF_SD_PIN);
  delay(10000000);
  GPIO_ResetBits(NRF_SD_GPIO, NRF_SD_PIN);
  delay(10000000);
  
  ItStatus1 = SI4432_ReadReg(0x03);  
  ItStatus2 = SI4432_ReadReg(0x04);  	 
  
  /* Reset SI4432 */
  SI4432_WriteReg(0x07, 0x80); 
  
  while (SI4432_NIRQ);     
  
  do
  {
    SI4432_WriteReg(0x40, 0xaa);
    ItStatus1 = SI4432_ReadReg(0x40);  
  }
  while(ItStatus1!=0xaa);
  
  ItStatus1 = SI4432_ReadReg(0x03);  
  ItStatus2 = SI4432_ReadReg(0x04);  
  
  /* Set frequency 433M  */
  SI4432_WriteReg(0x75, 0x51);
  SI4432_WriteReg(0x76, 0x7D); 
  SI4432_WriteReg(0x77, 0x00);
  SI4432_WriteReg(0x79, ch*10);
  SI4432_WriteReg(0x7A, 16);    
  
  /* send rate */
  SI4432_WriteReg(0x6e, 0x41);
  SI4432_WriteReg(0x6f, 0x89);
  SI4432_WriteReg(0x70, 0x0c);  
  SI4432_WriteReg(0x58, 0xED);	

  SI4432_WriteReg(0x72, 0xCD);	        //frequency offset
  SI4432_WriteReg(0x71, 0x23);          //Disable CLK£¬FiFo £¬in FSK mode

  SI4432_WriteReg(0x1C, 0x8C);          //write 0x24 to the Clock Recovery Timing Loop Gain 0 register
  SI4432_WriteReg(0x20, 0x2f);          //write 0xD0 to the Clock Recovery Oversampling Ratio register
  SI4432_WriteReg(0x21, 0x02);          //write 0x00 to the Clock Recovery Offset 2 register
  SI4432_WriteReg(0x22, 0xbb);          //write 0x9D to the Clock Recovery Offset 1 register
  SI4432_WriteReg(0x23, 0x0d);          //write 0x49 to the Clock Recovery Offset 0 register
  SI4432_WriteReg(0x24, 0x07);          //write 0x00 to the Clock Recovery Timing Loop Gain 1 register
  SI4432_WriteReg(0x25, 0xff);          //write 0x0A to the AFC Timing Control register
  SI4432_WriteReg(0x1D, 0x40);          //write 0x40 to the AFC Loop Gearshift Override register
  SI4432_WriteReg(0x1E, 0x02);          //write 0x1E to the IF Filter Bandwidth register
  SI4432_WriteReg(0x2A, 0x50);	        //write 0x20 to the AFC Limiter register
  SI4432_WriteReg(0x1F, 0x03);	        //write 0x20 to the AFC Limiter register
  SI4432_WriteReg(0x69, 0x60);          //AGC overload 

  /* Preamble and synchronization word */
  SI4432_WriteReg(0x30, 0xAC);          //enable PH+ FIFO mode £¬enable CRC
  SI4432_WriteReg(0x32, 0x8C);          //disable frame head 
  SI4432_WriteReg(0x33, 0x00);  
  SI4432_WriteReg(0x34, 0X08);          // send 5 bytes as Preamble
  SI4432_WriteReg(0x35, 0x22);          // enable detect Preamble
  SI4432_WriteReg(0x36, ch*10+ch);      //synchronization word is 0x2dd4
  SI4432_WriteReg(0x37, ch*10+ch);

  /* set GPIO from SI4432 */
  SI4432_WriteReg(0x0b, 0x12); 
  SI4432_WriteReg(0x0c, 0x15); 

  /* set other function from SI4432 */ 
  SI4432_WriteReg(0x6d, 0x1e);

  SI4432_WriteReg(0x07, 0x05);
  /* open receive intertupt */
  SI4432_WriteReg(0x05, 0x02); 
  SI4432_WriteReg(0x06, 0x00);            //preamble detect -RSSI 
  /* clear interrupt */
  ItStatus1 = SI4432_ReadReg(0x03);	  //read the Interrupt Status1 register
  ItStatus2 = SI4432_ReadReg(0x04);	  //read the Interrupt Status2 register		
  SI4432_WriteReg(0x07, 0x05);            	
  
  SI4432_WriteReg(0x08, 0x12); 
  SI4432_WriteReg(0x08, 0x10);            //clear fifo
	
}   
  
/**
  * @brief  send data to SI4432 .
  * @param  None
  * @retval None
  */
void si4432_send(uint8_t* buff,uint8_t count)
{
    SI4432_WriteReg(0x3e, count);       // count = 8
    for(uint8_t i=0;i<count;i++)
            SI4432_WriteReg(0x7F, buff[i]);
    
    SI4432_WriteReg(0x06, 0x00);
    
    SI4432_ReadReg(0x03);               //clear interrupt	  
    SI4432_ReadReg(0x04);
    
    SI4432_WriteReg(0x07, 0x09);        //open send	
    while (SI4432_NIRQ);
    
    SI4432_ReadReg(0x03);  	
    SI4432_ReadReg(0x04);		 	
    
    SI4432_WriteReg(0x08, 0x12); 
    SI4432_WriteReg(0x08, 0x10);        //clear fifo
    SI4432_WriteReg(0x05, 0x02); 
    SI4432_WriteReg(0x07, 0x05);        //open receive	
}

/**
  * @brief  control SI4432 receive data .
  * @param  None
  * @retval None
  */
void si4432_startRx()
{
    SI4432_WriteReg(0x08, 0x12); 
    SI4432_WriteReg(0x08, 0x10);        //clear fifo
    SI4432_WriteReg(0x07, 0x05);        //open send	
    SI4432_WriteReg(0x05, 0x02); 
}

/**
  * @brief  get IRQ from SI4432 .
  * @param  None
  * @retval None
  */
uint8_t si4432_getIRQ()
{
    return SI4432_NIRQ;
}

/**
  * @brief  receive data from SI4432 .
  * @param  None
  * @retval None
  */
u8 si4432_read(uint8_t* buff, uint8_t* rssi)
{
    uint8_t SI4432_RxLenth=0;
  
    if(SI4432_NIRQ)
       return 0;
    if(rssi)
       *rssi=latchedRssi;
    
    ItStatus1 = SI4432_ReadReg(0x03);		//read the Interrupt Status1 register
    ItStatus2 = SI4432_ReadReg(0x04);		//read the Interrupt Status2 register
    if ((ItStatus1 & 0x02))                     //RXed
    {
        SI4432_RxLenth = SI4432_ReadReg(0x4B);	
        if (SI4432_RxLenth != SI4432_ReadReg(0x7F) /*|| (SI4432_RxLenth > 20)*/) {
                SI4432_WriteReg(0x08, 0x12);
                SI4432_WriteReg(0x08, 0x10);	 //clear fifo
                return 0;
        }
        for (uint8_t SI4432_RxCount=0;SI4432_RxCount < SI4432_RxLenth;SI4432_RxCount++){
                buff[SI4432_RxCount] = SI4432_ReadReg(0x7F);	
        }
        SI4432_WriteReg(0x08, 0x12);	
        SI4432_WriteReg(0x08, 0x10);
        SI4432_WriteReg(0x07, 0x05);            //open receive
    }
    else if(ItStatus2 &0x80)//rssi
    {
        latchedRssi=SI4432_ReadReg(0x26);
    }
    return SI4432_RxLenth;
}

/**
  * @brief  send data about control command .
  * @param  None
  * @retval None
  */
void si4432_send_no_wait(uint8_t* buff,uint8_t count)
{
    SI4432_WriteReg(0x3e, count);  
    for(uint8_t i=0;i<count;i++)
            SI4432_WriteReg(0x7F, buff[i]);
    
    SI4432_WriteReg(0x05, 0x00);	//generated interrupt 	
    SI4432_ReadReg(0x03);               //clear intertupt	  
    SI4432_ReadReg(0x04);
    
    SI4432_WriteReg(0x07, 0x09);        //open send	
    return;
}

/**
  * @brief  end send .
  * @param  None
  * @retval None
  */
void si4432_send_exit()
{
    SI4432_ReadReg(0x03);  	
    SI4432_ReadReg(0x04);		 	
    
    SI4432_WriteReg(0x08, 0x12); 
    SI4432_WriteReg(0x08, 0x10);        //clear fifo
    SI4432_WriteReg(0x05, 0x02); 
    SI4432_WriteReg(0x07, 0x05);        //open send	

    return;
}

/**
  * @brief  transmission of SI4432 .
  * @param  None
  * @retval None
  */
void si4432_poll()
{
    si4432_read_count = si4432_read(buf_4432,NULL);
    exti3_flag = 0;
    
    switch (si4432_read_count)
    {
      case 7:		//light sync
        if (!(buf_4432[0] & 0x80))              //lighthouse A
        {
              wireless_phase_control_run();
              
              wireless_phase_control_scan_num_push(buf_4432[6]);
              
              fpga_light_sync_package_process(buf_4432,&(light_sync_package[0]));
              
              if (!((measure_result_valid[0])&&(measure_result_valid[1])))
              {
                    light_sync_package[0].flush_flag = 0;
              }
              sync_report_routine();  
        }
        else                                   //lighthouse B
        {
              light_b_sync_flag = 1;
              
              fpga_light_sync_package_process(buf_4432,&(light_sync_package[1]));

              if (!((measure_result_valid[2])&&(measure_result_valid[3])))
              {
                  light_sync_package[1].flush_flag = 0;
              }

        } 
        
        lighthouse_version_report_flush();//µÆËþÍ¬²½°ü
        
        break;
    default:
        asm("nop");
        break;
    }
}

/*****************************END OF FILE****/
