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
  * @file    usbd_trans.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides firmware functions to USB transformation.
  ******************************************************************************

  */ 

/* Includes ------------------------------------------------------------------*/ 
#include  "usbd_trans.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USB_MAX_LEN                 64
#define AGRE_MAX_LEN                512
/* Private macro -------------------------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;

/* Private variables ---------------------------------------------------------*/
uint8_t bulk_rx_flag = 0;

uint16_t TX_length=0;                   //usb_trans length
uint16_t RX_length=0;                   //usb_receive length

uint8_t  buf_temp[temp_rage][USB_MAX_LEN];//send buf stack
uint8_t  save_cnt=0;                     //stack receive location
uint8_t  send_cnt=0;                     //stack send location
uint16_t push_cnt=0;                    //stack overflow

uint8_t  bulk_rx_buf[USB_MAX_LEN];
uint8_t  bulk_rx_count;

uint8_t  PrevXferDone = 1;

uint8_t  bulk_tx_packet[AGRE_MAX_LEN];     //tx_buf
uint8_t  bulk_rx_packet[AGRE_MAX_LEN];     //rx_buf     
uint8_t  bulk_rx_packet_len = 0;

lighthouse_version_report_type lighthouse_version_report[2];
unsigned char cube_flush_flag[2] = {0,0};

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Fight the transmit frame.
  * @param  None
  * @retval None
  */
uint8_t TX_push(uint8_t *buf,uint16_t len,uint16_t type)//将发送数据送入发送堆栈
{
      uint16_t i,j;
      uint8_t  chksum=0,temp_cnt=0;
      USB_DATA tx_push_buf;
      
      if(push_cnt >= temp_rage)
        return 1;
      else
      {
        temp_cnt = (len + 9 - 1) / 64 + 1;
        if((push_cnt + temp_cnt) >= temp_rage)
          return 1;
        else
        {
          memcpy(tx_push_buf.BODY,buf,len);
          tx_push_buf.START  = 0x2B;
          tx_push_buf.START1 = 0x2B;
          tx_push_buf.LEN    = len;
          tx_push_buf.TYPE   = type;
          
          chksum=0;
          for(i = 0;i < len;i++)
            chksum = chksum + *(tx_push_buf.BODY + i);
          *(tx_push_buf.BODY + len)=chksum;
          
          *(tx_push_buf.BODY + len + 1)=0xB2;
          *(tx_push_buf.BODY + len + 2)=0xB2;   
          
          for(i = 0;i < temp_cnt;i++)
          { 
            for(j = 0;j < USB_MAX_LEN;j++)
              buf_temp[save_cnt][j]=*(&tx_push_buf.START + i*USB_MAX_LEN + j);
            save_cnt++;
            if(save_cnt >= temp_rage -1)
              save_cnt=0;
            push_cnt++;
          }
          return 0;
        }        
      }     
}

/**
  * @brief  push lighthouse version to USB stack
  * @param  None
  * @retval None
  */
void lighthouse_version_report_flush()
{
	signed char comp_x = buf_4432[0]<<1;
	signed char comp_y = buf_4432[1]<<1;

	if (comp_x<0) comp_x = -comp_x;
	if (comp_y<0) comp_y = -comp_y;
	
	uint8_t id = buf_4432[0] >> 7;
	lighthouse_version_report[id].lighthouse_id = id;
	lighthouse_version_report[id].version = ((buf_4432[5]>>4)) + (((uint16_t)(buf_4432[5] & 0x0f))<<8);
	lighthouse_version_report[id].error = 0;
	
	if (comp_x<120 && comp_y<120)
	{
		lighthouse_version_report[id].axis_sync = 1;
	}
	else
	{
		lighthouse_version_report[id].axis_sync = 0;
	}
	
	cube_flush_flag[id] = 1;
}

/**
  * @brief  push sync report to USB stack
  * @param  None
  * @retval None
  */
void sync_report_routine()
{
	uint8_t send_buf[8];
	
	send_buf[0] = scan_num;
	send_buf[1] = scan_num>>8;
	TX_push(send_buf,2,6);
}

/**
  * @brief  read data about IMU offset.
  * @param  None
  * @retval None
  */
uint8_t read_IMU(void)
{
    uint8_t i;
    
    bulk_tx_packet[0]=0x03;
      
    for(i = 0;i < (sizeof(imu_calib_type) / 4);i++)
    {
        ((uint32_t *)(void *)(bulk_tx_packet + 1))[i] = *(uint32_t *)(IMU_CALIB_DATA_ADDR + i*4);
    }
          
    TX_push(bulk_tx_packet,sizeof(imu_calib_type) + 1,4);
    return 0;
}

/**
  * @brief  write data about IMU offset.
  * @param  None
  * @retval None
  */
uint8_t write_IMU(uint8_t *buf)
{
    uint32_t nv_cpy[64];
    uint32_t *ptr = (uint32_t *)(buf+1);
    
    for (int i = 0;i<64;i++)
    {
            nv_cpy[i] = *((uint32_t *)(NV_ADDR_BASE + 4*i));
    }
            
    for (int i = 0;i<(sizeof(imu_calib_type)/4);i++)
    {
            nv_cpy[i+8] = ptr[i];
    }

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3);
    
    for (int i=0;i<64;i++)
    {
            FLASH_ProgramWord(NV_ADDR_BASE + 4*i,nv_cpy[i]);
    }
    
    FLASH_Lock();
    
    imu_data_init();
            
    return 0;
}

/**
  * @brief  read NV data from MCU.
  * @param  None
  * @retval None
  */
uint8_t read_NV(void)
{
  uint8_t i;
  uint32_t *ptr = (uint32_t *)(bulk_tx_packet+1);
	
    bulk_tx_packet[0]=0x05;
		
    for(i=0;i<8;i++)
      ptr[i]= *((uint32_t *)(NV_ADDR_BASE + 4*i));
		
    TX_push(bulk_tx_packet,33,4);
    return 0;
}

/**
  * @brief  write NV data from MCU.
  * @param  None
  * @retval None
  */
uint8_t write_NV(uint8_t *buf)
{
	uint32_t nv_cpy[64];
	uint32_t *ptr = (uint32_t *)(buf+1);
	
	for (int i = 0;i<64;i++)
	{
		nv_cpy[i] = *((uint32_t *)(NV_ADDR_BASE + 4*i));
	}
		
	for (int i = 0;i<8;i++)
	{
		nv_cpy[i] = ptr[i];
	}

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3);
	
	for (int i=0;i<64;i++)
	{
            FLASH_ProgramWord(NV_ADDR_BASE + 4*i,nv_cpy[i]);
	}
	
	FLASH_Lock();
		
	return 0;
}

/**
  * @brief  read version of firmware.
  * @param  None
  * @retval None
  */
uint8_t read_softver(void)
{
    bulk_tx_packet[0]=0x07;
    
    bulk_tx_packet[1]=MAIN_VERSION;
    bulk_tx_packet[2]=SUB_VERSION;

    TX_push(bulk_tx_packet,3,4);
    return 0;
}

/**
  * @brief  Fight the transmit frame.
  * @param  None
  * @retval None
  */
uint8_t push_rx_packet(uint8_t *buf,uint8_t count)
{
    uint8_t chksum;
    uint8_t i;
  
    if (bulk_rx_packet_len == 0)
    {
        if (!((buf[USB_START] == 0x2b)&& (buf[USB_START1] == 0x2b)))
        {
            asm("nop");
            return 0;
        }
    }
    
    if (bulk_rx_packet_len + count > (AGRE_MAX_LEN-1))
    {
        bulk_rx_packet_len = 0;
        return 0;
    }
    else
    {
        memcpy(bulk_rx_packet+ bulk_rx_packet_len, buf, count);
        bulk_rx_packet_len  += count;
    }
    
    if (bulk_rx_packet_len>9)
    {		
          RX_length = bulk_rx_packet[USB_LEN]+bulk_rx_packet[USB_LEN1]*256;
          if (RX_length == bulk_rx_packet_len - 9)
          {
              if ((bulk_rx_packet[bulk_rx_packet_len-2] == 0xb2) && (bulk_rx_packet[bulk_rx_packet_len-1]==0xb2))
              {
                    chksum=0;
                    for(i=0;i<RX_length;i++)
                      chksum=chksum+bulk_rx_packet[USB_BODY+i];               
                    if(chksum == bulk_rx_packet[bulk_rx_packet_len-3])
                    {
                          if((bulk_rx_packet[USB_TYPE]+bulk_rx_packet[USB_TYPE1]*256)==0x0003)
                          {
                                switch(bulk_rx_packet[USB_CMD])
                                {
                                                                                       
                                      case 0x03:  read_IMU(); 
                                              break;
                                                                                              
                                      case 0x04:   write_IMU(bulk_rx_packet+USB_BODY);
                                              break;
                                                                                              
                                      case 0x05:  read_NV();
                                              break;
                                                                                       
                                      case 0x06:  write_NV(bulk_rx_packet+USB_BODY);
                                              break;
                                                                                       
                                      case 0x07:  read_softver();
                                              break;
                                      default:
                                              break;             
                                }            
                          }
                          bulk_rx_packet_len = 0;
                    }
                    else
                      bulk_rx_packet_len = 0;
              }
              else
                bulk_rx_packet_len = 0;
          }
    }    
    return 0;
}

/**
  * @brief  usb trans trans and receive.
  * @param  None
  * @retval None
  */
void trans_poll(void)
{

   if (bulk_rx_flag)
    {
			
      push_rx_packet(bulk_rx_buf,bulk_rx_count);

      bulk_rx_flag=0;
    
    }
	 
    if(push_cnt>0)
    {
        if((PrevXferDone)  && (USB_OTG_dev.dev.device_status==USB_OTG_CONFIGURED))
        {
          if(USBD_CUSTOM_bulk_send  (&USB_OTG_dev, buf_temp[send_cnt], USB_MAX_LEN)==0)//是否发送成功
          {
            push_cnt--;
            send_cnt++;
            if(send_cnt>=(temp_rage-1))
              send_cnt=0;
          }
        PrevXferDone = 0;
        }
    }

}

/**
  * @brief  send lighthouse version report.
  * @param  None
  * @retval None
  */
void LV_poll()
{
	if (systick_count_l % 5000 == 0 && systick_flag)
	{			
            if (cube_flush_flag[0])
            {
                    TX_push((uint8_t *)(&lighthouse_version_report[0]),sizeof(lighthouse_version_report),5);
                    cube_flush_flag[0] = 0;
            }
            if (cube_flush_flag[1])
            {
                    TX_push((uint8_t *)(&lighthouse_version_report[1]),sizeof(lighthouse_version_report),5);
                    cube_flush_flag[1] = 0;
            }
            
            systick_flag = 0;
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
