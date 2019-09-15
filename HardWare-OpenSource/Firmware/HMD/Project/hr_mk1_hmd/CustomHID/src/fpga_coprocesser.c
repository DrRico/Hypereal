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
  *          functionalities of the FSMC peripheral:
  *            + FSMC Interface configuration
  *            + communicate with FPGA for measure light application time from lighthouses
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "fpga_coprocesser.h"
 
/* Private variables ---------------------------------------------------------*/
uint8_t measure_ch;
uint8_t measure_busy_flag = 0;

uint8_t measure_done_flag = 0;
uint8_t measure_done_ch = 0;

uint8_t wireless_phase_lowpass_en = 0;

uint8_t measure_result_valid[4] = {0,0,0,0};    //0: X from lighthouse A,1: Y from lighthouse A,2: X from lighthouse B,3: Y from lighthouse B

measure_result_type measure_result[4];  //0: X from lighthouse A,1: Y from lighthouse A,2: X from lighthouse B,3: Y from lighthouse B
measure_result_type measure_result_buf;
		
LightSensorData_type light_sensor_data_buf;
light_sync_package_type light_sync_package[2];	//0: light A; 1: lightB;


/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Configure the FSMC peripheral for FPGA .
  * @param  None
  * @retval None
  */
static void fsmc_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  readWriteTiming;

    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE); 
    
    /* Configure GPIOD/GPIOE */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7
                                  |GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8
                                 |GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource7,GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource11,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FSMC);

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource2,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource3,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource4,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FSMC);

    /* Set setuptime 24ns, datasetuptime 24ns,mode A*/
    readWriteTiming.FSMC_AddressSetupTime = 0x04;	
    readWriteTiming.FSMC_AddressHoldTime = 0x04;	
    readWriteTiming.FSMC_DataSetupTime = 0x04;	       	 
    readWriteTiming.FSMC_BusTurnAroundDuration = 0x01;
    readWriteTiming.FSMC_CLKDivision = 0x04;
    readWriteTiming.FSMC_DataLatency = 0x00;
    readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A; 

    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;  
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;    
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b; 
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low; 
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait= FSMC_AsynchronousWait_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;  
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;	
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;  
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;  
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &readWriteTiming; 

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
		
    return;
}

/**
  * @brief  Configure the IO peripheral for FPGA .
  * @param  None
  * @retval None
  */
void fpga_coprocesser_init()
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    
    fsmc_init();

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    RCC_MCO2Config(RCC_MCO2Source_SYSCLK,RCC_MCO2Div_4);
    
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_MCO);

    /* FPGA_CTL0 = RST_N */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* FPGA_MEASURE_START */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* FPGA_INT */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* 3.3V_S_EN */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC,GPIO_Pin_15);

    /* control FPGA start measure */
    FPGA_RESET(0);
    FPGA_MEASURE(0);
    
    light_sync_package[0].flush_flag = 0;
    light_sync_package[1].flush_flag = 0;
    
    
    wireless_phase_lowpass_en = *(uint8_t *)NV_BYTE0_WIRELESS_PHASE_LOWPASS;
    if (wireless_phase_lowpass_en != WIRELESS_LOWPASS)
      wireless_phase_lowpass_en = 0;
    
    return;
}

/**
  * @brief  Put data from FPGA to result_buf.
  * @param  None
  * @retval None
  */
void fpga_poll()
{
  uint32_t calc_temp;
  uint8_t valid_count = 0;
  
  if (measure_busy_flag)
  {
    if (FPGA_IF_INT())
    {
              
      for (int i =0 ;i<32;i++)
      {
        calc_temp = FPGA_READ(i+128);
        
        if (calc_temp != 0) 
        {
          light_sensor_data_buf.MeasureResult = calc_temp;
          light_sensor_data_buf.SensorID = i;
          light_sensor_data_buf.TriggerCount = 1;
          
          measure_result_buf.LightSensor[valid_count] = light_sensor_data_buf;
          valid_count = valid_count + 1;
          LED1(0);
        }
        else
          LED1(1);
      }
      FPGA_RESET(0);
      
      measure_done_ch = measure_ch;
      measure_result_buf.AxisScanned = measure_done_ch;
      measure_result_buf.ScannedNumber = scan_num;
      measure_result_buf.SensorCount = valid_count;
      measure_busy_flag = 0;
      measure_done_flag = 1;					
    }
  }	
}

/**
  * @brief  Put data from lighthouse to package.
  * @param  None
  * @retval None
  */
void fpga_light_sync_package_process(uint8_t *buf, light_sync_package_type * package)
{
  package->id = buf[0]>>7;
  package->comp_x = ((signed char)(buf[0]<<1))/2;
  package->comp_y = ((signed char)(buf[1]<<1))/2;
  package->d_x = ((signed short)((((uint16_t)buf[2])<<8) + (buf[4]&0xf0)))/16;
  package->d_y = ((signed short)((((uint16_t)buf[3])<<8) + ((buf[4]&0x0f)<<4)))/16;
  package->flush_flag = 1;
  
  return;
}

/**
  * @brief  calculate fix from FPGA .
  * @param  None
  * @retval None
  */
void fpga_fix_calc(uint8_t tower_id)
{
  uint32_t result_temp;	
		
  /* deal with x-axis */
  if (light_sync_package[tower_id].comp_x >60 || light_sync_package[tower_id].comp_x < -60)
  {
          measure_result[tower_id<<1].SensorCount = 0;
  }
  else
  {
      for (int i = 0;i< measure_result[tower_id<<1].SensorCount;i++)
      {
          result_temp = measure_result[tower_id<<1].LightSensor[i].MeasureResult ;
          result_temp += light_sync_package[tower_id].d_x * RESULT_DIV_TOWER_SCALE;
          result_temp -= (result_temp-SPEED_CALC_SCALE1) * light_sync_package[tower_id].comp_x / SPEED_CALC_SCALE2;
          
          if (wireless_phase_lowpass_en)
          {
                  result_temp = result_temp - tim5_diff/2;
          }
          
          measure_result[tower_id<<1].LightSensor[i].MeasureResult = (uint16_t)result_temp;

      }
  }

  /* deal with y-axis */
  if (light_sync_package[tower_id].comp_y >60 || light_sync_package[tower_id].comp_y < -60)
  {
          measure_result[(tower_id<<1)+1].SensorCount = 0;
  }
  else
  {

    for (int i = 0;i< measure_result[(tower_id<<1)+1].SensorCount;i++)
    {
        result_temp = measure_result[(tower_id<<1)+1].LightSensor[i].MeasureResult ;
        result_temp += light_sync_package[tower_id].d_y * RESULT_DIV_TOWER_SCALE;
        result_temp -= (result_temp-SPEED_CALC_SCALE1) * light_sync_package[tower_id].comp_y / SPEED_CALC_SCALE2;
            
        if (wireless_phase_lowpass_en)
        {
                result_temp = result_temp - tim5_diff/2;
        }

        measure_result[(tower_id<<1)+1].LightSensor[i].MeasureResult = (uint16_t)result_temp;
    }
  }
  
  return;
}

/**
  * @brief  control FPGA to measure x-axis from lighthouse A .
  * @param  None
  * @retval None
  */
#pragma optimize=none
void wp_callback_ax() 
{
    LED1(1);
    FPGA_RESET(1);
    asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");
    
    FPGA_MEASURE(1);
    asm("nop");asm("nop");asm("nop");asm("nop");
    FPGA_MEASURE(0);
                    
    measure_ch = 1;

    measure_busy_flag = 1;
    
    if (!light_b_sync_flag && light_b_sync_flag_last)
    {
        light_sync_package[1].d_x -= 2*light_sync_package[1].comp_x;
        light_sync_package[1].d_y -= 2*light_sync_package[1].comp_y;
        light_sync_package[1].flush_flag = 1;
    }
}

/**
  * @brief  control FPGA to measure y-axis from lighthouse A .
  * @param  None
  * @retval None
  */
#pragma optimize=none
void wp_callback_ay() 
{
    LED1(1);
    FPGA_RESET(1);
    asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");

    FPGA_MEASURE(1);
    asm("nop");asm("nop");asm("nop");asm("nop");
    FPGA_MEASURE(0);
                    
    measure_ch = 2;

    measure_busy_flag = 1;
}

/**
  * @brief  control FPGA to measure x-axis from lighthouse b .
  * @param  None
  * @retval None
  */
#pragma optimize=none
void wp_callback_bx() 
{
    LED1(1);
    FPGA_RESET(1);
    asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");

    FPGA_MEASURE(1);
    asm("nop");asm("nop");asm("nop");asm("nop");
    FPGA_MEASURE(0);
                    
    measure_ch = 3;

    measure_busy_flag = 1;
    
    if (!light_a_sync_flag && light_a_sync_flag_last)
    {
            scan_num +=1;
            
            light_sync_package[0].d_x -= 2*light_sync_package[0].comp_x;
            light_sync_package[0].d_y -= 2*light_sync_package[0].comp_y;
            light_sync_package[0].flush_flag = 1;
    }
}

/**
  * @brief  control FPGA to measure y-axis from lighthouse B .
  * @param  None
  * @retval None
  */
#pragma optimize=none
void wp_callback_by() 
{
    LED1(1);
    FPGA_RESET(1);
    asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");

    FPGA_MEASURE(1);
    asm("nop");asm("nop");asm("nop");asm("nop");
    FPGA_MEASURE(0);
                    
    measure_ch = 4;

    measure_busy_flag = 1;
}

/*****************************END OF FILE****/
