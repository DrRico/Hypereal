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
  * @file    app.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides all the Application firmware functions.
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "hr_mk1_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
    
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;    
    
/* Private variables ---------------------------------------------------------*/
uint16_t InterpupillaryDistance = 0;

extern uint8_t measure_done_flag;
extern uint8_t exti3_flag;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Configure system
  * @param  None
  * @retval None
  */

void main_init()
{
        /* Enable Power Control clock */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG, ENABLE);		

        /* Init Device Library */
        USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
        USB_OTG_HS_CORE_ID,
#else            
        USB_OTG_FS_CORE_ID,
#endif
        &USR_desc, 
        &USBD_CUSTOMHID_cb, 
        &USR_cb);
  
        /* Configure the systick to 1ms*/
	SysTick_Config(SYS_CLK);          
        /* Configure acc,gyro Module */
	max21100_init();
        /* Configure mag Module */
	HMC5883L_init();                      
        /* Configure FSMC interface with FPGA */
	fpga_coprocesser_init();        
	/* Initializes calibration for acc and gyro */
	imu_data_init();                 
        /* Configure led IO */
	led_init();                     
	/* Set wireless callback */
	wireless_phase_control_init();         
        /* Configure si4432 Nodule */
	si4432_init();              

	return;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int main(void)
{
    /* Configure Systerm */
    main_init();

    while (1)
    {            
	/* Communication by USB*/
        trans_poll();
        /* read data about light application time from FPGA*/
	fpga_poll();
	/* Receive Synchronizing signal from lighthouse by wireless*/	
        if (exti3_flag || (!SI4432_NIRQ))
            si4432_poll();
        /* write measure result reg*/
        if (measure_done_flag)
        {            
            measure_result[measure_result_buf.AxisScanned - 1] = measure_result_buf;
            measure_result_valid[measure_result_buf.AxisScanned - 1] = 1;
            /* Clear measure flag*/
            measure_done_flag = 0;
        }
        /* Put measure result by lighthouse A in tansfer process */ 
        if ((light_sync_package[0].flush_flag == 1)&&(measure_result_valid[0])&&(measure_result_valid[1]))
        {
            /* measure location by lighthouse A  */
            fpga_fix_calc(0);
            /* Put light result in USB transfer stack */
            TX_push((uint8_t *)(void *)(&measure_result[0]),sizeof(measure_result[0]) - (128-4*measure_result[0].SensorCount),1);
            TX_push((uint8_t *)(void *)(&measure_result[1]),sizeof(measure_result[1]) - (128-4*measure_result[1].SensorCount),1);    
            
            /* Clear measure result flag*/
            light_sync_package[0].flush_flag = 0;
            measure_result_valid[0] = 0;
            measure_result_valid[1] = 0;
        }
	/* Put measure result of lighthouse B in tansfer process */ 	
        if ((light_sync_package[1].flush_flag == 1)&&(measure_result_valid[2])&&(measure_result_valid[3])) 
        {
            /* measure location by lighthouse B  */
            fpga_fix_calc(1);
            /* Put light result in USB transfer stack */
            TX_push((uint8_t *)(void *)(&measure_result[2]),sizeof(measure_result[2]) - (128-4*measure_result[2].SensorCount),1);
            TX_push((uint8_t *)(void *)(&measure_result[3]),sizeof(measure_result[3]) - (128-4*measure_result[3].SensorCount),1);
            
            /* Clear measure result flag*/
            light_sync_package[1].flush_flag = 0;
            measure_result_valid[2] = 0;
            measure_result_valid[3] = 0;
        }
        /* Put IMU result in USB transfer stack */                    
        imu_data_poll();	
        /* Put Mag result in temp buf */  
        hmc5883_poll();       
        /* Put lighthouse version in tansfer process */
        LV_poll();//µÆËþ°æ±¾°ü
    }
} 

#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/*****************************END OF FILE****/

