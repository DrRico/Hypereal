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
  * @file    stm32fxxx_it.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32fxxx_it.h"
#include "wireless_phase_control.h"
#include "led.h"
#include "hmc5883l.h"
#include "hr_mk1_config.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED 
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif
extern uint8_t hmc5883_flush_flag;
extern uint32_t wireless_attached_count_down;
extern uint8_t HMC5883_RX_BUF[];

extern uint32_t wp_time_line_comp[];
extern uint8_t last_sync_flag; 

unsigned int systick_flag = 0;
uint32_t systick_count_l = 0;
uint32_t systick_count_h = 0;

uint8_t systick_2ms_first_flag = 0;
uint8_t systick_2ms_second_flag = 0;

uint8_t s1_flag = 0;
uint8_t s1_count = 0;

uint8_t exti3_flag = 0;

/* Private function prototypes -----------------------------------------------*/

/******************************************************************************/
/*             Cortex-M Processor Exceptions Handlers                         */
/******************************************************************************/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    if (systick_count_l % 2)
    {
        systick_2ms_first_flag = 1;
    }
    else
    {
        systick_2ms_second_flag = 1;
    }

    systick_flag = 1;
    systick_count_l ++;
    if (!systick_count_l)
                    systick_count_h ++;
    
    if (wireless_attached_count_down)
    {
        wireless_attached_count_down --;
    }
    
    if (s1_count != (systick_count_l>>10))
    {
        s1_count = systick_count_l>>10;
        s1_flag = 1;
    }
    
    return;
}

/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}

/**
* @brief  This function handles EXTI3_IRQ Handler.
* @param  None
* @retval None
*/
void EXTI3_IRQHandler(void)
{

	if ((currunt_wireless_phase == WP_IDLE) )
	{
		wireless_phase_control_sync();
		hz_30_timmer_sync();	
	}
	else if (currunt_wireless_phase == WP_LIGHT_A)
	{
		wireless_phase_control_sync();
		hz_30_timmer_sync();	
	}
	else if (currunt_wireless_phase == WP_LIGHT_B)
	{
		wireless_phase_control_sync();
	}
	
	EXTI_ClearITPendingBit(EXTI_Line3);	

	exti3_flag = 1;
}

/**
* @brief  This function handles EXTI15_10_IRQ Handler.
* @param  None
* @retval None
*/
#ifdef USE_STM3210C_EVAL 
void EXTI9_5_IRQHandler(void)
#else
void EXTI15_10_IRQHandler(void)
#endif
{
}

/**
* @brief  This function handles TIM2_IRQ Handler.
* @param  None
* @retval None
*/
void TIM2_IRQHandler()
{
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
    {
        if (currunt_wireless_phase) 
        {
              currunt_wireless_phase = (currunt_wireless_phase + 1)%WP_PHASE_STAGE_NUM;
              TIM2->ARR = wp_time_line_comp[currunt_wireless_phase] -1;
              wireless_phase_flag = currunt_wireless_phase;
              
              if (!(wp_callback[currunt_wireless_phase] == NULL))
              {
                      (*wp_callback[(currunt_wireless_phase)])();
              }
        }
        else
        {
              if (last_sync_flag)
              {
                      currunt_wireless_phase = (currunt_wireless_phase+1)%WP_PHASE_STAGE_NUM;
                      TIM2->ARR = wp_time_line_comp[currunt_wireless_phase] -1;
                      wireless_phase_flag = currunt_wireless_phase;
                      
                      if (!(wp_callback[currunt_wireless_phase] == NULL))
                      {
                              (*wp_callback[(currunt_wireless_phase)])();
                      }
                      
                      last_sync_flag = 0;
                      scan_num ++;
              }
              else
              {
                      TIM_Cmd(TIM2,DISABLE);
              }
        }
        
        TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
        TIM_ClearFlag(TIM2,TIM_FLAG_Update);
    }
}

/**
  * @brief  This function handles ADCx DMA IRQHandler Handler.
  * @param  None
  * @retval None
  */
void ADCx_DMA_IRQHandler(void)
{  
}


/**
* @brief  This function handles OTG_HS Handler.
* @param  None
* @retval None
*/
#ifdef USE_USB_OTG_HS  
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED 
/**
* @brief  This function handles EP1_IN Handler.
* @param  None
* @retval None
*/
void OTG_HS_EP1_IN_IRQHandler(void)
{
  USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

/**
* @brief  This function handles EP1_OUT Handler.
* @param  None
* @retval None
*/
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}
#endif

/**
  * @brief  This function handles DMA1_Channel7_IRQHandler/trans I2C.
  * @param  None
  * @retval None
  */
void I2C_DMA_TX_IRQHandler(void)
{
    /* Check if the DMA transfer is complete */ 
    if(DMA_GetFlagStatus(I2C_DMA_STREAM_TX,I2C_DMA_IT_TX_TC) != RESET) 
    { 
      /* Disable the DMA Tx Channel and Clear all its Flags */ 
      DMA_Cmd(I2C_DMA_STREAM_TX, DISABLE);  
      DMA_ClearFlag(I2C_DMA_STREAM_TX,I2C_TX_DMA_FLAG_TCIF); 
      /*!< Wait till all data have been physically transferred on the bus */  
      while(!I2C_GetFlagStatus(HMC5883_I2Cx, I2C_FLAG_BTF));
      /*!< Send STOP condition */  
      I2C_GenerateSTOP(HMC5883_I2Cx, ENABLE); 
      /* Perform a read on SR1 and SR2 register to clear eventualaly pending flags */ 
      (void)HMC5883_I2Cx->SR1; 
      (void)HMC5883_I2Cx->SR2; 
      /* Clear hmc5883 trans flag */ 
      hmc5883_busy = 0;
    }
}

/**
  * @brief  This function handles DMA1_Channel3_IRQHandler/READ I2C.
  * @param  None
  * @retval None
  */
void I2C_DMA_RX_IRQHandler(void)
{
    /* Check if the DMA transfer is complete */  
    if(DMA_GetFlagStatus(I2C_DMA_STREAM_RX,I2C_DMA_IT_RX_TC) != RESET) 
    {       

         /* Disable the DMA Rx Channel and Clear all its Flags */ 
        DMA_Cmd(I2C_DMA_STREAM_RX, DISABLE); 
        DMA_ClearFlag(I2C_DMA_STREAM_RX,I2C_RX_DMA_FLAG_TCIF);   
        
        I2C_AcknowledgeConfig(HMC5883_I2Cx, DISABLE);
        /*!< Send STOP Condition */  
        I2C_GenerateSTOP(HMC5883_I2Cx, ENABLE); 
            
         /* Perform a read on SR1 and SR2 register to clear eventualaly pending flags */ 
        hmc5883_buf[1] = HMC5883_RX_BUF[0];
        hmc5883_buf[0] = HMC5883_RX_BUF[1];
        hmc5883_buf[3] = HMC5883_RX_BUF[2];
        hmc5883_buf[2] = HMC5883_RX_BUF[3];
        hmc5883_buf[5] = HMC5883_RX_BUF[4];
        hmc5883_buf[4] = HMC5883_RX_BUF[5];

        /* Perform a read on SR1 and SR2 register to clear eventualaly pending flags */ 
        (void)HMC5883_I2Cx->SR1; 
        (void)HMC5883_I2Cx->SR2; 
        
        hmc5883_busy = 0;
        hmc5883_flush_flag = 1;
     } 
}

/*****************************END OF FILE****/
