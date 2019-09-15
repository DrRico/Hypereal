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
  * @file    wireless_phase_control.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides firmware functions to configure timing about wireless of General-purpose timers.
  ******************************************************************************

  */ 
/* Includes ------------------------------------------------------------------*/ 
#include "stm32f4xx_tim.h"
#include "stdio.h"
#include "hr_mk1_config.h"

/* Private define ------------------------------------------------------------*/
#define hz_30_time      350000

/* Private variables ---------------------------------------------------------*/
const uint16_t wp_time_line_point[WP_PHASE_STAGE_NUM] = WP_TIME_LINE;
const float wp_time_line_scale = WP_TIME_LINE_SCALE; 

uint32_t wp_time_line_comp[WP_PHASE_STAGE_NUM];
uint8_t currunt_wireless_phase;
uint8_t wireless_phase_flag;
uint16_t scan_num = 0;

uint32_t wireless_attached_count_down = 0;

uint8_t last_sync_flag = 0;
uint8_t light_a_sync_flag = 0;
uint8_t light_b_sync_flag = 0;
uint8_t light_a_sync_flag_last = 0;
uint8_t light_b_sync_flag_last = 0;

uint32_t tim5_cpy;
signed int tim5_diff = 0;
float tim5_adj = 0;

uint32_t tim5_arr = hz_30_time;
uint8_t hz_30_timmer_overflow_flag = 0;

extern uint32_t systick_count_l;

void (*wp_callback[WP_PHASE_STAGE_NUM])();

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  TIM2,TIM14 configuration.
  * @param  None
  * @retval None
  */
void phase_timmer_init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM14,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 7;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              //timer about process time
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);             //timer about between interrupt and deal data
	
        /*configure TIM2 IRQ*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	return;
}

/**
  * @brief  TIM5 configuration.
  * @param  None
  * @retval None
  */
void hz_30_timmer_init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

	TIM_TimeBaseStructure.TIM_Period = hz_30_time;
	TIM_TimeBaseStructure.TIM_Prescaler = 7;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;  
	
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_SetCounter(TIM5,hz_30_time/2);
	
	return;
}

/**
  * @brief  clear flag about timer.
  * @param  None
  * @retval None
  */
void wp_callback_test0()
{
	LED0(1);
		
	if (light_a_sync_flag || light_b_sync_flag)
	{
            last_sync_flag = 1;
	}
	
	light_a_sync_flag_last = light_a_sync_flag;
	light_b_sync_flag_last = light_b_sync_flag;
	
	light_a_sync_flag = 0;
	light_b_sync_flag = 0;
}

/**
  * @brief  timer a cycle in 30Hz   .
  * @param  None
  * @retval None
  */
void hz_30_timmer_sync()
{
	tim5_cpy = TIM5->CNT;
	TIM5->CNT = hz_30_time/2;
	TIM_Cmd(TIM5,ENABLE);    	
	
	if (tim5_cpy >(hz_30_time/2))
	{
            tim5_diff = tim5_cpy - (hz_30_time/2);
	}
	else if (tim5_cpy<=(hz_30_time/2))
	{
            tim5_diff = -(hz_30_time/2 - tim5_cpy);
	}
	else
	{
            asm("nop");
	}

	tim5_adj = tim5_adj * 0.9f + tim5_diff*0.1f;
	tim5_arr = hz_30_time + tim5_adj;

	TIM5->ARR = tim5_arr;
	
}

/**
  * @brief  TIM5 configuration.
  * @param  None
  * @retval None
  */
void wireless_phase_control_scan_num_push(uint8_t num_in)
{
		
	if (num_in<(scan_num&0x00ff))
	{
		scan_num = (((scan_num>>8)+1)<<8) + num_in;
	}
	else
	{
		scan_num = (scan_num &0xff00) + num_in;
	}
	
	if (wireless_attached_count_down == 0)
	{
		scan_num &= 0x00ff;
	}
	wireless_attached_count_down = 5000;
	
	return;
}

/**
  * @brief  configure callback.
  * @param  None
  * @retval None
  */
void wireless_phase_control_init()
{
	for (int i=0;i<WP_PHASE_STAGE_NUM;i++)
	{
		wp_time_line_comp[i] = (uint32_t)(wp_time_line_point[i] * wp_time_line_scale);
		wp_callback[i] = NULL;
	}
	
	/* timmer init */
	phase_timmer_init();
	hz_30_timmer_init();
	
	/* wp_status init */
	currunt_wireless_phase = WP_IDLE;
	
	wp_callback[0]  = wp_callback_test0;
        wp_callback[1]  = NULL;
	wp_callback[2]  = wp_callback_bx;
	wp_callback[3]  = wp_callback_hmc5883;
	wp_callback[6]  = wp_callback_hmc5883_ptr_flush;
	wp_callback[5]  = wp_callback_by;
	wp_callback[7]  = wp_callback_ax;
	wp_callback[10] = wp_callback_ay;
	
	return;
}

/**
  * @brief  clear TIM14
  * @param  None
  * @retval None
  */
void wireless_phase_control_sync()
{
	TIM_SetCounter(TIM14,0);
	TIM_ClearFlag(TIM14,TIM_FLAG_Update);
	TIM_Cmd(TIM14,ENABLE);    
}

/**
  * @brief  timer about receive wireless data from lighthouse A
  * @param  None
  * @retval None
  */
void wireless_phase_control_run()
{
	uint32_t phase_offset;
	
	LED0(0);
	
	asm("CPSID I");
	
	TIM_Cmd(TIM2,DISABLE);	
	TIM_Cmd(TIM14,DISABLE);
	
	if ((wp_time_line_comp[WP_LIGHT_A] > WP_SYNC_DELAY + TIM_GetCounter(TIM14)) && !(TIM_GetFlagStatus(TIM14,TIM_FLAG_Update) == SET))
	{
               /*+ tim5_diff_buf*/
              phase_offset = (wp_time_line_comp[WP_LIGHT_A] - WP_SYNC_DELAY - TIM_GetCounter(TIM14));
              currunt_wireless_phase = WP_LIGHT_A;	
              
                      TIM2->ARR = phase_offset;
                      
                      TIM_ClearFlag(TIM2,TIM_FLAG_Update);   
                      TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);   
                      TIM_SetCounter(TIM2,0);
                      TIM_Cmd(TIM2,ENABLE);   	
                      
                      light_a_sync_flag = 1;
	}
	else
	{
              asm("nop");
              currunt_wireless_phase = WP_IDLE;
	}
	
	asm("CPSIE I");
	
	TIM_ClearFlag(TIM14,TIM_FLAG_Update);
}

/**
  * @brief  timer about receive wireless data from lighthouse B
  * @param  None
  * @retval None
  */
void wireless_phase_control_run_from_light_b()
{
	uint32_t phase_offset;
	
	asm("CPSID I");
	
	TIM_Cmd(TIM2,DISABLE);	
	TIM_Cmd(TIM14,DISABLE);
	
	if ((wp_time_line_comp[WP_LIGHT_B] > WP_SYNC_DELAY + TIM_GetCounter(TIM14)) && !(TIM_GetFlagStatus(TIM14,TIM_FLAG_Update) == SET))
	{
  
              phase_offset = (wp_time_line_comp[WP_LIGHT_B] - WP_SYNC_DELAY - TIM_GetCounter(TIM14));
              currunt_wireless_phase = WP_LIGHT_B;	
              
                      TIM2->ARR = phase_offset;
                      
                      TIM_ClearFlag(TIM2,TIM_FLAG_Update);  
                      TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);   
                      TIM_SetCounter(TIM2,0);
                      TIM_Cmd(TIM2,ENABLE);    
                      
                      light_b_sync_flag = 1;
	}
	else
	{
              asm("nop");
              currunt_wireless_phase = WP_IDLE;
	}
	
	asm("CPSIE I");
	
	TIM_ClearFlag(TIM14,TIM_FLAG_Update);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
