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
  * @file    bsp_init.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the all peripheral:
  *            + systick configuration
  *            + tim1 configuration
  *            + tim2 configuration
  *            + tim3 configuration
  *            + GPIO peripheral configuration
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private macro -------------------------------------------------------------*/
GPIO_TypeDef* GPIO_LED[2] = { GPIOB, GPIOB };
const uint16_t PIN_LED[2] = { GPIO_Pin_3, GPIO_Pin_4 };

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Systick configuration.
  * @param  None
  * @retval None
  */
void systickInit() 
{
	SysTick_Config(SYS_CLK);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  TIM1 configuration.
  * @param  None
  * @retval None
  */
void tim1Init() 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	TIM_DeInit(TIM1);                                   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	/* configure TIM1 */
	TIM_TimeBaseStructure.TIM_Period = TIM1_PERIOD - 1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

	TIM_OCStructInit(&TIM_OCInitStructure);
        
	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	TIM_OCInitStructure.TIM_Pulse = TIM1_PERIOD;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  	
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  	
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = 0x90;  
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM_ARRPreloadConfig(TIM1, ENABLE);                	
	TIM_Cmd(TIM1, ENABLE);  	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);	
	TIM1->BDTR |= 1 << 15;
}

/**
  * @brief  TIM1 configuration.
  * @param  None
  * @retval None
  */
void tim2Init() 
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
        
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
        
        /* configure TIM2 */
	TIM_DeInit(TIM2);
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Prescaler = (2400/ TIM2_UPDATE_COUNT / MOTOR_TARGET_FREQ) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM2_PERIOD - 1;                             //100Hz
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;     
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;    
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;       
	TIM_ICInitStructure.TIM_ICFilter = 0xf;          
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
	TIM_OCInitStructure.TIM_Pulse = TIM2_PERIOD;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	if (IS_SLAVE) 
        {
              TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising ; 
              TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
              TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;      
              TIM_ICInitStructure.TIM_ICFilter = 0xf;          
              TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
              TIM_ICInit(TIM2, &TIM_ICInitStructure);
	}
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_Cmd(TIM2, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  TIM3 configuration.
  * @param  None
  * @retval None
  */
void tim3Init() 
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_DeInit(TIM3);
	TIM_InternalClockConfig(TIM3);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD - 1;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
        
        
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/10;           
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);                       
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
        
	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/10;          
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);                           
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  GPIO peripheral of output configuration.
  * @param  None
  * @retval None
  */
void gpioOutputInit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t outputValue) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	if (outputValue)
		GPIO_SetBits(GPIOx, GPIO_Pin);
	else
		GPIO_ResetBits(GPIOx, GPIO_Pin);
}

/**
  * @brief  GPIO peripheral of IPU configuration.
  * @param  None
  * @retval None
  */
void gpioIPUInit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/**
  * @brief  GPIO peripheral configuration.
  * @param  None
  * @retval None
  */
void gpioInit() 
{

	gpioIPUInit(GPIOB, GPIO_Pin_0);
	gpioOutputInit(GPIO_LED[0], PIN_LED[0], 1);     //Laser1
	gpioOutputInit(GPIO_LED[1], PIN_LED[1], 1);     //Laser2
	gpioIPUInit(GPIOA, GPIO_Pin_0);                 //TIM2_CH1 for HLC2701
	gpioIPUInit(GPIOA, GPIO_Pin_1);                 //TIM2_CH2 for HLC2701
	gpioIPUInit(GPIOA, GPIO_Pin_3);                 //sync in
}

/****END OF FILE****/

