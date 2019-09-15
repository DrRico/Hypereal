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
  * @file    stm32_it.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32_it.h"
#include "main.h"

/* Private variables ---------------------------------------------------------*/
int period[2], capLast[2], capLast4;
static int16_t phaseOffset[2] = { 0, 0 };
static uint16_t capCount[2];

uint8_t tim2UpdateCount = 0;

void updateLaser(uint8_t id);
const static int target30HzPeriod = TIM2_PERIOD * TIM2_UPDATE_COUNT;
const static int target60HzPeriod = TIM2_PERIOD * TIM2_UPDATE_COUNT / 2;

int32_t periodtemp[2][256];
uint8_t pti[2] = { 0, 0 };
int dPeriod[2];

volatile uint8_t turnOffCount[2] = { 0, 0 };

extern float pwm[2];
extern uint8_t phaseLockCount[2];

/**
* @brief  measure feedback of rotate about motor.
* @param  None
* @retval None
*/
static void onMotorFeedback(int m, uint32_t cap) 
{
    int p;
    
    cap += TIM2_PERIOD * tim2UpdateCount;
    if(capCount[m]>3)
        p = target30HzPeriod;
    else if (cap > capLast[m])
        p = (cap - capLast[m]);
    else
        p = (cap + target30HzPeriod - capLast[m]);
    capLast[m] = cap;
    periodtemp[m][pti[m]++] = p;
    period[m] = p;
    updateLaser(m);
    capCount[m] = 0;
}

/**
* @brief  This function handles TIM2_IRQ Handler.
* @param  None
* @retval None
*/
void TIM2_IRQHandler(void) 
{
    uint32_t p[2];
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) 
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
        uint16_t cap = TIM_GetCapture2(TIM2);
        onMotorFeedback(1, cap);
    } 
    else if (TIM_GetITStatus(TIM2, TIM_IT_CC1)) 
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        uint16_t cap = TIM_GetCapture1(TIM2);
        onMotorFeedback(0, cap);
    } 
    else if (TIM_GetITStatus(TIM2, TIM_IT_Update)) 
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        tim2UpdateCount++;
        capCount[0]++;
        capCount[1]++;
        if (tim2UpdateCount == TIM2_UPDATE_COUNT) 
        {
                for (int i = 0; i < 2; i++) 
                {
                        p[i] = period[i];
                        if (p[i] < TIM2_PERIOD * TIM2_UPDATE_COUNT / 4)
                                p[i] = TIM2_PERIOD * TIM2_UPDATE_COUNT / 4;
                        if (phaseLockCount[i] >= MOTOR_LOCK_THR) 
                        {
                                dPeriod[i] = (p[i] - target60HzPeriod) / 2;
                        } 
                        else 
                        {
                                dPeriod[i] = 63;
                        }
                        phaseOffset[i] = (int) (SCAN_PHASE_OFFSET[i] * target30HzPeriod - capLast[i]) / 2;
                }
                sendSyncPackage(dPeriod[0], dPeriod[1], phaseOffset);
                onPeriodUpdate(p[0], p[1], capLast[0], capLast[1], 0);
                
                tim2UpdateCount = 0;
        }
    }
}

/**
* @brief  control led after motor spinning.
* @param  None
* @retval None
*/
void updateLaser(uint8_t index) 
{
      uint16_t tim2 = TIM_GetCounter(TIM2);
      uint16_t volatile *ccr = &TIM1->CCR1;
        
      if (phaseLockCount[index] < MOTOR_LOCK_THR)             //not sync
		return;
        
	ccr += index * 2;
	if (tim2UpdateCount >= TIM2_UPDATE_COUNT / 2 ) 
        {
		turnOffCount[index] = 7;
		*ccr = 18;                              //turn on laser
                
		GPIO_SetBits(GPIO_LED[index], PIN_LED[index]);
	}
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler() 
{
	for (int i = 0; i < 2; i++) 
        {
		uint16_t volatile *ccr = &TIM1->CCR1;
		uint16_t volatile *ccr3 = &TIM3->CCR2;
                
		ccr += i * 2;
		ccr3 -= i * 2;
		if (turnOffCount[i]) {
			turnOffCount[i]--;
		} else 
                {
			*ccr = 36; //turn off laser
			if (phaseLockCount[i] < MOTOR_LOCK_THR || (GPIO_LED[i]->ODR & PIN_LED[i])) 
                        {
				*ccr3 = (int) pwm[i];
			}
			GPIO_ResetBits(GPIO_LED[i], PIN_LED[i]);
		}
	}
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
	while (1) 
        {
	}
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void) 
{
      /* Go to infinite loop when Hard Fault exception occurs */
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

/****END OF FILE****/