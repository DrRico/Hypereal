/**
  ******************************************************************************
  * @file    main_df.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    23-September-2016
  * @brief   This file provides all the Application firmware functions.
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "si4432.h"
#include "math.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define KP                                              100.0f
#define KI                                              0.0f            // not used yet
#define KD                                              600.0f
#define SYNC_PACKET_LENGTH				7
#define TIM2_CNT                                        50000
/* Private variables ---------------------------------------------------------*/
static uint16_t isNotSync = WIRELESS_NOT_SYNC_THRESHOLD;
const static int targetPeriod = TIM2_PERIOD * TIM2_UPDATE_COUNT;
const int *IS_SLAVE = (const int *) 0x0800f000;
float errorIntegral[2];
float pwm[2];

int slaveSyncCapture = 0;

uint8_t wData[7] = { 0, 0, 0, 0, 0, 0, 0 };

static uint8_t sendBuffer[250], l = 0;


float SCAN_PHASE_OFFSET[2] = { 0.55f, 0.8f };

static int dphase[2];
int ddphase = 0;
uint8_t periodLock[2] = { 0, 0 };
uint8_t phaseLockCount[2] = { 0, 0 };

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Configure delay time .
  * @param  None
  * @retval None
  */
void delay(uint32_t us) {
	while (us--)
		asm("nop");
}

/**
  * @brief  Configure phase of the feedback signal of the two axes 
  *         with the wireless synchronization signal  
  * @param  None
  * @retval None
  */
static void pidUpdate(uint8_t m, int period, int ct, int ct4) 
{
	float df = 30 - 30.0f * (targetPeriod / 2) / period;            //cycle offset
        
	periodLock[m] = (df < 0.05f && df > -0.05f);
	if (periodLock[m]) 
        {
		if (ct4 < ct) 
                {
			ct4 += targetPeriod / 2;
		}
		ddphase = dphase[m];
		dphase[m] = ct4 - ct;                                   //0~targetPeriod
		dphase[m] += SCAN_PHASE_OFFSET[m] * targetPeriod;
		while (dphase[m] > targetPeriod / 4)
			dphase[m] -= targetPeriod / 2;
		ddphase -= dphase[m];
		if (dphase[m] < 2000 && dphase[m] > -2000) 
                {
			if (phaseLockCount[m] < MOTOR_LOCK_THR) 
                        {
				phaseLockCount[m]++;
			}
		} else 
                {
			phaseLockCount[m] = 0;
		}
		df -= (float) dphase[m] / targetPeriod / 2;
		pwm[m] += df * KP + ddphase * KD / targetPeriod;
		errorIntegral[m] += df;
	} 
        else 
        {
		phaseLockCount[m] = 0;
		dphase[m] = 0;
		pwm[m] += df * KP * 5;
		errorIntegral[m] += df;
	}
	if (pwm[m] > PWM_PERIOD * 7 / 8)
		pwm[m] = PWM_PERIOD * 7 / 8;
	if (pwm[m] < PWM_PERIOD / 2)
		pwm[m] = PWM_PERIOD / 2;
}

/**
  * @brief  Updata PID at 60Hz
  * @param  None
  * @retval None
  */
void onPeriodUpdate(int p1, int p2, uint32_t c1, uint32_t c2, uint32_t c4) 
{
	if (isNotSync != WIRELESS_NOT_SYNC_THRESHOLD || (!*IS_SLAVE)) {
		pidUpdate(0, p1, c1, c4);
		pidUpdate(1, p2, c2, c4);
	}
}

/**
  * @brief  Configure package which send to HMD
  * @param  None
  * @retval None
  */
void sendSyncPackage(int8_t dp1, int8_t dp2, int16_t phaseOffset[2]) 
{
	if (*IS_SLAVE && isNotSync == WIRELESS_NOT_SYNC_THRESHOLD) 
        {
		return;
	}
	uint8_t isSlave = *IS_SLAVE ? 1 : 0;
	isNotSync += isSlave;
	wData[isSlave] = 0;
	wData[1 - isSlave] = 0x80;
	wData[0] |= (dp1 & 0x7f);
	wData[1] |= (dp2 & 0x7f);
	wData[2] = phaseOffset[0] >> 4;
	wData[3] = phaseOffset[1] >> 4;
	wData[4] = (phaseOffset[0] << 4) | (phaseOffset[1] & 0xf);
	wData[5] = VER;
	wData[6]++;
	si4432_send(wData, SYNC_PACKET_LENGTH);
}

/**
  * @brief  slave lighthouse receive the wireless synchronization signal from master lighthouse 
  * @param  None
  * @retval None
  */
static void onGlobalSync() 
{
	int tim2Capture = (int16_t) TIM_GetCapture4(TIM2);
        
	if (slaveSyncCapture > 0 && slaveSyncCapture < 2500* TIM2_UPDATE_COUNT)
		slaveSyncCapture = (int) (tim2Capture * 0.1f + slaveSyncCapture * 0.9f);
	else
		slaveSyncCapture = TIM_GetCapture4(TIM2);

	int dp =slaveSyncCapture -  ((960 * 3) - 11) * TIM2_UPDATE_COUNT / 2;
	int absdp = dp < 0 ? -dp : dp;
        
	if (absdp < 15) 
        {
		isNotSync = 0;
	}
	tim2UpdateCount = TIM2_UPDATE_COUNT / 2;
	if (dp > 500)
		dp = 500;
	if (dp < -500)
		dp = -500;
	TIM2->CNT -= dp;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) 
{
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
        
	delay(2000000);
        /* Configure GPIO peripheral*/
	gpioInit();
        /* Configure parameters of lighthouse*/
        initConfiguration();
        /* Configure si4432 Module */
	si4432_init(WIRELESS_CHANNEL);
        /* Configure TIM1 peripheral for laser*/
	tim1Init();
        /* Configure TIM2 peripheral for measure cycle of motor*/
	tim2Init();
        /* Configure TIM3 peripheral for control cycle about motor*/
	tim3Init();
        /* Configure the systick to 1ms*/
	systickInit();
        
	while (1) 
        {
            
            if (TIM2->CNT < TIM2_CNT) 
            {
                    l = si4432_read(sendBuffer);
                    if (l == SYNC_PACKET_LENGTH && (sendBuffer[0] & 0x80) == 0 && *IS_SLAVE) 
                    {
                            onGlobalSync();
                    }
            }
	}
}

/************************ (C) COPYRIGHT Hypereal *****END OF FILE****/
