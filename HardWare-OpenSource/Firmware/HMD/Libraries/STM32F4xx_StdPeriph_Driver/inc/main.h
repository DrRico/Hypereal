#ifndef _MAIN_H_
#define _MAIN_H_

#define HZ_60
#define UART_BUF_MAX 2048

//DELAY_START = wireless sync signal to CAPTURE_START
//1unit = 762ns
#ifdef HZ_30
	#define DELAY_START 16995*4
	#define DELAY_END (26706*4-DELAY_START)
#endif

#ifdef HZ_60
//	#define DELAY_START (6069-1237)
//	#define DELAY_END  (15780-1237-DELAY_START)

	#define DELAY_START ((6069-1237)*4)
	//#define DELAY_END ((15780-1237)*4-DELAY_START)
	#define DELAY_END ((14000-1237)*4-DELAY_START)
#endif


//#define CLOCK_SCALE_762
#define CLOCK_SCALE_190
//190.476ns

#ifdef CLOCK_SCALE_762
	#define _TIM_DIV_HIGH_ 127
	#define _TIM_DIV_LOW_ 63
#endif

#ifdef CLOCK_SCALE_190
	#define _TIM_DIV_HIGH_ 31
	#define _TIM_DIV_LOW_ 15
#endif

#define USE_GYRO

//#define USE_PERIOD_FIX

#define TIM_OFFSET 100

#define LED_ON() GPIO_SetBits(GPIOG,GPIO_Pin_2);
#define LED_OFF() GPIO_ResetBits(GPIOG,GPIO_Pin_2);
#define LED_TOGGLE() GPIO_ToggleBits(GPIOG,GPIO_Pin_2);

#define DEBUG_ON() GPIO_SetBits(GPIOF,GPIO_Pin_14);
#define DEBUG_OFF() GPIO_ResetBits(GPIOF,GPIO_Pin_14);

#endif