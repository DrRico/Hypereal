#include "stm32f10x.h"
#include "led.h"

void ir_led_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE );
  
  GPIO_InitStructure.GPIO_Pin =  0xffff;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  0xffff;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  return;
}

void ir_led_set(unsigned char *arg)
{
      GPIO_Write(GPIOD,arg[0]+(((unsigned int)arg[1])<<8));
      GPIO_Write(GPIOE,arg[2]+(((unsigned int)arg[3])<<8));
      
      return;
}

//too old version
/*
void ir_led_set(unsigned char group,unsigned int data)
{
  uint16_t temp;

  switch(group)
  {
  case LED_P5:
    {
      temp = GPIO_ReadOutputData(GPIOB);
      temp &= 0xff0f;
      data = temp |( data<<4);
      GPIO_Write(GPIOB,data);
      break;
    }
  case LED_P7:
    {
      temp = GPIO_ReadOutputData(GPIOB);
      temp &= 0xf0ff;
      data = temp |( data<<8);
      GPIO_Write(GPIOB,data);
      break;
    }
  case LED_P8:
    {
      temp = GPIO_ReadOutputData(GPIOC);
      temp &= 0xfff0;
      data = temp |( data);
      GPIO_Write(GPIOC,data);
      
      break;
    }
  case LED_P9:
    {
      temp = GPIO_ReadOutputData(GPIOC);
      temp &= 0xff0f;
      data = temp |( data<<4);
      GPIO_Write(GPIOC,data);
      
      break;
    }
  case LED_P10:
    {
      temp = GPIO_ReadOutputData(GPIOC);
      temp &= 0xf0ff;
      data = temp |( data<<8);
      GPIO_Write(GPIOC,data);
      
      break;
    }
  
  default: break;
  }
}
*/