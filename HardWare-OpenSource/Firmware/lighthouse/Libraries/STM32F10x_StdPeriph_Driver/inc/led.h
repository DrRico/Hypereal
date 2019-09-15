#ifndef _IR_LED_H_
#define _IR_LED_H_

#define LED_P5 5
#define LED_P7 7
#define LED_P8 8
#define LED_P9 9
#define LED_P10 10

void ir_led_init();
void ir_led_set(unsigned char *arg);
//void ir_led_set(unsigned char group,unsigned int data);

#endif