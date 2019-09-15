/**
  ******************************************************************************
  * @file    hub_simulator.c
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   This file provides firmware functions to receive data from gamepad 
  *          and push these data to usb stack.
  ******************************************************************************

  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "hub_simulator.h"
#include "usbd_trans.h"
#include "fpga_coprocesser.h"
#include "wireless_phase_control.h"

/* Private macro -------------------------------------------------------------*/
gamepad_battery_report_type gamepad_battery_report[2];
lighthouse_version_report_type lighthouse_version_report[2];
GamepadSensorReport gamepad_report;

/* Private define ------------------------------------------------------------*/
#define gamepad_num                             2
#define gamepad_packet_num                      8
/* Private variables ---------------------------------------------------------*/
unsigned char remote_flush_flag[gamepad_num] = {0,0};
uint16_t cubeRxCount[gamepad_num] = { 0, 0 };
uint16_t joystickRxCount[gamepad_num] = { 0, 0 };

remote_data_type remote_data_stack[gamepad_packet_num];
uint8_t remote_data_stack_count = 0;
uint32_t remote_data_push_count = 0;

/**
  * @brief  Analytic frames from gamepad 
  * @param  None
  * @retval None
  */
void hub_remote_data_push(uint8_t * buf)
{
	uint8_t *ptr_8;
	uint16_t *ptr_16;
        uint8_t i;
	
	if (remote_data_stack_count < gamepad_packet_num)
	{
		remote_data_stack[remote_data_stack_count].remote_id = buf[0]>>7;
		remote_data_stack[remote_data_stack_count].button = (buf[0]&0x7f)|0X80;
		remote_data_stack[remote_data_stack_count].trigger = buf[1];
		remote_data_stack[remote_data_stack_count].js_x = buf[2];
		remote_data_stack[remote_data_stack_count].js_y = buf[3];
		
		ptr_8 = (uint8_t *)(void * )(&(remote_data_stack[remote_data_stack_count].qi[0]));
		for ( i = 0;i < 12;i++)
		{
			ptr_8[i] = buf[4+i];
		}

		for (i = 0;i < 3;i++)
		{
			remote_data_stack[remote_data_stack_count].ls_en[i] = buf[16+i];
		}
		
		ptr_16 = (uint16_t *)(buf + 19);
		for (i = 0;i < 16;i++)
		{
			remote_data_stack[remote_data_stack_count].ls[i] = ptr_16[i];
		}
		
		remote_data_stack[remote_data_stack_count].bat = buf[51] & 0xfc;
		remote_data_stack[remote_data_stack_count].axis = buf[51]& 0x03;
		remote_data_stack[remote_data_stack_count].ver = buf[52];

		remote_data_stack_count += 1;
	}
	
	uint8_t gamepad_id = buf[0]>>7;
	
	gamepad_battery_report[gamepad_id].version = ((buf[52]>>4)) + (((uint16_t)(buf[52] & 0x0f))<<8);
	gamepad_battery_report[gamepad_id].id = gamepad_id;
	gamepad_battery_report[gamepad_id].bat = buf[51]&0xfc;
	
	remote_flush_flag[gamepad_id] = 1;
	
}

/**
  * @brief  Push data from gamepad to USB stack. 
  * @param  None
  * @retval None
  */
void hub_remote_data_stack_poll()
{
	uint8_t valid_count = 0;
	
	if (remote_data_stack_count)
	{
              gamepad_report.GamepadID = remote_data_stack[remote_data_stack_count-1].remote_id;
              gamepad_report.Buttons = remote_data_stack[remote_data_stack_count-1].button;
              gamepad_report.Trigger = remote_data_stack[remote_data_stack_count-1].trigger;
              gamepad_report.Joystick[0] = remote_data_stack[remote_data_stack_count-1].js_x;
              gamepad_report.Joystick[1] = remote_data_stack[remote_data_stack_count-1].js_y;
              for (uint8_t i=0;i<3;i++)
              {
                      gamepad_report.QuatIncrement[i] = remote_data_stack[remote_data_stack_count-1].qi[i];
              }
              gamepad_report.QuatIncrement[3] = 0;
              
              gamepad_report.ScanNumber = scan_num;
              gamepad_report.ScanAxis = remote_data_stack[remote_data_stack_count-1].axis+1;
              
              for (uint8_t i = 0;i < 24;i++)
              {
                      if (remote_data_stack[remote_data_stack_count-1].ls_en[i/8] & (0x01<<(i%8)))
                      {
                              if (valid_count < 16)
                              {
                                      gamepad_report.LightSensor[valid_count].MeasureResult = remote_data_stack[remote_data_stack_count-1].ls[valid_count];
                                      gamepad_report.LightSensor[valid_count].SensorID = i;
                                      gamepad_report.LightSensor[valid_count].TriggerCount = 1;
                                      valid_count++;
                              }
                      }
              }
              gamepad_report.SampleCount = valid_count;
              
              TX_push((uint8_t *)(&gamepad_report),sizeof(gamepad_report) - sizeof(LightSensorData)* (24-valid_count) ,0x05);
              remote_data_push_count ++;
              
              remote_data_stack_count --;
	}
}

/************************ (C) COPYRIGHT Hypereal *****END OF FILE****/