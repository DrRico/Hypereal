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
  * @file    fpga_coprocesser.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for calibrate.c.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _FPGA_COPROCESSER_H_
#define _FPGA_COPROCESSER_H_

/* Includes ------------------------------------------------------------------*/
#include "hr_mk1_config.h"
#include "imu_data.h"
/* Exported types ------------------------------------------------------------*/
#pragma pack (1)
typedef struct
{
	uint8_t SensorID;                       //!< Sensor number ID, the same sensor has the potential to be triggered repeatedly 
	uint8_t TriggerCount;                   //!< Evaluation of the results of the scan is good or bad (retention, the current fixed to 1) 
	uint16_t MeasureResult;                 //!< The amount of time the sensor is scanned 
} LightSensorData_type;

#pragma pack (1)
typedef struct
{
	uint16_t ScannedNumber;                 //!< Scan serial number
	uint8_t AxisScanned;                    //!< Scan serial source(1: X from lighthouse A,2: Y from lighthouse A,3: X from lighthouse B,4: Y from lighthouse B)
	uint8_t SensorCount;                    //!< Representation of how many groups of valid data in LightSensor 
	LightSensorData_type LightSensor[32];   //!< data about LightSensor
} measure_result_type;

#pragma pack(1)
typedef struct
{
	uint8_t id;                             //!< 0: light A; 1: lightB;
	signed char comp_x;                     //!< cycle for x-axis
	signed char comp_y;                     //!< cycle for y-axis
	signed short d_x;                       //!< time offset for x-axis
	signed short d_y;	                //!< time offset for y-axis
	uint8_t flush_flag;                     //!< send count
} light_sync_package_type;

typedef struct 
{
	uint8_t remote_id;                      //!< Gamepad ID
	uint8_t button;                         //!< Gamepad buttons
	uint8_t trigger;                        //!< Gamepad trigger
	uint8_t js_x;                           //!< Gamepad joystick x axis
	uint8_t js_y;                           //!< Gamepad joystick y axis
	float qi[4];                            //!< Quaternion increment
	uint8_t ls_en[4];                       //!< laser enable
	uint16_t ls[16];                        //!< time about light induction
	uint8_t bat;                            //!< Battery of gamepad
	uint8_t axis;                           //!< Laser scan axis(X1=1, Y1=2, X2=3, Y2=4)
	uint8_t ver;                            //!< MCU firmware version
}remote_data_type;

/* Exported constants --------------------------------------------------------*/
#define RESULT_DIV_TOWER_SCALE  (10*168/36.0f/32.0f)
#define SPEED_CALC_SCALE1       (3400.0f)
#define SPEED_CALC_SCALE2       (60000.0f)

#define MOTOR_0_TO_110_TIME_IN_RESULT_SCALE_x ((uint32_t)104513)
#define MOTOR_0_TO_110_TIME_IN_RESULT_SCALE_y ((uint32_t)104513)

#define Bank1_SRAM1_ADDR        ((u32)(0x60000000)) 
\
/* Exported macro ------------------------------------------------------------*/
#define FPGA_RESET(x)           {if(x) GPIO_SetBits(GPIOD,GPIO_Pin_2); else GPIO_ResetBits(GPIOD,GPIO_Pin_2);}
#define FPGA_MEASURE(x)         {if(x) GPIO_SetBits(GPIOD,GPIO_Pin_3); else GPIO_ResetBits(GPIOD,GPIO_Pin_3);}
#define FPGA_IF_INT()           (GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6))

#define FPGA_BASE_ADDR          Bank1_SRAM1_ADDR
#define FPGA_READ(x)            *((volatile unsigned short int *)(FPGA_BASE_ADDR +(x<<17)))
#define FPGA_WRITE(addr,data)   {*((volatile unsigned short int *)(FPGA_BASE_ADDR +(addr<<17)))=data;}

/* Exported functions ------------------------------------------------------- */
extern light_sync_package_type light_sync_package[2];	
extern measure_result_type measure_result_buf;
extern measure_result_type measure_result[4];
extern uint8_t measure_result_valid[4];

void fpga_coprocesser_init();
void fpga_poll();
void fpga_light_sync_package_process(uint8_t *buf, light_sync_package_type * package);
void fpga_fix_calc(uint8_t tower_id);

void wp_callback_ax();
void wp_callback_ay();
void wp_callback_bx();
void wp_callback_by();

#endif/* _FPGA_COPROCESSER_H_ */

/*****************************END OF FILE****/
