/**
  ******************************************************************************
  * @file    hub_simulator.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for calibrate.c.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HUB_SIMULATOR_H_
#define _HUB_SIMULATOR_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
enum HubPacktType {
	PACKET_TYPE_SENSOR = 0,
	PACKET_TYPE_BATTERY = 1,
	PACKET_TYPE_CUBE_VER = 2,
	PACKET_TYPE_CMD_DOWM = 3,
	PACKET_TYPE_CMD_UP = 4,
	PACKET_TYPE_SYNC = 5
};

typedef struct {
		uint16_t version;
		uint8_t bat;
		uint8_t id;
} gamepad_battery_report_type;

typedef struct {
		uint8_t lighthouse_id;
		uint8_t axis_sync;
		uint8_t version;
		uint32_t error;
} lighthouse_version_report_type;

#pragma pack (1)
typedef struct  {
	uint8_t		SensorID;
	uint8_t		TriggerCount;
	uint16_t	MeasureResult;
}LightSensorData;

#pragma pack (1)
typedef struct {
	uint8_t		GamepadID;			//!< Gamepad ID
	uint8_t		Buttons;			//!< Gamepad buttons
	uint16_t	Trigger;			//!< Gamepad trigger
	int16_t		Joystick[2];		        //!< Gamepad joystick x-y axis
	float		QuatIncrement[4];	        //!< Quaternion increment
	uint16_t	ScanNumber;			//!< Laser scan number
	uint8_t		ScanAxis;			//!< Laser scan axis(X1=1, Y1=2, X2=3, Y2=4)
	uint8_t		SampleCount;		        //!< Valid group in LightSensor
	LightSensorData	LightSensor[24];                //!< Light sensor data
}GamepadSensorReport;

/* Exported functions ------------------------------------------------------- */
void hub_remote_data_push(uint8_t * buf);
void hub_remote_data_stack_poll();

#endif/* _HUB_SIMULATOR_H_ */

/************************ (C) COPYRIGHT Hypereal *****END OF FILE****/