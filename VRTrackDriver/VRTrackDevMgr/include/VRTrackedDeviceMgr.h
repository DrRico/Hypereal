/*
 * The MIT License (MIT)
 * Copyright (c) 2017 Shanghai Chai Ming Huang Info&Tech Co，Ltd
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

#pragma once
/*
 * Date: May 3, 2016
 * Author: qianchang
 */

#ifndef VRTRACKED_DEVICE_MGR_H__
#define VRTRACKED_DEVICE_MGR_H__

#include <cstdint>
#include <string>
#include <functional>
#include <iostream>
#include <vector>
#include <map>
#include <list>

#ifdef VRTRACKDRVDLL_EXPORTS
#define VRTRACKDRVDLL_API __declspec(dllexport) 
#else
#define VRTRACKDRVDLL_API __declspec(dllimport) 
#endif

typedef int lowlayer_hw_id;

namespace Hypereal {

	//! VR position result struct
	typedef union {
		struct {
			double i, j, k, o;             //!< Orientation Quaternion
			double x, y, z, _reserved;     //!< Center Location (m)
		} var;
		double data[8];
	}VRPositionResult;

	//! VR position struct
	typedef struct {
		VRPositionResult P;				//!< VR position result
		bool quaternion_valid;			//!< flag to indicate if the quat is valid
		bool centerlocation_valid;		//!< flag to indicate if the location is valid
	}VRPosition;

	//! HMD extra information struct
	typedef struct {
		uint32_t   InterpupillaryDistance;	//!< IPD
		uint32_t   proximity_switch;		//!< proximity_switch, reserved for future
	}HMD_Sensor;

	//! Lighthouse extra infomation struct
	typedef struct {
		uint8_t    axis_sync;				//!< Synchronous-axis information of Lighthosue
		uint32_t   error_coefficient;		//!< Error coefficient of Lighthouse
	}LightHouse_Sensor;

	//! Timestamp in millisecond (ms)
	typedef uint64_t TimeStamp;

	//! Timestamp offset in millisecond (ms)
	typedef uint64_t TimeStampOffset;

	//! SubDev enumeration
	typedef enum {
		HMD0 = 0x0000,		//!< HMD device
		LightHouse0 = 0x0200,		//!< Lighthouse A device
		LightHouse1 = 0x0201,		//!< Lighthouse B device
	}SubDev;

	//! SubDev information struct
	typedef struct {
		uint32_t   hardware_version;	//!< Device firmware version
		uint16_t   battery;				//!< Device battery information (unavailable)
	}SubDev_info;

	//! TODO: IMU calibration data struct
	typedef struct {
		int16_t gyro[3];		//!< 3-axis gyroscope
		int16_t accel[3];		//!< 3-axis accelerometer
	}RAWSensorCaliData;

	//! IMU raw data struct
	typedef struct {
		int16_t gyro[3];		//!< 3-axis gyroscope
		int16_t accel[3];		//!< 3-axis accelerometer
	}RAWIMUData;

	//! IMU data which have been recalculated
	typedef struct {
		double angular_velocity[3];		//!< 3-axis angular velocity
		double angular_acceleration[3];	//!< 3-axis angular acceleration
	}IMUSensorData;

	//! TODO: not cleared
	typedef std::function<void(SubDev subdev, char* buffer, int len)> RawDataCB;

	//! Event type enumeration, TODO: may not need any more
	typedef enum {
		EVENT_NONE = 0x10,
		EVENT_FIRMWARE_NOT_MATCH,
	}VREvent_type;

	//! Event type bind with devices
	typedef	struct {
		SubDev subdev;
		VREvent_type ev_type;
	}VREvent;

	//! Return code enumeration
	typedef enum {
		HVRERROR = -1,
		HVR_CONFIG_FILE_CRRUPT = -2,
		HVRSUCCESS = 0,
		HVRNO_CALIBRATION,
	}HVR_RC_CODE;

	//! Init-calibration data struct
	typedef struct {
		double LHTranslation[2][3];
		double LHRotation[2][9];
	}InitCalibrationData;


	/*!
		\class VRTrackedDeviceMgr
		\brief Manager of all VR tracked devices

		Application always need to create a VRTrackedDeviceMgr object to get tracking data.
		*/
	class VRTRACKDRVDLL_API VRTrackedDeviceMgr
	{
	public:
		//! Constructed function
		VRTrackedDeviceMgr();

		//! Destructor function
		virtual ~VRTrackedDeviceMgr();

		//****************************************
		// Device relevant
		//****************************************

		//! VRTrackedDeviceMgr init API
		/*!
			\return HVR_RC_CODE
			*/
		virtual HVR_RC_CODE  Device_init();

		//! VRTrackedDeviceMgr deinit API
		/*!
			\return HVR_RC_CODE
			*/
		virtual HVR_RC_CODE  Device_deinit();

		//! Subdev probe to get version and battery information
		/*!
			\param subdev subdev enumeration
			\param info subdev_info structure
			\return
			- true on success
			- false on error
			*/
		virtual bool Device_probe_subdev(SubDev subdev, SubDev_info* info);

		//! CHeck the subdev is connected or not
		/*!
			\param subdev subdev enumeration
			\return
			- true if device is connected
			- false if device is not connected
			*/
		virtual bool Device_is_connected(SubDev subdev);

		//! Enable or disable the specified subdev
		/*
			\param subdev subdev enumeration
			\param en flag of enable or disable
			\return
			- true on success
			- false on error
			*/
		virtual bool Device_set_enable(SubDev subdev, bool en);

		//! Get the status(enable/disable) of the specified subdev
		/*!
			\param subdev subdev enumeration
			\return
			- true if the subdev is enabled
			- false if the subdev is disabled
			*/
		bool Device_get_enable(SubDev subdev);

		//! Start ini-calibration
		/*
			\return
			- true on success
			- false on error
			*/
		virtual bool Device_set_init_calibration_enable();

		//! Get the calibration status to check if init-calibration is done or not
		/*!
			\param calibration_rc calibration status
			\param icd calibration data
			\return
			- true if the init-calibration was done
			- false if the init-calibration was not finished yet
			*/
		virtual bool is_initCalibration_done(bool& calibration_rc, InitCalibrationData& icd);

		/// sync
		//! Get current timestamp
		/*!
			\return current time in millisecond(ms)
			*/
		virtual TimeStamp Device_get_current_time();

		//! TODO
		static TimeStampOffset Device_stamps_per_sec();

		//****************************************
		// Event relevant
		//****************************************

		//! Query event buffer
		/*!
			\param event event data which is returned in parameter list
			\return
			- true on success
			- false on error
			*/
		virtual bool poll_event(VREvent* event);

		//****************************************
		// Position relevant
		//****************************************

		//! Get position data with absolution time
		/*!
			\param subdev SubDev enumeration
			\param ts absolution time
			\param position VRPosition data structure
			\return
			- true on success
			- false on error
			*/
		virtual bool Device_get_position(SubDev subdev, TimeStamp ts, VRPosition* position);

		//! Get position data with relative time
		/*!
			\param subdev SubDev enumeration
			\param tsoff relative time offset
			\param position VRPosition data structure
			\return
			- true on success
			- false on error
			*/
		virtual bool Device_get_positionv2(SubDev subdev, TimeStampOffset tsoff, VRPosition* position);

		//! Get position data with more IMU values
		/*!
			\param subdev SubDev enumeration
			\param position VRPosition data structure
			\Param data IMUSensorData data structure
			\return
			- true on success
			- false on error
			*/
		bool Device_get_positionv3(SubDev subdev, VRPosition* position, IMUSensorData* data);

		//****************************************
		// HMD relevant
		//****************************************

		//! Get HMD extra information, including IPD and proximity switch
		/*!
			\param hmd_id SubDev enumeration, we only have one HMD device
			\param p_hmd_sensor Pointer to store HMD extra information data
			\return
			- true on success
			- false on error
			*/
		virtual bool HMD_get_data(SubDev hmd_id, HMD_Sensor* p_hmd_sensor);

		//****************************************
		// LIghthouse relevant
		//****************************************

		//! Get Lighthouse extra information, including sync axis and error coefficient
		/*!
			\param lighthouse_id SubDev enumeration, we have two lighthouse devices
			\param p_lighthouse_sensor Pointer to store Lighthouse extra information data
			\return
			- true on success
			- false on error
			*/
		virtual bool LightHouse_get_data(SubDev lighthouse_id, LightHouse_Sensor* p_lighthouse_sensor);

		//****************************************
		// Miscellaneous, for debug purpose
		//****************************************

		//! Get detail IMU data of HMD
		/*!
			\param hmd_id SubDev enumeration, we only have one HMD device
			\param rawdata Pointer to store the original IMU data
			\param data_convert Pointer to store the angular velocity and angular acceleration
			\return
			- true on success
			- false on error
			*/
		virtual bool HMD_get_imu_data(SubDev hmd_id, RAWIMUData *rawdata, IMUSensorData *data_convert);

		//! Get IMU calibration data of HMD
		/*!
			\return
			- Valid pointer to store IMU calibration data if succeed
			- NULL pointer if failed
			*/
		virtual RAWSensorCaliData HMD_get_cali_offset(void);

		//! Set IMU calibration data of HMD
		/*!
			This will write the cali_data into flash memory on the HMD

			\param cali_data Pointer to store the calibration data
			*/
		virtual void HMD_set_cali_offset(RAWSensorCaliData* cali_data);

		//! Dump debug information to the console
		/*!
			return Pointer to the debug information
			*/
		virtual char* dump();

		//! Get low layer handler mapping
		/*!
			return Low layer handler mapping
			*/
		std::map<std::string, int> getUSBlowlayers();

	private:
		//! pointer to implementation of VRTrackedDeviceMgrImpl
		class VRTrackedDeviceMgrImpl* vr_trackeddevicemgr_impl;
	};
}
#endif //VRTRACKED_DEVICE_MGR_H__