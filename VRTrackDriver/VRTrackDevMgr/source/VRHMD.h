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

#ifndef VRHMD_H__
#define VRHMD_H__

#include "ITrackedDevice.h"
#include "VRTrackedDeviceMgr.h"
#include "ringbuffer.h"
#include "internal.h"

/*!
	\class VRHMD
	\brief Derived class of ITrackedDevice, for HMD device
*/
class VRHMD :public ITrackedDevice
{
public:
	//! Constructed function
	/*!
		\param type SubDev enumeration for HMD
		\param system_config_content Pointer to config data
	*/
	VRHMD(SubDev type, const char* system_config_content);

	//! Probe the HMD device information 
	/*!
		\param info Pointer to store device info, such as firmware version, battery
		\return
			- true on success
			- false on error
	*/
	virtual bool probe(SubDev_info* dev_info);

	//! Parse HMD Input-Report
	/*!
		\param type Input-Report type, such as IMU, Lightsensor
		\param rawdata Pointer to Input-Report buffer
		\param len Input-Report size
		\return always true
	*/
	virtual bool raw_data_parser(int type, char* rawdata, int len);

	//! Get HMD's position data in VR
	/*!
		\param ts Absolute time, in microsecond
		\param pos Pointer to store the position data
		\return
			- true on success
			- false on error
	*/
	virtual bool get_position(TimeStamp ts, VRPosition *pos);

	//! Get HMD's other information
	/*!
		For HMD, the device information is IPD.
		\param pbuffer Pointer to store the device information data
		\return
			- true on success
			- false on error
	*/
	bool get_sensor_data(void* pbuffer);

	//! Get detail IMU data of HMD
	/*!
		\param raw_imu_data Pointer to store the original IMU data
		\param data_convert Pointer to store the angular velocity and angular acceleration
		\return
			- true on success
			- false on error
	*/
	bool get_raw_sensor_data(RAWIMUData *raw_imu_data, IMUSensorData *data_convert);

	//! Do calibration
	/*!
		\param icd To store the calibration data
		\return
			- true on success
			- false on error
	*/
	bool calibration(InitCalibrationData& icd);

private:
	//! Ringbuffer to store HMD IMU-Report
	ringbuffer <imu_packet>			hmd_imu_buffer;

	//! Ringbuffer to store HMD Light-Report
	ringbuffer <lightsensor_packet>	hmd_lightsensor_buffer;

	//! HMD's IPD and proximity switch data
	HMD_Sensor		sensor_data;

	//! HMD's IMU raw data
	RAWIMUData		_imu_raw_data;

	//! HMD's angular velocity and angular acceleration
	IMUSensorData	imu_data;

	//! Mutex to protect raw data copy
	std::mutex		mtx;
};
#endif //VRHMD_H__