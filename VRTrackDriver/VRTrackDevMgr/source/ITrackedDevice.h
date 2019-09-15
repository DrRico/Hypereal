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

#ifndef ITRACKED_DEVICE_H__
#define ITRACKED_DEVICE_H__

#include "DeviceConnector.h"
#include "VRTrackedDeviceMgr.h"
#include "TrackObjectManager.h"
#include "ringbuffer.h"

using namespace Hypereal;
using namespace HYPEREAL;

//! Get device name string
const char* SubDev_toString(SubDev sd);

/*!
	\class ITrackedDevice
*/
class ITrackedDevice
{
public:
	//! Constructed function
	/*!
		\param type SubDev enumeration
		\param system_config_content Pointer to config data
	*/
	ITrackedDevice(SubDev type, const char* system_config_content);

	//! Destructor function
	virtual ~ITrackedDevice()
	{
		debug_print("stop tracking\n");
		object->StopTracking();
	}

	//! Probe the device information 
	/*!
		\param info Pointer to store device info, such as firmware version, battery
		\return
			- true on success
			- false on error
	*/
	virtual bool probe(SubDev_info* info) = 0;

	//! Parse Input-Report raw data
	/*!
		\param type Input-Report type, such as IMU, Lightsensor
		\param buffer Pointer to Input-Report buffer
		\param len Input-Report size
		\return always true
	*/
	virtual bool raw_data_parser(int type, char* buffer, int len){ return true; };

	//! Get the device's position data in VR
	/*!
		\param ts Absolute time, in microsecond
		\param pos Pointer to store the position data
		\return
			- true on success
			- false on error
	*/
	virtual bool get_position(TimeStamp ts, VRPosition *pos){ return true; };

	//! Check if the device is active
	/*!
		\return
			- true if connected
			- false if disconnected
	*/
	virtual bool is_connected();

	//! Get system time in microsecond
	/*!
		\return current system time in us
	*/
	uint64_t get_system_time();

	//! Dump dubug info
	virtual void dump()
	{
		if (is_connected())
		{
			std::cout << "subdev type:" << std::setw(15) << SubDev_toString(subdev_type)
				<< ", is connected:(" << is_connected()
				<< "), is enable:" << is_enable
				<< ", rx cnt:" << rx_cnt
				<< " [byte]" << std::endl;
		}
	}

	//! Get the device other information
	/*!
		For HMD, the device information is IPD.
		For Lighthouse, see LightHouse_Sensor struct.
		\param pbuffer Pointer to store the device information data
		\return
			- true on success
			- false on error
	*/
	virtual bool get_sensor_data(void* pbuffer){ return true; };

	//! Set or clear device enable flag
	/*!
		\param en, true if you want to enable, false if disable
		\return
			- true if enable
			- false if disable
	*/
	bool set_enable(bool en);

	//! Get current enable status
	/*!
		\return
			- true if enable
			- false if disable
	*/
	bool get_enable();

	//! Device type
	const SubDev subdev_type;

	//! Ringbuffer to store hotplug event
	static ringbuffer<VREvent> event_buff;

	//! Flag to indicate if firmware version was got
	bool   b_set_version;

	//! Flag to indicate if calibration was done
	static  bool  is_calibration_done;

	//! Connection timeout, unit: microsecond(us)
	int     connection_timeout;

private:
	//! Set the last packet timestamp for connect timeout
	/*!
		\return the last packet timestamp in microsecond
	*/
	uint64_t set_last_timestamp();

	//! Start track thread of TrackObjectManager
	/*!
		\return always true
	*/
	bool     check_start_track_cond();

	//! Flag to indicate if the track is already started
	bool     is_track_start;

	//! Device connector handler
	DeviceConnetor* devconn_handle;

	//! Device enable/disable flag
	bool       is_enable;

	//! Device firmware version
	uint32_t   hardware_version;

	//! Received byte count
	uint32_t   rx_cnt;

	//! Mutex to protect check_start_track_cond() call
	std::mutex  mtx;

public:
	//! Device internal ID
	int internal_id;

	//! Track object
	TrackObjectManager *object;

	//! Last packet timestamp
	uint64_t   last_pkt_time;
};

/*!
	\class VRLightHouse
	\brief Derived class of ITrackedDevice, for Lighthouse devices
*/
class VRLightHouse :public ITrackedDevice
{
public:
	//! Constructed function
	/*!
		\param type SubDev enumeration for Lighthouse A or B
		\param system_config_content Pointer to config data
	*/
	VRLightHouse(SubDev type, const char* system_config_content);

	//! Probe the Lighthouse device information 
	/*!
		\param info Pointer to store lighthouse info, such as firmware version, battery
		\return
			- true on success
			- false on error
	*/
	virtual bool probe(SubDev_info* dev_info);

	//! Parse Lighthouse Info-Report
	/*!
		\param type Input-Report type, such as IMU, Lightsensor
		\param rawdata Pointer to Input-Report buffer
		\param len Input-Report size
		\return always true
	*/
	virtual bool raw_data_parser(int type, char* rawdata, int len);

	//! Get Lighthouses' position data in VR
	/*!
		\param ts Absolute time, in microsecond
		\param pos Pointer to store the position data
		\return
			- true on success
			- false on error
	*/
	virtual bool get_position(TimeStamp ts, VRPosition *pos);

	//! Get Lighthouses' other information
	/*!
		see LightHouse_Sensor struct.
		\param pbuffer Pointer to store the device information data
		\return
			- true on success
			- false on error
	*/
	bool get_sensor_data(void* pbuffer);

private:
	//! Laser axis synchronization info
	uint8_t		AxisSync;

	//! Error coefficient
	uint32_t	ErrorCoefficient;

	//! Device firmware version
	uint32_t    version;
};
#endif //ITRACKED_DEVICE_H__