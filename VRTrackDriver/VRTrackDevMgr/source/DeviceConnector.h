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

#ifndef DEVICE_CONNECTOR_H__
#define DEVICE_CONNECTOR_H__

#include <iostream>
#include <functional>
#include <iomanip>
#include <map>
#include <mutex>
#include <cstdint>
#include "VRTrackedDeviceMgr.h"
#include "HVR_hmd.h"
#include "internal.h"

using namespace Hypereal;

//! Callback function type
typedef std::function<void(int buffertype, char* buffer, int len)> DeviceCB;

typedef enum
{
	//internal
	KEY_PAD = 10,
	BATT,
	LIGHTHOUSE,
	HMD_VERSION,
}INTERNAL_PKT_TYPE;

//! Get device name string
const char* SubDev_toString(SubDev sd);

//! Input-Report statistical data struct
typedef struct{
	int hmd0_ImuReport_cnt;				//!< HMD IMU report count
	int average_hmd0_ImuReport;			//!< HMD IMU report frequency
	int hmd0_LightReport_cnt;			//!< HMD light report count
	int average_hmd0_LightReport;		//!< HMD light report frequency
	int LighthouseInfoReportcb_cnt;		//!< LTH info report count
	int average_LighthouseInfoReport;	//!< LTH info report frequency
	int LighthouseSyncReportcb_cnt;		//!< LTH sync report count
	int average_LighthouseSyncReport;	//!< LTH sync report frequency
}HALCallbackStatistic;

/*!
	\class DeviceConnetor
	\brief To connect all VR devices of Hypereal
*/
class DeviceConnetor{
public:
	//! Constructed function
	DeviceConnetor() { };

	//! Destructor function
	~DeviceConnetor() { };

	//! Get the object
	/*!
		\return the device connector handler
	*/
	static DeviceConnetor* get_instance();

	//! Get the corresponding callback function of each device
	/*!
		\param type SubDev enumeration
		\return The corresponding callback function
	*/
	DeviceCB get_cb_by_subtype(SubDev type);

	//! Binding the device and its callback function
	/*!
		\param dev SubDev enumeration
		\param devcb The corresponding callback function
	*/
	bool add_dev_cb(SubDev dev, DeviceCB devcb);

	//! Get the timestamp of PC, unit: us
	/*!
		\return Timestamp in microsecond
	*/
	uint64_t get_system_time();

	//! Flag to indicate if the object was initialized
	/*!
		\return
			- true means already done
			- false means not initialized yet
	*/
	static bool binit;

	//! Object initialize function
	/*!
		\return always true
	*/
	bool init();

	//! Object de-initialize function
	/*!
		\return always true
	*/
	bool deinit();

	//! Get low layer handler mapping
	/*!
		return Low layer handler mapping
	*/
	std::map<std::string, int> getUSBLowlayers();

	//! Callback function map
	std::map<SubDev, DeviceCB> cbmap;

	//! Dump Input-Report statistical data
	char* dump()
	{
		static char output[4096];
		int len = 0;
		std::map<std::string, int> dev_list = getUSBLowlayers();
		len += sprintf((char*)(output + len), "low layer open flags:  hmd0: %d\n", usb_dev_hmd0_open_flag);
		len += sprintf((char*)(output + len), "Callback Status: HMD-IMU (500:%d pkt/s) [%s], HMD-Light (120:%d pkt/s) [%s], LTH-Info %d, LTH-Sync (30:%d pkt/s) [%s]\n",
			cbstat.average_hmd0_ImuReport, \
			cbstat.average_hmd0_ImuReport < 500 * 0.95 ? "pkt lost!" : "normal",
			cbstat.average_hmd0_LightReport, \
			cbstat.average_hmd0_LightReport < 120 * 0.95 ? "pkt lost!" : "normal",
			cbstat.LighthouseInfoReportcb_cnt, \
			cbstat.average_LighthouseSyncReport, \
			cbstat.average_LighthouseSyncReport < 30 * 0.95 ? "pkt lost!" : "normal" \
			);
		return output;
	}

	//! HVR_Hmd object
	HVR_Hmd *hmd0;

	//! HMD device firmware version
	uint32_t hmd0_hardware_version;

	//! Device Input-Report statistical data monitor
	void health_monitor(void);

	//! Monitor thread
	std::thread health_mon_thread;

	//! Input-Report statistical data
	HALCallbackStatistic  cbstat;

	//! Device hotplug manager
	HVR_UsbHotPlugMgr hotplugmgr;

	//! Flag to indicate if HMD device was opened
	bool usb_dev_hmd0_open_flag;

	//! Flag to indicate if HMD device firmware version matched
	bool usb_dev_hmd0_firmware_match;

	//! Low layer handler mapping
	std::map<std::string, int> lowlayers;
};
#endif //DEVICE_CONNECTOR_H__