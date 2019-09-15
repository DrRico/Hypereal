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

#include <windows.h>
#include "VRTrackedDeviceMgr.h"
#include "ITrackedDevice.h"
#include "VRHMD.h"
#include "VRCalibration.h"

#ifdef WIN32  
#include <io.h>                      //C (Windows)    access  
#else  
#include <unistd.h>                  //C (Linux)      access     
#endif  

using namespace Hypereal;

class VRLightHouse;

static int cnt = 0;

namespace Hypereal{

	class VRTrackedDeviceMgrImpl
	{
	public:
		/// Device related
		HVR_RC_CODE  Device_init();
		HVR_RC_CODE Device_deinit();

		bool Device_probe_subdev(SubDev subdev, SubDev_info* info);
		bool Device_is_connected(SubDev subdev);

		bool Device_set_enable(SubDev subdev, bool en);
		bool Device_get_enable(SubDev subdev);

		TimeStamp Device_get_current_time();
		static TimeStampOffset Device_stamps_per_sec();

		bool Device_get_positionv2(SubDev subdev, TimeStampOffset ts, VRPosition* postion);
		bool Device_get_positionv3(SubDev subdev, VRPosition* position, IMUSensorData* data);

		bool poll_event(VREvent* event);

		bool HMD_get_data(SubDev hmd_id, HMD_Sensor* p_hmd_sensor);

		bool HMD_get_imu_data(SubDev hmd_id, RAWIMUData *rawdata, IMUSensorData *imu_data);

		bool Device_set_init_calibration_enable_sync();

		bool initCalibrationDaemon();

		bool is_initCalibration_done(bool* calibration_rc, InitCalibrationData& data);

		bool Device_set_init_calibration_enable();

		std::map<std::string, int> getUSBlowlayers();

		char* dump();

	protected:

	private:
		std::map<SubDev, ITrackedDevice*> subdev_map;
		bool  reCalibration;
		bool  async_cali;
		std::thread  t1;
		InitCalibrationData icd;

	public:
		bool enable_calibration;
		bool calibration_rc;
		bool thread_flag;
	};
}

char* get_system_user_dir();

std::map<std::string, int> VRTrackedDeviceMgrImpl::getUSBlowlayers()
{
	DeviceConnetor* pdc = DeviceConnetor::get_instance();

	return pdc->getUSBLowlayers();
}
bool VRTrackedDeviceMgrImpl::Device_set_init_calibration_enable()
{
	calibration_rc = false;

	if (t1.joinable())
		t1.join();

	thread_flag = false;
	t1 = std::thread(&VRTrackedDeviceMgrImpl::initCalibrationDaemon, this);

	return true;
}

bool VRTrackedDeviceMgrImpl::initCalibrationDaemon()
{
	thread_flag = true;
	bool rc = false;
	char calibration_data_file[1024];
	sprintf_s(calibration_data_file, "%s\Calibration.txt", get_system_user_dir());

	/*
	 *	 async wait and calibration
	 */
	async_cali = true;
	int cnt = 0;
	while (async_cali && cnt++ <= 6)
	{

		if (enable_calibration && Device_is_connected(SubDev::HMD0) && (Device_is_connected(SubDev::LightHouse0) && Device_is_connected(SubDev::LightHouse1)))
		{
			debug_print("wait ok,  start calibration\n");

			VRHMD* phmd0 = (VRHMD*)subdev_map[SubDev::HMD0];
			if (phmd0)
			{
				rc = phmd0->calibration(icd);
				calibration_rc = rc;
			}
			break;
		}
		Sleep(1000);
	}
	debug_print("-----initcalibration exit\n");
	thread_flag = false;
	return rc;
}

bool Hypereal::VRTrackedDeviceMgrImpl::is_initCalibration_done(bool* calibration_rc, InitCalibrationData& data)
{
	if (calibration_rc != NULL)
	{
		*calibration_rc = this->calibration_rc;
		data = icd;
	}

	return !thread_flag;
}

bool VRTrackedDeviceMgrImpl::Device_set_init_calibration_enable_sync()
{
	enable_calibration = true;
	if (t1.joinable())
	{
		t1.join();
	}

	return true;
}

HVR_RC_CODE VRTrackedDeviceMgrImpl::Device_init()
{
	static bool  createobject = false;

	DeviceConnetor* pdc = DeviceConnetor::get_instance();

	enable_calibration = false;
	if (pdc->binit)
	{
		subdev_map[SubDev::HMD0] = new VRHMD(SubDev::HMD0, NULL);
		subdev_map[SubDev::LightHouse0] = new VRLightHouse(SubDev::LightHouse0, NULL);
		subdev_map[SubDev::LightHouse1] = new VRLightHouse(SubDev::LightHouse1, NULL);

		createobject = true;
	}
	else
	{
		return HVRERROR;
	}

	cfg _cfg;
	VRCalibration vrcali(NULL);
	if (0 == _access(vrcali.get_cfg_filename(), 0))   //! json cfg file is exist
	{
		//try parse json file 
		if (!vrcali.load_calibration_cfg(&_cfg)) //! file is crrupt
		{
			return HVR_CONFIG_FILE_CRRUPT;
		}
		else //! json file is ok
		{
			if (_cfg.mode == "0")
			{
				ITrackedDevice::is_calibration_done = true;
			}
			else
			{
				if (!vrcali.load_init_calibration(&icd)) //! load init calibration fail
				{
					return HVRNO_CALIBRATION;
				}
				else
				{
					VRHMD* phmd0 = (VRHMD*)subdev_map[SubDev::HMD0];
					if (phmd0)
						calibration_rc = phmd0->calibration(icd);
				}
			}

		}
	}
	else
		return HVRNO_CALIBRATION;	//! json cfg file is not exist

	cnt++;

	return HVRSUCCESS;
}

HVR_RC_CODE VRTrackedDeviceMgrImpl::Device_deinit()
{
	debug_print("enter %s\n", __FUNCTION__);
	async_cali = false;
	if (t1.joinable())
		t1.join();

	debug_print("calibration thread exit\n");

	DeviceConnetor* pdc = DeviceConnetor::get_instance();
	pdc->deinit();

	std::map<SubDev, ITrackedDevice*>::iterator it;
	for (it = subdev_map.begin(); it != subdev_map.end(); ++it)
	{
		ITrackedDevice* pdev = it->second;
		delete pdev;
	}

	debug_print("leave %s\n", __FUNCTION__);

	return HVRSUCCESS;
}

bool VRTrackedDeviceMgrImpl::Device_probe_subdev(SubDev subdev, SubDev_info* info)
{
	ITrackedDevice* pDevice = subdev_map[subdev];

	if (pDevice && pDevice->is_connected())
		return pDevice->probe(info);

	return false;
}

bool VRTrackedDeviceMgrImpl::Device_is_connected(SubDev subdev)
{
	ITrackedDevice* pDevice = subdev_map[subdev];

	if (pDevice)
		return pDevice->is_connected();

	return false;
}

bool VRTrackedDeviceMgrImpl::Device_set_enable(SubDev subdev, bool en)
{
	ITrackedDevice* pDevice = subdev_map[subdev];

	if (pDevice)
		return pDevice->set_enable(en);

	return false;
}


bool VRTrackedDeviceMgrImpl::Device_get_enable(SubDev subdev)
{
	ITrackedDevice* pDevice = subdev_map[subdev];

	if (pDevice)
		return pDevice->get_enable();

	return false;
}

TimeStamp VRTrackedDeviceMgrImpl::Device_get_current_time()
{
	std::map<SubDev, ITrackedDevice*>::iterator it;
	for (it = subdev_map.begin(); it != subdev_map.end(); ++it)
	{
		ITrackedDevice* pdev = it->second;
		if (pdev && pdev->is_connected())
			return pdev->get_system_time();
	}
	return -1;
}

TimeStampOffset VRTrackedDeviceMgrImpl::Device_stamps_per_sec()
{
	return (uint64_t)1e6;
}

bool VRTrackedDeviceMgrImpl::poll_event(VREvent* event)
{
	if (ITrackedDevice::event_buff.getOccupied() != 0)
	{
		int ret = ITrackedDevice::event_buff.read(event, 1);
		if (ret != 0)
			return true;
	}
	else
		event->ev_type = EVENT_NONE;

	return true;

}
bool VRTrackedDeviceMgrImpl::Device_get_positionv3(SubDev subdev, VRPosition* position, IMUSensorData* data)
{
	bool rc = false;
	//////////////////////////////////////////////////////////////////////////
	//1. get current position 
	ITrackedDevice* pDevice = subdev_map[subdev];
	TimeStamp _ts = Device_get_current_time();

	if (pDevice)
		rc = pDevice->get_position(_ts, position);

	//////////////////////////////////////////////////////////////////////////
	//2. fill data 
	memset(data, 0, sizeof(IMUSensorData));
	//fill sensor data
	switch (subdev)
	{
	case SubDev::HMD0:
	{
		RAWIMUData rawdata;
		HMD_get_imu_data(subdev, &rawdata, data);
		break;
	}
	default:
		break;
	}

	return rc;
}

bool VRTrackedDeviceMgrImpl::Device_get_positionv2(SubDev subdev, TimeStampOffset ts, VRPosition* postion)
{
	ITrackedDevice* pDevice = subdev_map[subdev];
	TimeStamp _ts = Device_get_current_time() + ts;

	if (pDevice)
		return pDevice->get_position(_ts, postion);

	return false;
}
bool VRTrackedDeviceMgrImpl::HMD_get_data(SubDev hmd_id, HMD_Sensor* p_hmd_sensor)
{
	ITrackedDevice* pDevice = subdev_map[SubDev::HMD0];

	if (pDevice)
		return pDevice->get_sensor_data(p_hmd_sensor);

	return false;
}

char* VRTrackedDeviceMgrImpl::dump()
{
	DeviceConnetor* pdc = DeviceConnetor::get_instance();
	char *dcdump = pdc->dump();
	debug_print("%s\n", dcdump);

	debug_print("-------------------------------------------------\n");
	std::map<SubDev, ITrackedDevice*>::iterator it;
	for (it = subdev_map.begin(); it != subdev_map.end(); ++it)
	{
		ITrackedDevice* pdev = it->second;
		if (pdev)
			pdev->dump();
	}

	return dcdump;
}

bool VRTrackedDeviceMgrImpl::HMD_get_imu_data(SubDev hmd_id, RAWIMUData *rawdata, IMUSensorData *imu_data)
{
	VRHMD* phmd = (VRHMD*)subdev_map[SubDev::HMD0];
	if (phmd)
		return phmd->get_raw_sensor_data(rawdata, imu_data);

	return false;
}

//impl done
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//export class

VRTrackedDeviceMgr::VRTrackedDeviceMgr()
{
	vr_trackeddevicemgr_impl = new VRTrackedDeviceMgrImpl();
}

VRTrackedDeviceMgr::~VRTrackedDeviceMgr()
{
	delete vr_trackeddevicemgr_impl;
}

int setup_async(VRTrackedDeviceMgr *_pMgr);

/// Device related
HVR_RC_CODE VRTrackedDeviceMgr::Device_init()
{
#if 0
	setup_async(this);
#endif
	return vr_trackeddevicemgr_impl->Device_init();
}

bool VRTrackedDeviceMgr::Device_probe_subdev(SubDev subdev, SubDev_info* info)
{
	return vr_trackeddevicemgr_impl->Device_probe_subdev(subdev, info);
}



bool VRTrackedDeviceMgr::Device_is_connected(SubDev subdev)
{
	return vr_trackeddevicemgr_impl->Device_is_connected(subdev);
}

bool VRTrackedDeviceMgr::Device_set_enable(SubDev subdev, bool en)
{
	return vr_trackeddevicemgr_impl->Device_set_enable(subdev, en);
}


/// sync
TimeStamp VRTrackedDeviceMgr::Device_get_current_time()
{
	return vr_trackeddevicemgr_impl->Device_get_current_time();
}
TimeStampOffset VRTrackedDeviceMgr::Device_stamps_per_sec()
{
	return VRTrackedDeviceMgrImpl::Device_stamps_per_sec();
}


bool VRTrackedDeviceMgr::HMD_get_data(SubDev hmd_id, HMD_Sensor* p_hmd_sensor)
{
	return vr_trackeddevicemgr_impl->HMD_get_data(hmd_id, p_hmd_sensor);
}


bool VRTrackedDeviceMgr::Device_get_position(SubDev subdev, TimeStamp ts, VRPosition* position)
{
	TimeStampOffset _ts = ts - vr_trackeddevicemgr_impl->Device_get_current_time();

	return vr_trackeddevicemgr_impl->Device_get_positionv2(subdev, _ts, position);
}
bool VRTrackedDeviceMgr::Device_get_positionv2(SubDev subdev, TimeStampOffset ts, VRPosition* postion)
{
	return vr_trackeddevicemgr_impl->Device_get_positionv2(subdev, ts, postion);
}

bool VRTrackedDeviceMgr::Device_get_positionv3(SubDev subdev, VRPosition* position, IMUSensorData* data)
{
	return vr_trackeddevicemgr_impl->Device_get_positionv3(subdev, position, data);
}

bool VRTrackedDeviceMgr::LightHouse_get_data(SubDev lighthouse_id, LightHouse_Sensor* p_lighthouse_sensor)
{
	return true;
}

//////////////////////////////////////////////////////////////////////////
//debug only
bool VRTrackedDeviceMgr::HMD_get_imu_data(SubDev hmd_id, RAWIMUData *rawdata, IMUSensorData *imu_data)
{
	return vr_trackeddevicemgr_impl->HMD_get_imu_data(hmd_id, rawdata, imu_data);
};


RAWSensorCaliData VRTrackedDeviceMgr::HMD_get_cali_offset(void)
{
	RAWSensorCaliData data;
	memset(&data, 0, sizeof(RAWSensorCaliData));
	return data;
};

void VRTrackedDeviceMgr::HMD_set_cali_offset(RAWSensorCaliData* cali_data)
{
	return;
};


char* VRTrackedDeviceMgr::dump()
{
	return vr_trackeddevicemgr_impl->dump();
}

HVR_RC_CODE VRTrackedDeviceMgr::Device_deinit()
{
	return vr_trackeddevicemgr_impl->Device_deinit();
}

bool VRTrackedDeviceMgr::poll_event(VREvent* event)
{
	return vr_trackeddevicemgr_impl->poll_event(event);
}

bool VRTrackedDeviceMgr::Device_set_init_calibration_enable()
{
	vr_trackeddevicemgr_impl->enable_calibration = true;
	vr_trackeddevicemgr_impl->Device_set_init_calibration_enable();
	return true;
}

bool Hypereal::VRTrackedDeviceMgr::is_initCalibration_done(bool& calibration_rc, InitCalibrationData& icd)
{
	bool ret;
	ret = vr_trackeddevicemgr_impl->is_initCalibration_done(&calibration_rc, icd);
	return ret;
}

std::map<std::string, int> VRTrackedDeviceMgr::getUSBlowlayers()
{
	return vr_trackeddevicemgr_impl->getUSBlowlayers();
}