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

#include "DeviceConnector.h"
#include "HVR_hmd.h"
#include "internal.h"
#include "HVR_Common.h"
#include "VRCalibration.h"
#include "FirmwareVersion.h"
#include "ITrackedDevice.h"
#include <windows.h>

class DataRecoder
{
public:
	DataRecoder();
	~DataRecoder();
	bool save_to_file(int type, char* bf, int size);
	bool startplayback();
	bool startrec();

private:
	void deamon();
	std::mutex mtx;
	FILE* fp;
	bool readmode;
	std::thread t1;
	char filename[0xff];
};

DataRecoder::DataRecoder()
{
	char *path;
	size_t len = 0;;

	strcpy_s(filename, "recdata.dat");
	_dupenv_s(&path, &len, "localappdata");

	if (len > 0)
	{
		sprintf_s(filename, "%s\\HYPEREAL\\recdata.dat", path);
		free(path);
	}
	readmode = true;
	fp = NULL;
}

DataRecoder::~DataRecoder()
{
	if (fp != NULL)
		fclose(fp);
}

uint64_t getSystemTime();

//////////////////////////////////////////////////////////////////////////
//
DataRecoder rec;

bool DeviceConnetor::binit = false;

const char* SubDev_toString(SubDev sd)
{
	switch (sd)
	{
	case SubDev::HMD0:
		return "HMD0";
	case LightHouse0:
		return "LightHouse0";
	case LightHouse1:
		return "LightHouse1";
	}
	return "null-type";
}

int32_t hmd0_ImuReport(IMUReport *report)
{
	if (!DeviceConnetor::binit)
		return 0;

	DeviceConnetor *pdc = DeviceConnetor::get_instance();

	if (!pdc->usb_dev_hmd0_firmware_match)
		return 0;

	pdc->cbstat.hmd0_ImuReport_cnt++;

	DeviceCB hmdcb = pdc->get_cb_by_subtype(SubDev::HMD0);

	if (hmdcb)
	{
		rec.save_to_file(0, (char*)report, sizeof(IMUReport));
		hmdcb(IMU_RAWDATA, (char*)report, sizeof(IMUReport));
	}

	return 0;
}
int32_t hmd0_GetHmdVersionCallback(FirmwareVersion *report)
{
	if (!DeviceConnetor::binit)
		return 0;

	DeviceConnetor *pdc = DeviceConnetor::get_instance();

	DeviceCB hmdcb = pdc->get_cb_by_subtype(SubDev::HMD0);

	if (hmdcb)
		hmdcb(HMD_VERSION, (char*)report, sizeof(FirmwareVersion));
}
int32_t hmd0_LightReport(LightSensorReport *report)
{
	if (!DeviceConnetor::binit)
		return 0;

	DeviceConnetor *pdc = DeviceConnetor::get_instance();
	if (!pdc->usb_dev_hmd0_firmware_match)
		return 0;

	pdc->cbstat.hmd0_LightReport_cnt++;

	DeviceCB hmdcb = pdc->get_cb_by_subtype(SubDev::HMD0);

	if (hmdcb)
	{
		rec.save_to_file(1, (char*)report, sizeof(LightSensorReport));
		hmdcb(PACKET_TYPE::LIGHTSENSOR_RAWDATA, (char*)report, sizeof(LightSensorReport));
	}

	return 0;
}

//! Callback function type: handle LighthouseInfoReport
int32_t LighthouseInfoReportcb(LighthouseInfoReport *report)
{
	//no callback at this time
	int _internal_lighthouse_id = report->LighthouseID;
	SubDev lighthouse_id;

	if (!DeviceConnetor::binit)
		return 0;

	DeviceConnetor *pdc = DeviceConnetor::get_instance();
	pdc->cbstat.LighthouseInfoReportcb_cnt++;

	if (_internal_lighthouse_id == 0)
	{
		lighthouse_id = SubDev::LightHouse0;
	}
	else if (_internal_lighthouse_id == 1)
	{
		lighthouse_id = SubDev::LightHouse1;
	}

	DeviceCB licb = pdc->get_cb_by_subtype(lighthouse_id);
	if (licb)
	{
		rec.save_to_file(5, (char*)report, sizeof(LighthouseInfoReport));

		licb(INTERNAL_PKT_TYPE::LIGHTHOUSE, (char*)report, sizeof(LighthouseInfoReport));
	}

	return 0;
}

int32_t OnGetLighthouseSyncReportcb(LighthouseSyncReport *report)
{
	if (!DeviceConnetor::binit)
		return 0;

	DeviceConnetor *pdc = DeviceConnetor::get_instance();

	pdc->cbstat.LighthouseSyncReportcb_cnt++;
	report->ScanNumber &= 0x00ff;

	rec.save_to_file(6, (char*)report, sizeof(LighthouseSyncReport));

	DeviceCB cb = pdc->get_cb_by_subtype(SubDev::HMD0);
	if (cb)
		cb(PACKET_TYPE::LIGHTHOUSE_SYNC, (char*)report, sizeof(LighthouseSyncReport));

	return 0;
}

void hmd_hotplug(int event, USB_HWID usb_dev)
{
	DeviceConnetor* pDC = DeviceConnetor::get_instance();
	debug_print("hmd0 connect %d\n", event);
	if (event == 1 && !pDC->usb_dev_hmd0_open_flag)
	{
		pDC->hmd0 = new HVR_Hmd(hmd0_ImuReport, hmd0_LightReport, LighthouseInfoReportcb, OnGetLighthouseSyncReportcb, hmd0_GetHmdVersionCallback);

		if (pDC->hmd0->Open("HR-MK1-HMD") == 0)
		{
			//Open succeed
			pDC->usb_dev_hmd0_open_flag = true;

			FirmwareVersion *pVer = &pDC->hmd0->McuVersion;
			if (pVer){
				pDC->hmd0_hardware_version = 0x0 << 16 | pVer->Major << 8 | pVer->Minor;

				pDC->lowlayers[usb_dev.description] = pDC->hmd0_hardware_version;

				if (!Check_HMD_Firmware(pDC->hmd0_hardware_version))
				{
					debug_print("firmware version not correct\n");
					VREvent ev;
					ev.ev_type = EVENT_FIRMWARE_NOT_MATCH;
					ev.subdev = SubDev::HMD0;
					ITrackedDevice::event_buff.write(&ev, 1);
				}
				else
				{
					pDC->usb_dev_hmd0_firmware_match = true;
					debug_print("firmware version is correct\n");
				}
			}
		}
		else
		{
			//Open failed
		}
	}
	else if (event == 0)
	{
		if (pDC->hmd0)
		{
			pDC->hmd0->Close();
			delete pDC->hmd0;
			pDC->hmd0 = NULL;
		}
		pDC->usb_dev_hmd0_open_flag = false;
		pDC->lowlayers.erase(usb_dev.description);
	}
}

void DeviceConnetor::health_monitor()
{
	HALCallbackStatistic _cbstat;

	int cnt = 0;
	int idx = 0;
	while (binit)
	{
		Sleep(1000);
		if (idx == 0)
		{
			_cbstat = cbstat;
		}
		else if (idx == 1)
		{
			cbstat.average_hmd0_ImuReport = (cbstat.hmd0_ImuReport_cnt - _cbstat.hmd0_ImuReport_cnt);
			cbstat.average_hmd0_LightReport = (cbstat.hmd0_LightReport_cnt - _cbstat.hmd0_LightReport_cnt);
			cbstat.average_LighthouseInfoReport = cbstat.LighthouseInfoReportcb_cnt - _cbstat.LighthouseInfoReportcb_cnt;
			cbstat.average_LighthouseSyncReport = cbstat.LighthouseSyncReportcb_cnt - _cbstat.LighthouseSyncReportcb_cnt;
		}
		idx = (cnt++) % 2;
	}
	debug_print("health_monitor exit\n");
}

/*
HR-MK1-HMD
*/

bool DeviceConnetor::init()
{
	VRCalibration vr(NULL);
	cfg _cfg;
	if (vr.load_calibration_cfg(&_cfg))
	{
		if (_cfg.debug_mode == "record")
		{
			rec.startrec();
		}
		else if (_cfg.debug_mode == "playback")
		{
			rec.startplayback();
		}
	}
	hotplugmgr.init();
	hotplugmgr.add_callback(0x5751, 0x0484, "HR-MK1-HMD", hmd_hotplug);

	health_mon_thread = std::thread(&DeviceConnetor::health_monitor, this);

	return true;
}

bool DeviceConnetor::deinit()
{
	hotplugmgr.deinit();
	if (usb_dev_hmd0_open_flag)
	{
		hmd0->Close();
		delete hmd0;
	}
	usb_dev_hmd0_open_flag = false;

	///after binit set to false, the getIntance API can't be access any more.
	binit = false;
	if (health_mon_thread.joinable())
	{
		health_mon_thread.join();
	}
	return true;
}

DeviceConnetor* DeviceConnetor::get_instance()
{
	static DeviceConnetor* deviceconnector_handler = NULL;
	static std::mutex mtx;
	mtx.lock();
	if (!binit)
	{
		binit = true;
		debug_print("into DeviceConnector::get_instance\n");
		deviceconnector_handler = new DeviceConnetor();
		deviceconnector_handler->usb_dev_hmd0_firmware_match = false;
		deviceconnector_handler->usb_dev_hmd0_open_flag = false;
		deviceconnector_handler->init();
	}
	mtx.unlock();
	return deviceconnector_handler;
}

DeviceCB DeviceConnetor::get_cb_by_subtype(SubDev type)
{
	return cbmap[type];
}

bool DeviceConnetor::add_dev_cb(SubDev dev, DeviceCB devcb)
{
	cbmap[dev] = devcb;

	return true;
}
/*
hmd0, hmd1, repeater0, con0, con1, lighthouse0, lighthouse1
*/
std::map<std::string, int> DeviceConnetor::getUSBLowlayers()
{
	return lowlayers;
};

////////////////////////////////////////////////////////////////////////////
bool DataRecoder::save_to_file(int type, char* bf, int size)
{
	static bool savefile = false;

	if (readmode)
		return true;

	std::lock_guard<std::mutex> autolk(mtx);
	if (!savefile)
	{
		fopen_s(&fp, filename, "wb+");
		savefile = true;
	}
	fwrite(&type, sizeof(int), 1, fp);
	uint64_t tt = getSystemTime();
	fwrite(&tt, sizeof(uint64_t), 1, fp);
	fwrite(&size, 1, 4, fp);
	fwrite(bf, 1, size, fp);

	fflush(fp);

	return true;
}

bool DataRecoder::startplayback()
{
	t1 = std::thread(&DataRecoder::deamon, this);
	return true;
}

bool DataRecoder::startrec()
{
	readmode = false;
	return true;
}

void DataRecoder::deamon()
{
	char buffer[4096];
	int bflen;
	fopen_s(&fp, filename, "rb");
	int type;
	uint64_t tt, tt_load;
	int k = 0;
	int sz = fread(&type, sizeof(int), 1, fp);
	//debug_print("read back %d type %d\n", sz, type);
	if (sz == 0)
		return;

	fread(&tt_load, sizeof(uint64_t), 1, fp);
	fread(&bflen, sizeof(int), 1, fp);
	fread(buffer, sizeof(char), bflen, fp);

	uint64_t startts = getSystemTime();
	while (true)
	{
		int sz = fread(&type, sizeof(int), 1, fp);

		if (sz == 0)
			break;
		fread(&tt, sizeof(uint64_t), 1, fp);
		fread(&bflen, sizeof(int), 1, fp);
		fread(buffer, sizeof(char), bflen, fp);

		uint64_t curts = getSystemTime();

		while ((tt - tt_load) > (curts - startts))
		{
			curts = getSystemTime();
			//debug_print("%lld %lld\n", tt - tt_load, curts - startts);
			continue;
		}
		//debug_print("read back %d type %d\n", sz, type);
		switch (type)
		{
		case 0:
			hmd0_ImuReport((IMUReport*)buffer);
			break;
		case 1:
			hmd0_LightReport((LightSensorReport*)buffer);
			break;
		case 5:
			LighthouseInfoReportcb((LighthouseInfoReport*)buffer);
			break;
		case 6:
			OnGetLighthouseSyncReportcb((LighthouseSyncReport*)buffer);
			break;
		default:
			break;
		}
	}
}