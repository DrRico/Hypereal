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

#ifndef HVR_HMD_H__
#define HVR_HMD_H__

#include <stdint.h>
#include <mutex>
#include <thread>
#include <list>
#include <vector>
#include <condition_variable>
#include <atomic>
#include "HVR_Common.h"

#define PROD_STR_HMD	"HR-MK1-HMD"

//! IMU raw data type
typedef struct IMURawData_ {
	int16_t Gyro[3];	//!< 3-axis gyro data
	int16_t Acce[3];	//!< 3-axis acce data
}IMURawData;

//! IMU report structure
typedef struct IMUReport_ {
	uint32_t		Timestamp;				//!< Timestamp of MCU
	IMURawData		Samples[2];				//!< Two samples of IMU data
	int16_t			MagneticField[3];		//!< 3-axis magnetometer data
	uint16_t		Temperature;			//!< Temperature reported by MCU
	uint16_t		InterpupillaryDistance;	//!< IPD
	uint8_t			ProximitySwitch;		//!< Reserved for future
	uint8_t			Reserve;				//!< Reserved for 4-Byte align
}IMUReport;

//! Max sample count of light-sensor data in HMD
const uint8_t HmdLightSensorNumber = 32;

//! Light sensor report structure
typedef struct LightSensorReport_ {
	uint16_t	ScanNumber;		//!< Scan number from lighthouse
	uint8_t		ScanAxis;		//!< Scan axis id, see also ScanAxisID enum
	uint8_t		SampleCount;	//!< Indicate how many sets data in LightSensor[]
	LightSensorData	LightSensor[HmdLightSensorNumber];	//!< lighthouse scan result
}LightSensorReport;

//! Lighthouse information report type
typedef struct LighthouseInfoReport_ {
	uint8_t		LighthouseID;		//!< Lighthouse ID
	uint8_t		AxisSync;			//!< Laser axis synchronization info
	FirmwareVersion	Version;		//!< Lighthouse MCU version
	uint32_t	ErrorCoefficient;	//!< Error coefficient
}LighthouseInfoReport;

//! Lighthouse synchronization report type
typedef struct LighthouseSyncReport_ {
	uint16_t ScanNumber;	//!< ScanNumber send to HMD
}LighthouseSyncReport;

//! Temperature correction struct type
typedef struct TempCorrection_{
	float Slop[3];		//!< Slope in math
	float Intercept[3];	//!< Intercept in math
}TempCorrection;

//! Data structure of IMU calibration
typedef struct CalibrationData_ {
	TempCorrection	GyroCorrect;	//!< Gyro temperature drift correction
	TempCorrection	AcceCorrect;	//!< Acce temperature drift correction
}CalibrationData;

//! NV area only support 32 Bytes
typedef struct NoneVolatileData_ {
	uint8_t buf[32];
}NoneVolatileData;

//! Callback function type: handle the imu-report
typedef int32_t(*OnGetImuReportCallback)(IMUReport *report);

//! Callback function type: handle light-sensor-report
typedef int32_t(*OnGetLightReportCallback)(LightSensorReport *report);

//! Callback function type: handle hmd-version
typedef int32_t(*OnGetHmdVersionCallback)(FirmwareVersion *report);

//! Callback function type: handle lighthouse-information-report
typedef int32_t(*OnGetLthInfoCallback)(LighthouseInfoReport *report);

//! Callback function type: handle lighthouse-sync-report
typedef int32_t(*OnGetLthSyncCallback)(LighthouseSyncReport *report);

//! USB device hardware information structure
typedef struct 
{
	int pid;					//!< USB device Product ID
	int vid;					//!< USB device Verdor ID
	std::string description;	//!< USB device product description string
}USB_HWID;

//! USB device hotplug event callback function type
typedef void(*pusb_event_callback)(int event, USB_HWID usb_dev);

/*!
	\class HVR_UsbHotPlugMgr
	\brief class of usb devices hotplug manager

	Used by class VRTrackedDeviceMgr.
*/
class HVR_UsbHotPlugMgr
{
public:
	//! Constructed function
	HVR_UsbHotPlugMgr()
	{
		binit = true;
		usb_dev_list.clear();
	};

	//! Destructor function
	~HVR_UsbHotPlugMgr(){};
	
	//! Register hotplug event callback for a USB device
	/*!
		\param pid PID of the USB device
		\param vid VID of the USB device
		\param product_string Product description string of the USB device
		return always be 0
	*/
	bool add_callback(int pid, int vid, std::string product_string, pusb_event_callback callback);

	//! Background thread for USB device hotplug
	void daemon();

	//! Object initialize
	bool init();

	//! Object de-initialize
	bool deinit();

	//! Get current active USB devcie list
	std::list<USB_HWID> get_active_hw();

private:
	//! A single USB device status structure 
	typedef struct
	{
		USB_HWID hwid;					//!< USB device information
		pusb_event_callback callback;	//!< Corresponding hotplug event callback
		int curr_status;				//!< Connect status: 0 = disconnect, 1 = connect
	}USB_dev;

	std::vector<USB_dev> usb_dev_list;	//!< USB device list
	std::atomic<bool> binit;			//!< flag for background thread
	std::thread t1;						//!< background thread
};

/*!
	\class HVR_Hmd
	\brief class of HMD

	Application should create a HVR_Hmd object before communicating with HMD device.
*/
class HVR_Hmd
{
public:
	//! Constructed function
	/*!
		Using background thread and callback function
		\param onGetImuReport callback to handle hmd-imu-report
		\param onGetLightReport callback to handle hmd-lightsensor-report
		\param onGetLthInfoReprot callback to handle lth-info-report
		\param onGetLthSyncReport callback to handle lth-sync-report
		\param onGetHmdVersion callback to handle version-report from HMD
	*/
	HVR_Hmd(OnGetImuReportCallback onGetImuReport,
		OnGetLightReportCallback onGetLightReport,
		OnGetLthInfoCallback onGetLthInfoReprot,
		OnGetLthSyncCallback onGetLthSyncReport,
		OnGetHmdVersionCallback onGetHmdVersion);

	//! Destructor function
	~HVR_Hmd();

	//! HMD Packet Type
	enum HmdPacketType {
		PACKET_TYPE_IMU_REPORT			= 0x0,	//!< IMU report from MCU
		PACKET_TYPE_LIGHT_SENSOR_REPORT = 0x1,	//!< light sensor report from MCU
		PACKET_TYPE_HMD_CMD_DOWN		= 0x3,	//!< Command packet to MCU
		PACKET_TYPE_HMD_CMD_UP			= 0x4,	//!< Response from MCU
		PACKET_TYPE_LTH_INFO_REPORT		= 0x5,	//!< LTH information report from MCU
		PACKET_TYPE_LTH_SYNC_REPORT		= 0x6,	//!< LTH synchronization report from MCU
	};

	//! Command ID of control report
	enum CmdID {
		GET_IMU_CALI	= 0x3,	//!< get imu calibration data
		SET_IMU_CALI	= 0x4,	//!< set imu calibration data
		GET_NV_DATA		= 0x5,	//!< get none-volatile data
		SET_NV_DATA		= 0x6,	//!< set none-volatile data
		GET_FW_VER		= 0x7,	//!< fetch firmware version
	};

	//! Lighthouse scan axis id
	enum ScanAxisID {
		SCAN_AXIS_X1 = 0x1,		//!< lighthouse 1 X-axis
		SCAN_AXIS_Y1 = 0x2,		//!< lighthouse 1 Y-axis
		SCAN_AXIS_X2 = 0x3,		//!< lighthouse 2 X-axis
		SCAN_AXIS_Y2 = 0x4		//!< lighthouse 2 Y-axis
	};

	//! Parameter type of background thread
	typedef struct ThreadArgv_ {
		HVR_Hmd					*pHmd;			//!< Pointer to HVR_Hmd object
		std::mutex				Mutex;			//!< Mutex
		std::condition_variable	CondVar;		//!< Condition variable
		bool					bKillThread;	//!< Flag to kill background thread
		FirmwareVersion			*pMcuVer;		//!< MCU version
		uint8_t					RegValue;		//!< Register value
		NoneVolatileData		*pNVData;		//!< Pointer to NV Data
		OnGetImuReportCallback		CbHmdImu;	//!< Callback to handle imu-report
		OnGetLightReportCallback	CbHmdLight;	//!< Callback to handle light-sensor-report
		OnGetLthInfoCallback		CbLthInfo;	//!< Callback to handle lth-info-report
		OnGetLthSyncCallback		CbLthSync;	//!< Callback to handle lth-sync-report
		OnGetHmdVersionCallback		CbHmdVer;	//!< Callback to handle HMD version
		CalibrationData			*pCaliData;		//!< Pointer to Calibration data
	}ThreadArgv;

	//! Open HMD device
	/*!
		start the background thread
		\return
			- 0 on success
			- -1 on error
	*/
	int32_t Open();

	//! Open HMD device with specified product string
	/*!
		start the background thread
		\return
			- 0 on success
			- -1 on error
	*/
	int32_t Open(char *productString);

	//! Close HMD device
	/*!
		kill the background thread
	*/
	void Close();

	//! Set calibration data
	/*!
		\param caliData pointer to calibration data
		\return
			- 0 on success
			- others on error
	*/
	int32_t SetCalibrationData(CalibrationData *caliData);

	//! Get calibration data
	/*!
		\return
			- valid pointer on success
			- NULL on error
	*/
	CalibrationData *GetCalibrationData();

	//! Set none-volatile data
	/*!
		\param data pointer to none-volatile data
		\return
			- 0 on success
			- others on error
	*/
	int32_t SetNoneVolatileData(NoneVolatileData *data);

	//! Get none-volatile data
	/*!
		\return
			- valid pointer on success
			- NULL on error
	*/
	NoneVolatileData *GetNoneVolatileData();

	//! Fetch firmware version of MCU
	/*!
		\return
			- valid pointer on success
			- NULL on error
	*/
	FirmwareVersion *GetFirmwareVersion();

	//! Buffer size of a bulk operation
	const uint32_t BULK_SIZE = 64;

	//! To store MCU version
	FirmwareVersion McuVersion;

private:
	//! Timeout when waiting for response from MCU. Unit: ms.
	const uint32_t ResponseTimeout = 500;

	//! To store NV data
	NoneVolatileData NvData;

	//! To store Calibration data
	CalibrationData CaliData;

	//! Pointer to background thread
	std::thread *pThreadWorker;

	//! Parameter for background thread
	ThreadArgv ThreadWorkerArgv;

	//! Background thread
	static void WorkerFunc(ThreadArgv &argv);

	//! Mutex for bulk write and bulk read
	HANDLE WR_Mutex;

	//! Mutex timeout
	const DWORD WR_Mutex_Timeout = 500;	//unit: ms

private:
	//! USB vendor id of HMD
	const uint16_t UsbVid = 0x0484;

	//! USB product id of HMD
	const uint16_t UsbPid = 0x5751;

	//! Endpoint in
	const uint8_t EpIn = 0x82;

	//! Endpoint out
	const uint8_t EpOut = 0x02;

	//! USB configuration index
	const uint8_t UsbConfigIndex = 1;

	//! USB interface index
	const uint8_t UsbInterfaceIndex = 0;
};

#endif	// HVR_HMD_H__