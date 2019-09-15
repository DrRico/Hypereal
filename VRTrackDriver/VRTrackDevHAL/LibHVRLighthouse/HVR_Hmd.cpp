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

#include "HVR_Hmd.h"
#include "LIBUSB_Device.h"

//! Pointer to LIBUSB_Device object
static LIBUSB_Device *pUsbDevice;

HVR_Hmd::HVR_Hmd(OnGetImuReportCallback onGetImuReport,
	OnGetLightReportCallback onGetLightReport,
	OnGetLthInfoCallback onGetLthInfoReprot,
	OnGetLthSyncCallback onGetLthSyncReport,
	OnGetHmdVersionCallback onGetHmdVersion)
{
	pThreadWorker = NULL;
	ThreadWorkerArgv.CbHmdImu = onGetImuReport;
	ThreadWorkerArgv.CbHmdLight = onGetLightReport;
	ThreadWorkerArgv.CbLthInfo = onGetLthInfoReprot;
	ThreadWorkerArgv.CbLthSync = onGetLthSyncReport;
	ThreadWorkerArgv.CbHmdVer = onGetHmdVersion;
	ThreadWorkerArgv.pMcuVer = &McuVersion;
	ThreadWorkerArgv.pCaliData = &CaliData;
	ThreadWorkerArgv.pNVData = &NvData;
}

HVR_Hmd::~HVR_Hmd()
{
}

void HVR_Hmd::WorkerFunc(ThreadArgv &argv)
{
	int ret;
	char buffer[RX_PACKET_MAX_SIZE];
	DataPacketHead *header;
	DataPacketTail *tail;

	while (!argv.bKillThread){
		WaitForSingleObject(argv.pHmd->WR_Mutex, argv.pHmd->WR_Mutex_Timeout);
		ret = pUsbDevice->BulkRead(buffer, argv.pHmd->BULK_SIZE);
		ReleaseMutex(argv.pHmd->WR_Mutex);
		if (ret < 0)
			continue;

		// Check packet Header
		header = (DataPacketHead *)buffer;
		if (header->MagicNumber != MAGIC_NUMBER_HEAD)
			continue;
		if (header->Type != HmdPacketType::PACKET_TYPE_IMU_REPORT &&
			header->Type != HmdPacketType::PACKET_TYPE_LIGHT_SENSOR_REPORT &&
			header->Type != HmdPacketType::PACKET_TYPE_HMD_CMD_UP &&
			header->Type != HmdPacketType::PACKET_TYPE_LTH_INFO_REPORT &&
			header->Type != HmdPacketType::PACKET_TYPE_LTH_SYNC_REPORT)
			continue;

		if ((header->Length + sizeof(DataPacketHead) + sizeof(DataPacketTail)) > RX_PACKET_MAX_SIZE)
			continue;

		// Check payload data length. Since one bulk read operation only get 64Bytes from MCU, 
		// so if one packet is bigger than (64-9=54)Bytes, we need do more bulk read operation
		// to compelete this whole packet
		int total_packet = (header->Length + sizeof(DataPacketHead) + sizeof(DataPacketTail)) / argv.pHmd->BULK_SIZE + 1;
		if (total_packet > 1){
			bool read_failed = false;
			// Read One or more packet
			for (int i = 1; i < total_packet; i++){
				WaitForSingleObject(argv.pHmd->WR_Mutex, argv.pHmd->WR_Mutex_Timeout);
				ret = pUsbDevice->BulkRead(buffer + i * argv.pHmd->BULK_SIZE, argv.pHmd->BULK_SIZE);
				ReleaseMutex(argv.pHmd->WR_Mutex);
				if (ret < 0){
					read_failed = true;
					break;
				}
			}
			if (read_failed)
				continue;
		}

		// Check the packet tail
		tail = (DataPacketTail *)(buffer + sizeof(DataPacketHead) + header->Length);
		if (tail->MagicNumber != MAGIC_NUMBER_TAIL)
			continue;

		// Check the packet crc
		uint8_t crc = 0;
		for (uint32_t i = sizeof(DataPacketHead); i < sizeof(DataPacketHead) + header->Length; i++)
			crc += buffer[i];
		if (crc != tail->Crc)
			continue;

		// Do the right operation according to the packet type
		switch (header->Type){
		case HmdPacketType::PACKET_TYPE_IMU_REPORT:
		{
			if (argv.CbHmdImu)
				argv.CbHmdImu((IMUReport *)(buffer + sizeof(DataPacketHead)));
			break;
		}
		case HmdPacketType::PACKET_TYPE_LIGHT_SENSOR_REPORT:
		{
			if (argv.CbHmdLight)
				argv.CbHmdLight((LightSensorReport *)(buffer + sizeof(DataPacketHead)));
			break;
		}
		case HmdPacketType::PACKET_TYPE_LTH_INFO_REPORT:
		{
			if (argv.CbLthInfo)
				argv.CbLthInfo((LighthouseInfoReport *)(buffer + sizeof(DataPacketHead)));
			break;
		}
		case HmdPacketType::PACKET_TYPE_LTH_SYNC_REPORT:
		{
			if (argv.CbLthSync)
				argv.CbLthSync((LighthouseSyncReport *)(buffer + sizeof(DataPacketHead)));
			break;
		}
		case HmdPacketType::PACKET_TYPE_HMD_CMD_UP:
		{
			// The first Byte of Payload Data is CommandID
			switch (buffer[sizeof(DataPacketHead)]) {
			case CmdID::GET_IMU_CALI:
			{
				memcpy_s(argv.pCaliData, sizeof(CalibrationData), buffer + sizeof(DataPacketHead) + 1, sizeof(CalibrationData));
				argv.CondVar.notify_one();
				break;
			}
			case CmdID::GET_NV_DATA:
			{
				memcpy_s(argv.pNVData, sizeof(NoneVolatileData), buffer + sizeof(DataPacketHead) + 1, sizeof(NoneVolatileData));
				argv.CondVar.notify_one();
				break;
			}
			case CmdID::GET_FW_VER:
			{
				memcpy_s(argv.pMcuVer, sizeof(FirmwareVersion), buffer + sizeof(DataPacketHead) + 1, sizeof(FirmwareVersion));
				argv.CondVar.notify_one();
				if (argv.CbHmdVer)
					argv.CbHmdVer((FirmwareVersion *)(buffer + sizeof(DataPacketHead) + 1));
				break;
			}
			default:
				break;
			}
		}
		default:
			break;
		}
	}
}

int32_t HVR_Hmd::Open()
{
	pUsbDevice = new LIBUSB_Device(UsbVid, UsbPid, EpIn, EpOut, UsbConfigIndex, UsbInterfaceIndex);

	int ret = pUsbDevice->Open();

	if (ret != 0)
		return ret;
	WR_Mutex = CreateMutex(NULL, FALSE, NULL);
	if (WR_Mutex == NULL)
		return -1;
	ThreadWorkerArgv.pHmd = this;
	ThreadWorkerArgv.bKillThread = false;

	if (pThreadWorker == NULL){
		pThreadWorker = new std::thread(WorkerFunc, std::ref(ThreadWorkerArgv));
		HANDLE handle = pThreadWorker->native_handle();
		SetThreadPriority(handle, THREAD_PRIORITY_TIME_CRITICAL);
	}

	Sleep(500);

	//call get firmware version two times
	FirmwareVersion *ver = GetFirmwareVersion();
	if (!ver) {
		GetFirmwareVersion();
	}

	return ret;
}

int32_t HVR_Hmd::Open(char *productString)
{
	if (productString == NULL)
		return -1;

	pUsbDevice = new LIBUSB_Device(UsbVid, UsbPid, EpIn, EpOut, UsbConfigIndex, UsbInterfaceIndex);

	if (pUsbDevice->Open(productString) != 0)
		return -1;

	WR_Mutex = CreateMutex(NULL, FALSE, NULL);
	if (WR_Mutex == NULL)
		return -1;
	ThreadWorkerArgv.pHmd = this;
	ThreadWorkerArgv.bKillThread = false;

	if (pThreadWorker == NULL){
		pThreadWorker = new std::thread(WorkerFunc, std::ref(ThreadWorkerArgv));
		HANDLE handle = pThreadWorker->native_handle();
		SetThreadPriority(handle, THREAD_PRIORITY_HIGHEST);
	}

	Sleep(500);

	//call get firmware version two times
	FirmwareVersion *ver = GetFirmwareVersion();
	if (!ver) {
		GetFirmwareVersion();
	}

	return 0;
}

void HVR_Hmd::Close()
{
	if (pThreadWorker) {
		ThreadWorkerArgv.bKillThread = true;
		pThreadWorker->join();
		delete pThreadWorker;
		pThreadWorker = NULL;
	}

	if (WR_Mutex) {
		CloseHandle(WR_Mutex);
		WR_Mutex = NULL;
	}

	pUsbDevice->Close();
	delete pUsbDevice;
	pUsbDevice = NULL;
}

int32_t HVR_Hmd::SetCalibrationData(CalibrationData *caliData)
{
	uint8_t *cmd_data = new uint8_t[sizeof(CalibrationData) + 1];

	if (!cmd_data)
		return -1;

	cmd_data[0] = CmdID::SET_IMU_CALI;
	memcpy_s(cmd_data + 1, sizeof(CalibrationData), caliData, sizeof(CalibrationData));

	uint16_t total_len = GetPacketTotalLength(sizeof(CalibrationData) + 1);
	char tx_buf[CMD_PACKET_MAX_SIZE];

	if (total_len > CMD_PACKET_MAX_SIZE){
		delete[] cmd_data;
		return -1;
	}

	if (AssemblePacket(tx_buf, PACKET_TYPE_HMD_CMD_DOWN, cmd_data, sizeof(CalibrationData) + 1) != total_len){
		delete[] cmd_data;
		return -1;
	}

	delete[] cmd_data;

	int ret;
	if (total_len > BULK_SIZE) {
		int offset = 0;

		while (total_len != 0) {
			if (total_len >= BULK_SIZE){
				WaitForSingleObject(WR_Mutex, WR_Mutex_Timeout);
				ret = pUsbDevice->BulkWrite(tx_buf + offset * BULK_SIZE, BULK_SIZE);
				ReleaseMutex(WR_Mutex);
				if (ret < 0)
					return ret;
				offset++;
				total_len -= BULK_SIZE;
			}
			else{
				WaitForSingleObject(WR_Mutex, WR_Mutex_Timeout);
				ret = pUsbDevice->BulkWrite(tx_buf + offset * BULK_SIZE, total_len);
				ReleaseMutex(WR_Mutex);
				if (ret < 0)
					return ret;
				total_len -= total_len;
			}
		}
	}
	else{
		WaitForSingleObject(WR_Mutex, WR_Mutex_Timeout);
		ret = pUsbDevice->BulkWrite(tx_buf, total_len);
		ReleaseMutex(WR_Mutex);
	}

	// wait 1 second for MCU to erase flash memory
	Sleep(1000);
	return ret;
}

CalibrationData * HVR_Hmd::GetCalibrationData()
{
	uint8_t cmd_data[1] = { CmdID::GET_IMU_CALI };
	uint16_t total_len = GetPacketTotalLength(sizeof(cmd_data));
	char tx_buf[CMD_PACKET_MAX_SIZE];

	if (total_len > CMD_PACKET_MAX_SIZE)
		return NULL;

	if (AssemblePacket(tx_buf, PACKET_TYPE_HMD_CMD_DOWN, cmd_data, sizeof(cmd_data)) != total_len)
		return NULL;

	WaitForSingleObject(WR_Mutex, WR_Mutex_Timeout);
	int ret = pUsbDevice->BulkWrite(tx_buf, total_len);
	ReleaseMutex(WR_Mutex);
	if (ret < 0)
		return NULL;

	std::unique_lock<std::mutex> lk(ThreadWorkerArgv.Mutex);
	if (ThreadWorkerArgv.CondVar.wait_for(lk, std::chrono::milliseconds(ResponseTimeout)) == std::cv_status::timeout)
		return NULL;
	else
		return &CaliData;
}

int32_t HVR_Hmd::SetNoneVolatileData(NoneVolatileData *data)
{
	uint8_t *cmd_data = new uint8_t[sizeof(NoneVolatileData) + 1];

	if (!cmd_data)
		return -1;

	cmd_data[0] = CmdID::SET_NV_DATA;
	memcpy_s(cmd_data + 1, sizeof(NoneVolatileData), data->buf, sizeof(NoneVolatileData));

	uint16_t total_len = GetPacketTotalLength(sizeof(NoneVolatileData) + 1);
	char tx_buf[CMD_PACKET_MAX_SIZE];

	if (total_len > CMD_PACKET_MAX_SIZE){
		delete[] cmd_data;
		return -1;
	}

	if (AssemblePacket(tx_buf, PACKET_TYPE_HMD_CMD_DOWN, cmd_data, sizeof(NoneVolatileData) + 1) != total_len){
		delete[] cmd_data;
		return -1;
	}

	delete[] cmd_data;
	WaitForSingleObject(WR_Mutex, WR_Mutex_Timeout);
	int ret = pUsbDevice->BulkWrite(tx_buf, total_len);
	ReleaseMutex(WR_Mutex);

	// wait 1 second for MCU to erase flash memory
	Sleep(1000);
	return ret;
}

NoneVolatileData * HVR_Hmd::GetNoneVolatileData()
{
	uint8_t cmd_data[1] = { CmdID::GET_NV_DATA };
	uint16_t total_len = GetPacketTotalLength(sizeof(cmd_data));
	char tx_buf[CMD_PACKET_MAX_SIZE];

	if (total_len > CMD_PACKET_MAX_SIZE)
		return NULL;

	if (AssemblePacket(tx_buf, PACKET_TYPE_HMD_CMD_DOWN, cmd_data, sizeof(cmd_data)) != total_len)
		return NULL;

	WaitForSingleObject(WR_Mutex, WR_Mutex_Timeout);
	int ret = pUsbDevice->BulkWrite(tx_buf, total_len);
	ReleaseMutex(WR_Mutex);
	if (ret < 0)
		return NULL;

	std::unique_lock<std::mutex> lk(ThreadWorkerArgv.Mutex);
	if (ThreadWorkerArgv.CondVar.wait_for(lk, std::chrono::milliseconds(ResponseTimeout)) == std::cv_status::timeout)
		return NULL;
	else
		return &NvData;
}

FirmwareVersion * HVR_Hmd::GetFirmwareVersion()
{
	uint8_t cmd_data[1] = { CmdID::GET_FW_VER };
	uint16_t total_len = GetPacketTotalLength(sizeof(cmd_data));
	char tx_buf[CMD_PACKET_MAX_SIZE];

	if (total_len > CMD_PACKET_MAX_SIZE)
		return NULL;

	if (AssemblePacket(tx_buf, PACKET_TYPE_HMD_CMD_DOWN, cmd_data, sizeof(cmd_data)) != total_len)
		return NULL;

	WaitForSingleObject(WR_Mutex, WR_Mutex_Timeout);
	int ret = pUsbDevice->BulkWrite(tx_buf, total_len);
	ReleaseMutex(WR_Mutex);
	if (ret < 0)
		return NULL;

	std::unique_lock<std::mutex> lk(ThreadWorkerArgv.Mutex);
	if (ThreadWorkerArgv.CondVar.wait_for(lk, std::chrono::milliseconds(ResponseTimeout)) == std::cv_status::timeout)
		return NULL;
	else
		return &McuVersion;
}

bool HVR_UsbHotPlugMgr::add_callback(int pid, int vid, std::string product_string, pusb_event_callback callback)
{
	USB_dev dev;
	dev.hwid.pid = pid;
	dev.hwid.vid = vid;
	dev.hwid.description = product_string;
	dev.callback = callback;
	dev.curr_status = 0;
	usb_dev_list.push_back(dev);

	return 0;
}
std::list<USB_HWID> HVR_UsbHotPlugMgr::get_active_hw()
{
	std::list< USB_HWID > active_list;
	for (USB_dev n : usb_dev_list)
		if (n.curr_status == 1)
			active_list.push_back(n.hwid);

	return active_list;
}

void HVR_UsbHotPlugMgr::daemon()
{
	while (binit)
	{
		bool match = false;

		usb_find_busses();
		usb_find_devices();

		int idx = 0;
		for (USB_dev n : usb_dev_list) {

			struct usb_bus *bus;
			struct usb_device *dev;

			match = false;

			for (bus = usb_get_busses(); bus; bus = bus->next){
				for (dev = bus->devices; dev; dev = dev->next){
					if (n.hwid.pid == dev->descriptor.idProduct && n.hwid.vid == dev->descriptor.idVendor)
						match = true;
				}
			}

			switch (n.curr_status)
			{
			case 0:
				if (match)
				{
					usb_dev_list[idx].curr_status = 1;
					if (n.callback)
						n.callback(1, n.hwid);
				}
				break;
			case 1:
				if (!match)
				{
					usb_dev_list[idx].curr_status = 0;
					if (n.callback)
						n.callback(0, n.hwid);
				}
				break;
			default:
				break;
			}
			idx++;
		}
		Sleep(1500);
	}
}


bool HVR_UsbHotPlugMgr::init()
{
	binit = true;

	t1 = std::thread(&HVR_UsbHotPlugMgr::daemon, this);
	
	return true;
}

bool HVR_UsbHotPlugMgr::deinit()
{
	binit = false;
	if (t1.joinable())
	{
		t1.join();
	}
	debug_print("usb hotpulg thread exit\n");
	return true;
}