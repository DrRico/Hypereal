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

#include "VRHMD.h"
#include "HVR_hmd.h"
#include "internal.h"
#include "ringbuffer.h"
#include "VRCalibration.h"
#include <fstream>
#include <assert.h>

using namespace std;

char* get_system_user_dir()
{
	static std::mutex mtx;
	static char* path;
	static char buff[1024];
	size_t len;
	std::lock_guard<mutex> lock(mtx);
	strcpy_s(buff, "");
	_dupenv_s(&path, &len, "localappdata");

	if (len > 0)
	{
		sprintf_s(buff, "%s\\HYPEREAL", path);
		free(path);
	}
	return buff;
}

VRHMD::VRHMD(SubDev type, const char* system_config_content) :ITrackedDevice(type, system_config_content), hmd_imu_buffer(50), hmd_lightsensor_buffer(50)
{
	DeviceConnetor *pdc = DeviceConnetor::get_instance();
	connection_timeout = 1e5;
	set_enable(true);
}

bool VRHMD::calibration(InitCalibrationData& icd)
{
	CalibrationResult ret = CalibrationResult::SUCCESS;
	double LHTranslation[2][3];
	double LHRotation[2][9];

	VRCalibration vrcali(NULL);
	InitCalibrationData tempicd;

	if (!vrcali.load_init_calibration(&tempicd))
	{
		debug_print("start calibration\n");
		ret = object->CalculateLHPose(internal_id, LHTranslation, LHRotation);
		if (ret != CalibrationResult::SUCCESS)
		{
			goto end_entry;
		}
		debug_print("calibration done\n");
	}
	else
	{
		debug_print("Load Calibration File!!!\n");
		memcpy(LHRotation, tempicd.LHRotation, sizeof(LHRotation));
		memcpy(LHTranslation, tempicd.LHTranslation, sizeof(LHTranslation));
	}

end_entry:
	if (ret == CalibrationResult::SUCCESS)
	{
		object->SetLHPose(LHTranslation, LHRotation);
		memcpy(icd.LHRotation, LHRotation, sizeof(LHRotation));
		memcpy(icd.LHTranslation, LHTranslation, sizeof(LHTranslation));
	}

	///set
	ITrackedDevice::is_calibration_done = (ret == CalibrationResult::SUCCESS) ? true : false;
	debug_print("ITrackedDevice::is_calibration_done %d\n", ITrackedDevice::is_calibration_done);

	return (ret == CalibrationResult::SUCCESS) ? true : false;
}
bool VRHMD::raw_data_parser(int type, char* rawdata, int len)
{
	uint64_t current_time = get_system_time();
	static uint64_t last_imu_report_ts = 0;
	static double last_gyrovec[4];
	switch (type)
	{
	case IMU_RAWDATA: /// imu report
	{
		IMUReport* report = (IMUReport*)rawdata;
		assert(sizeof(IMUReport) == len);

		imu_packet _imu_packet;
		_imu_packet.type = IMU_RAWDATA;
		_imu_packet.timestamp = current_time;

		memcpy(_imu_packet._imu.rawdata.Samples, report->Samples, sizeof(report->Samples));
		memcpy(_imu_packet._imu.rawdata.MagneticField, report->MagneticField, sizeof(int16_t) * 3);

		size_t ret = hmd_imu_buffer.write(&_imu_packet, 1);

		sensor_data.InterpupillaryDistance = report->InterpupillaryDistance;
		sensor_data.proximity_switch = report->ProximitySwitch;

		mtx.lock();
		//////////////////////////////////////////////////////////////////////////
		//raw data copy
		memcpy(&_imu_raw_data, &report->Samples[0], sizeof(_imu_raw_data));

		//////////////////////////////////////////////////////////////////////////
		// convert data
		double gyrovec[4];
		gyrovec[0] = _imu_raw_data.gyro[0];
		gyrovec[1] = _imu_raw_data.gyro[1];
		gyrovec[2] = _imu_raw_data.gyro[2];
		object->GetGyroVec(internal_id, gyrovec);

		memcpy(imu_data.angular_velocity, gyrovec, sizeof(double) * 3);
		//////////////////////////////////////////////////////////////////////////
		// acce
		if (last_imu_report_ts != 0) {
			double offset_seconds = (current_time - last_imu_report_ts) / 1e6;
			if (offset_seconds <= 10e-3 && offset_seconds > 0)
			{
				imu_data.angular_acceleration[0] = (gyrovec[0] - last_gyrovec[0]) / offset_seconds;
				imu_data.angular_acceleration[1] = (gyrovec[1] - last_gyrovec[1]) / offset_seconds;
				imu_data.angular_acceleration[2] = (gyrovec[2] - last_gyrovec[2]) / offset_seconds;
			}
		}
		memcpy(last_gyrovec, gyrovec, sizeof(double) * 3);

		last_imu_report_ts = current_time;
		mtx.unlock();
		break;
	}
	case LIGHTSENSOR_RAWDATA: ///light sensor report
	{
		LightSensorReport* report = (LightSensorReport*)rawdata;
		assert(sizeof(LightSensorReport) == len);

		lightsensor_packet _light_packet;
		_light_packet.type = LIGHTSENSOR_RAWDATA;
		_light_packet.timestamp = current_time;

		_light_packet.SampleCount = report->SampleCount;
		_light_packet.ScanAxis = report->ScanAxis;
		_light_packet.ScanNumber = report->ScanNumber;

		memcpy(_light_packet.LightSensor, report->LightSensor, sizeof(report->LightSensor));

		hmd_lightsensor_buffer.write(&_light_packet, 1);

		break;
	}
	case PACKET_TYPE::LIGHTHOUSE_SYNC:
	{
		assert(len == sizeof(LighthouseSyncReport));

		LighthouseSyncReport *preport = (LighthouseSyncReport*)rawdata;
		lightsensor_packet _l_packet;
		_l_packet.timestamp = current_time;
		_l_packet.type = PACKET_TYPE::LIGHTHOUSE_SYNC;
		_l_packet.ScanNumber = preport->ScanNumber;
		hmd_lightsensor_buffer.write(&_l_packet, 1);
		break;
	}
	case HMD_VERSION:
	{
		FirmwareVersion *pver = (FirmwareVersion*)rawdata;
		if (pver != NULL && !b_set_version)
		{
			debug_print("get hmd version callback (%x %x)\n", pver->Major, pver->Minor);
			if (object)
			{
				HYPEREAL::TrackObjectFlagsEnum flags;
				flags = HYPEREAL::TrackObjectFlagsEnum::DEFUALT;
				VRCalibration vrc(NULL);
				cfg _cfg;
				vrc.load_calibration_cfg(&_cfg);

				flags = HYPEREAL::TrackObjectFlagsEnum::DEFUALT;
				if (_cfg.mode == "1")
				{
					flags = HYPEREAL::TrackObjectFlagsEnum::DEFUALT;
				}
				else if (_cfg.mode == "0")
				{
					flags = HYPEREAL::TrackObjectFlagsEnum::DEFUALT;
				}
				HYPEREAL::TrackObjectDeviceType type;
				if (subdev_type == SubDev::HMD0){
					type = HYPEREAL::TrackObjectDeviceType::HMD;
				}

				internal_id = object->AddTrackObject(type, &hmd_imu_buffer, &hmd_lightsensor_buffer, flags, pver->Major, pver->Minor);
				debug_print("internal id = %d\n", internal_id);
			}
			b_set_version = true;
		}
		break;
	}
	default:
		break;
	}
	return true;
}

bool VRHMD::get_position(TimeStamp ts, VRPosition *pos)
{
	double translation[3];
	double quat[4];

	//x,y,z,w
	//x,y,z
	if (is_connected() && internal_id >= 0 && get_enable())
	{
		PoseResultFlag rc = object->GetPose(internal_id, quat, translation, ts);

		pos->quaternion_valid = rc & PoseResultFlag::quatValid ? true : false;
		pos->centerlocation_valid = rc & PoseResultFlag::transValid ? true : false;

		if (pos->quaternion_valid)
		{
			pos->P.var.i = quat[0];
			pos->P.var.j = quat[1];
			pos->P.var.k = quat[2];
			pos->P.var.o = quat[3];
		}
		/// Convert mm to m
		if (pos->centerlocation_valid)
		{
			pos->P.var.x = translation[0] / 1000;
			pos->P.var.y = translation[1] / 1000;
			pos->P.var.z = translation[2] / 1000;
		}

		return (pos->centerlocation_valid || pos->quaternion_valid);
	}

	return false;
}

bool VRHMD::probe(SubDev_info* dev_info)
{
	if (is_connected())
	{
		DeviceConnetor *pdc = DeviceConnetor::get_instance();

		if (subdev_type == SubDev::HMD0)
			dev_info->hardware_version = pdc->hmd0_hardware_version;

		dev_info->battery = 100;
		return true;
	}

	return false;
}

bool VRHMD::get_sensor_data(void* pbuffer)
{
	if (is_connected() && get_enable())
	{
		uint32_t temp = sensor_data.InterpupillaryDistance;

		if (temp < 597)
			temp = 597;

		if (temp > 3178)
			temp = 3178;

		sensor_data.InterpupillaryDistance = (temp + 15405) / 258.1;

		memcpy(pbuffer, &sensor_data, sizeof(sensor_data));
		return true;
	}

	return false;
}

bool VRHMD::get_raw_sensor_data(RAWIMUData *raw_imu_data, IMUSensorData *imu_data)
{
	if (is_connected() && get_enable())
	{
		mtx.lock();
		*raw_imu_data = _imu_raw_data;
		*imu_data = this->imu_data;
		mtx.unlock();

		return true;
	}

	return false;
}