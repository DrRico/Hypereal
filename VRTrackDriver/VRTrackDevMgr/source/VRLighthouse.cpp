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

#include "VRTrackedDeviceMgr.h"
#include "ITrackedDevice.h"
#include "ringbuffer.h"
#include "internal.h"
#include <assert.h>

using namespace Hypereal;

VRLightHouse::VRLightHouse(SubDev type, const char* system_config_content) :ITrackedDevice(type, system_config_content)
{
	set_enable(true);
}

bool VRLightHouse::probe(SubDev_info* dev_info)
{
	if (is_connected())
	{
		DeviceConnetor *pdc = DeviceConnetor::get_instance();
		dev_info->hardware_version = version;
		dev_info->battery = 100;
		return true;
	}
	return false;
}

bool VRLightHouse::raw_data_parser(int type, char* rawdata, int len)
{
	if (type != INTERNAL_PKT_TYPE::LIGHTHOUSE)
		return false;

	assert(len = sizeof(LighthouseInfoReport));

	LighthouseInfoReport *preport = (LighthouseInfoReport*)rawdata;
	version = preport->Version.Major << 8 | preport->Version.Minor;
	AxisSync = preport->AxisSync;
	ErrorCoefficient = preport->ErrorCoefficient;
	b_set_version = true;

	return true;
}

bool VRLightHouse::get_position(TimeStamp ts, VRPosition *pos)
{
	double translation[3];
	double quat[4];

	if (!is_connected() || !get_enable())
		return false;

	int li_internal_id = (subdev_type == LightHouse0 ? 0 : 1);

	PoseResultFlag rc = object->GetLighthousePoses(li_internal_id, quat, translation);

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

bool VRLightHouse::get_sensor_data(void* pbuffer)
{
	if (is_connected() && get_enable())
	{
		LightHouse_Sensor *psensor = (LightHouse_Sensor*)pbuffer;
		psensor->axis_sync = AxisSync;
		psensor->error_coefficient = ErrorCoefficient;
		return true;
	}

	return false;
}