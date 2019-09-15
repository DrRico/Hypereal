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

#include "ITrackedDevice.h"
#include <mutex>

#include "TimeManager.h"

#ifdef _WIN32
#include <Windows.h>
#endif

using namespace std;

ringbuffer<VREvent> ITrackedDevice::event_buff(50);
bool ITrackedDevice::is_calibration_done = false;

ITrackedDevice::ITrackedDevice(SubDev type, const char* system_config_content) :subdev_type(type)
{
	rx_cnt = 0;
	b_set_version = false;
	internal_id = -1;
	is_track_start = false;
	connection_timeout = 6e6;
	DeviceCB event_handler = [&](int type, char* buffer, int len)
	{
		if (PACKET_TYPE::LIGHTHOUSE_SYNC != type)
			set_last_timestamp();

		rx_cnt += len;

		if (!is_track_start)
		{
			mtx.lock();
			check_start_track_cond();
			mtx.unlock();
		}

		if (is_enable)
		{
			raw_data_parser(type, buffer, len);
		}
	};

	//register event_handler here
	devconn_handle = DeviceConnetor::get_instance();
	devconn_handle->add_dev_cb(subdev_type, event_handler);

	object = TrackObjectManager::GetInstance();

	last_pkt_time = 0xffffffff;
}

uint64_t getSystemTime() {
	static std::mutex mtx;
	std::lock_guard<std::mutex> lock(mtx);
#if defined(__linux__)
	timespec spec;
	clock_gettime(CLOCK_MONOTONIC, &spec);

	return spec.tv_sec * IMU_STAMPS_PER_SEC + spec.tv_nsec / 100;
#elif defined(_WIN32)
	return TimeManager::GetMicroSeconds();
#else
#error no impl.
#endif
}

uint64_t getSystemTime();

uint64_t ITrackedDevice::get_system_time()
{
	return getSystemTime();
}

bool ITrackedDevice::set_enable(bool en)
{
	return is_enable = en;
}

bool ITrackedDevice::is_connected()
{
	if ((get_system_time() - last_pkt_time) <= connection_timeout && b_set_version)
		return true;

	return false;
}

uint64_t ITrackedDevice::set_last_timestamp()
{
	last_pkt_time = get_system_time();

	return last_pkt_time;
}

bool ITrackedDevice::check_start_track_cond()
{
	if (!is_track_start && is_calibration_done && is_connected() && internal_id >= 0 && object)
	{
		object->Track(internal_id, false);
		is_track_start = true;
		debug_print("!!!!!!!!!!!!!!!!!!!!!!!!!start track %d\n", subdev_type);
	}

	return true;
}

bool ITrackedDevice::get_enable()
{
	return is_enable;
}