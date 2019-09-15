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

// VS2013.cpp : Defines the entry point for the console application.

#include <windows.h>
#include <stdio.h>
#include <conio.h>

#ifdef WIN32  
#include <io.h>                      //C (Windows)    access  
#else  
#include <unistd.h>                  //C (Linux)      access     
#endif

#include "VRTrackedDeviceMgr.h"
#include "VRCalibration.h"
#include "FirmwareVersion.h"
#include <thread>

using namespace Hypereal;
using namespace std;

typedef struct{
	int a;
	int b;
}Element;
char* VREvent_type_to_String(VREvent_type evt)
{
	switch (evt)
	{
	case VREvent_type::EVENT_NONE: return "event_none";
	default:
		break;
	}
	return "unknow";
}

static const char* SubDev_toString(SubDev sd)
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

void dump_event(VREvent ev)
{
	if (ev.ev_type == VREvent_type::EVENT_FIRMWARE_NOT_MATCH)
	{
		printf("EV: %s firmware not match\n", SubDev_toString(ev.subdev));
	}

}

char* VRPosition_toString(VRPosition *pos)
{
	static char buff[2048];
	char temp[1024];
	sprintf_s(buff, "{ quat: %d, centre: %d } ", pos->quaternion_valid, pos->centerlocation_valid);
	if (pos->quaternion_valid)
	{
		sprintf_s(temp, "quat: %f %f %f %f,   ", pos->P.var.i, pos->P.var.j, pos->P.var.k, pos->P.var.o);
		strcat_s(buff, temp);
	}
	if (pos->centerlocation_valid)
	{
		sprintf_s(temp, "translation %f %f %f\n", pos->P.var.x, pos->P.var.y, pos->P.var.z);
		strcat_s(buff, temp);
	}
	if (!(pos->centerlocation_valid || pos->quaternion_valid))
	{
		sprintf_s(buff, "no pos and quat");
	}
	return buff;
}

bool quit_event_thread = false;

void event_thread(VRTrackedDeviceMgr* pmgr)
{
	while (!quit_event_thread)
	{
		VREvent ev;
		if (pmgr->poll_event(&ev))
		{
			if (ev.ev_type == VREvent_type::EVENT_NONE) continue;
			dump_event(ev);
		}
		Sleep(100);
	}
}

char* cfg_to_String(cfg* pcfg)
{
	printf("enter %s:\n", __FUNCTION__);
	static char buff[1024];
	char output[2048];
	strcpy_s(output, "");

	sprintf_s(buff, "mode=%s\n", pcfg->mode.c_str());
	strcat_s(output, buff);

	sprintf_s(buff, "copy_right=%s\n", pcfg->copy_right.c_str());
	strcat_s(output, buff);

	sprintf_s(buff, "play_area (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n", pcfg->play_area[0], pcfg->play_area[1], pcfg->play_area[2], pcfg->play_area[3], pcfg->play_area[4], pcfg->play_area[5], pcfg->play_area[6], pcfg->play_area[7]);
	strcat_s(output, buff);


	sprintf_s(buff, "world_trans (%f,%f,%f,%f) (%f,%f,%f)\n", pcfg->world_trans_param.var.i, \
		pcfg->world_trans_param.var.j, \
		pcfg->world_trans_param.var.k, \
		pcfg->world_trans_param.var.o, \
		pcfg->world_trans_param.var.x, \
		pcfg->world_trans_param.var.y, \
		pcfg->world_trans_param.var.z);
	strcat_s(output, buff);
	printf("leave %s:\n", __FUNCTION__);
	return output;
}

//////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <signal.h>
#include <tchar.h>

int getch_noblock() {
	if (_kbhit())
		return _getch();
	else
		return -1;
}

typedef enum{
	MAIN_FRAME,
	HMD_IMU,
	HMD_LIGHT,
	PAD
}UIStat;

int main(int argc, char* argv[])
{
	UIStat _stat = MAIN_FRAME;

	VRTrackedDeviceMgr *pMgr = NULL;
	pMgr = new VRTrackedDeviceMgr();

	int init_rc;
	int ret = pMgr->Device_init();
	init_rc = ret;
	if (ret == HVRERROR)
	{
		printf("can't init dev\n");
		getchar();
		return -1;
	}

	pMgr->Device_set_enable(SubDev::HMD0, true);

	std::thread  t3(event_thread, pMgr);

	VRCalibration vrcali(pMgr);
	cfg _cfg;

	bool word_trans = false;

	InitCalibrationData icd;
	printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@initrc = %d\n", init_rc);
	vrcali.load_calibration_cfg(&_cfg);

	if (init_rc > 0)
	{
		Sleep(4000);

		VRCalibration *vrc = new VRCalibration(pMgr);
		pMgr->Device_set_init_calibration_enable();

		bool calirc = false;

		while (!pMgr->is_initCalibration_done(calirc, icd))
		{
			printf("calibration is not done\n");
			Sleep(1000);
		}
		printf("calibration return: %d\n", calirc);

		if (calirc)
		{
			//vrcali.save_init_calibration(&icd);
		}
	}

	printf("hit any key to continue\n");
	getchar();
	/*
	if (vrcali.load_calibration_cfg(&_cfg))
	{
	word_trans = vrcali.set_calibration_cfg_to_algorithm(&_cfg);
	}*/
	int cnt = 0;
	char ch;
	while (ch = getch_noblock())
	{
		system("cls");

		if (ch == 'q')
		{
			printf("\n\n I get q, exit ...!!!!!!!!!!!!!!!!!\n");
			break;
		}
		else if (ch == 'i')
		{
			pMgr->Device_set_init_calibration_enable();
		}
		else if (ch == 'm')
		{
			_stat = MAIN_FRAME;
		}
		else if (ch == 'h')
		{
			_stat = HMD_IMU;
		}

		if (_stat == MAIN_FRAME)
		{
			printf("Config filename %s. exist: %d,  mode= %s\n", vrcali.get_cfg_filename(), vrcali.is_cfg_file_exist(), _cfg.mode.c_str());
			bool rc1, rc2;
			rc1 = pMgr->is_initCalibration_done(rc2, icd);
			printf("[%d] device_init_rc = %d , init Calibration = %d / %d, set_to_world_trans = %d\n", cnt++, init_rc, rc1, rc2, word_trans);


			printf("\nConnect  info\n");
			printf("- lowlayer USB\n");
			map<string, int>::iterator it;
			map<string, int> lowUSB = pMgr->getUSBlowlayers();
			for (it = lowUSB.begin(); it != lowUSB.end(); ++it)
			{
				std::cout << it->first << " => " << std::hex << it->second << '\n';
			}
			printf("allow hmd firmware: ");
			for (uint16_t n : hmd_list)
			{
				printf("%04x ", n);
			}
			printf("\n");
			printf("\n- tracking layer\n");
			pMgr->dump();

			SubDev_info info;
			printf("\nVER  info\n");
			if (pMgr->Device_probe_subdev(SubDev::HMD0, &info))
			{
				printf("hmd0 VER = 0x%x, battery = %d\n", info.hardware_version, info.battery);
			}

			if (pMgr->Device_probe_subdev(SubDev::LightHouse0, &info))
			{
				printf("Lighthouse0 VER = 0x%x, battery = %d\n", info.hardware_version, info.battery);
			}

			if (pMgr->Device_probe_subdev(SubDev::LightHouse1, &info))
			{
				printf("Lighthouse1 VER = 0x%x, battery = %d\n", info.hardware_version, info.battery);
			}

			/*
			HMD_Sensor sensor;
			pMgr->HMD_get_data(&sensor);
			printf("%d %d\n", sensor.PD, sensor.proximity_switch);
			*/

			printf("\nTrack  info\n");
			VRPosition pos;

			if (pMgr->Device_get_positionv2(SubDev::HMD0, 0, &pos))
			{
				printf("hmd0 %s\n", VRPosition_toString(&pos));
			}

			if (pMgr->Device_get_positionv2(SubDev::LightHouse0, 0, &pos))
			{
				printf("li0 %s\n", VRPosition_toString(&pos));
			}

			if (pMgr->Device_get_positionv2(SubDev::LightHouse1, 0, &pos))
			{
				printf("li1 %s\n", VRPosition_toString(&pos));
			}
		}
		else if (_stat == HMD_IMU)
		{
			printf("HMD IMU\n");
		}
		printf("\t\t\t q¼üÍË³ö m: main ; h: hmd imu; j: hmd lightsensor [q to exit]!!!!!!\n");
		Sleep(1000);

	}

	quit_event_thread = true;
	t3.join();
	pMgr->Device_deinit();
	delete pMgr;

	printf("byebye");
	return 0;
}