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

#include <stdio.h>
#include <windows.h>
#include <conio.h>
#include "HVR_Hmd.h"
#include <iostream>
#include <vector>

//**************************************************************************
// Global variables
//**************************************************************************
const int print_interval = 200;		// ms

uint64_t report_count_hmd_imu = 0;
uint64_t report_count_hmd_light = 0;
uint64_t report_count_lth_info = 0;
uint64_t report_count_lth_sync = 0;

bool flag_print_hmd_imu = false;
bool flag_print_hmd_light = false;
bool flag_print_lth_info = false;
bool flag_print_lth_sync = false;

double freq_hmd_imu = 0;
double freq_hmd_light = 0;
double freq_lth_sync = 0;

IMUReport				g_HmdImuReport;
LightSensorData			g_HmdLightData[4][HmdLightSensorNumber] = { 0 };
LighthouseInfoReport	g_InfoReport[2] = { 0 };
LighthouseSyncReport	g_LthSync = { 0 };

//**************************************************************************
// Common functions
//**************************************************************************
// Control the cursor position in Win32 Console
static inline void console_cursor_gotoxy(int x, int y)
{
	CONSOLE_SCREEN_BUFFER_INFO csbiInfo;
	HANDLE   hConsoleOut;

	hConsoleOut = GetStdHandle(STD_OUTPUT_HANDLE);
	GetConsoleScreenBufferInfo(hConsoleOut, &csbiInfo);

	csbiInfo.dwCursorPosition.X = x;
	csbiInfo.dwCursorPosition.Y = y;
	SetConsoleCursorPosition(hConsoleOut, csbiInfo.dwCursorPosition);
}
// Get the PC timestamp by QPC, unit: 0.1ms
static inline uint64_t get_system_timestamp()
{
	static LARGE_INTEGER Frequency;
	LARGE_INTEGER CurTime;

	if (Frequency.QuadPart == 0)
		QueryPerformanceFrequency(&Frequency);

	QueryPerformanceCounter(&CurTime);

	return (CurTime.QuadPart * 1000 * 10 / Frequency.QuadPart);
}

static inline void wait_for_user(bool print)
{
	if (print)
		printf("\n\t\t...Press any key to conitnue...\n");
	system("pause > nul");
}

static inline void wait_for_thread_exit()
{
	Sleep(print_interval + 50);
}

//**************************************************************************
// Callback functions
//**************************************************************************
//Callback: to handle IMU report
int32_t CbImuReport(IMUReport *report)
{
	report_count_hmd_imu++;

	if (flag_print_hmd_imu)
		memcpy_s(&g_HmdImuReport, sizeof(IMUReport), report, sizeof(IMUReport));

	return 0;
}
//Callback: to handle lightsensor report of HMD
int32_t CbLightReport(LightSensorReport *report)
{
	report_count_hmd_light++;
	if (flag_print_hmd_light) {
		if (report->ScanAxis != HVR_Hmd::SCAN_AXIS_X1 &&
			report->ScanAxis != HVR_Hmd::SCAN_AXIS_Y1 &&
			report->ScanAxis != HVR_Hmd::SCAN_AXIS_X2 &&
			report->ScanAxis != HVR_Hmd::SCAN_AXIS_Y2){
			printf("HMD LightReport: wrong Scan Axis!\n");
			return -1;
		}
		memset(g_HmdLightData[report->ScanAxis - 1], 0, sizeof(LightSensorData) * HmdLightSensorNumber);

		for (int i = 0; i < report->SampleCount; i++)
			memcpy(&g_HmdLightData[report->ScanAxis - 1][report->LightSensor[i].SensorID], &report->LightSensor[i], sizeof(LightSensorData));
	}

	return 0;
}

int32_t CbLthInfoReprot(LighthouseInfoReport *report)
{
	report_count_lth_info++;

	if (flag_print_lth_info) {
		if (report->LighthouseID != 0 && report->LighthouseID != 1)
		{
			printf("Info Report: Wrong Lighthouse ID!\n");
			return -1;
		}
		memcpy(&g_InfoReport[report->LighthouseID], report, sizeof(LighthouseInfoReport));
	}

	return 0;
}

int32_t CbLthSyncReport(LighthouseSyncReport *report)
{
	report_count_lth_sync++;

	if (flag_print_lth_sync)
		memcpy_s(&g_LthSync, sizeof(LighthouseSyncReport), report, sizeof(LighthouseSyncReport));

	return 0;
}

int32_t CbHmdVer(FirmwareVersion *report)
{
	return 0;
}

HVR_Hmd hmd(CbImuReport, CbLightReport, CbLthInfoReprot, CbLthSyncReport, CbHmdVer);

//**************************************************************************
// Test Sub-modules
//**************************************************************************
//**********************************************************************************************************
DWORD WINAPI thread_calc_hmd_imu_freq(LPVOID lpParameter)
{
	uint64_t start_time = get_system_timestamp();
	uint64_t end_time;
	uint64_t start_count = report_count_hmd_imu;
	while (flag_print_hmd_imu)
	{
		end_time = get_system_timestamp();
		if (end_time - start_time >= 10000)
		{
			uint64_t end_count = report_count_hmd_imu;
			freq_hmd_imu = (double)(end_count - start_count) * 10000 / (double)(end_time - start_time);
			start_time = end_time;
			start_count = end_count;
		}
		//else
		//	Sleep(print_interval);
	}
	return 0;
}
DWORD WINAPI thread_print_hmd_imu_format(LPVOID lpParameter)
{
	while (flag_print_hmd_imu)
	{
		//system("cls");
		console_cursor_gotoxy(0, 0);
		printf(
			"Press any key to stop...\n"
			"==============================================================\n"
			"[ TimeStamp ] 0x%08x\n"
			"[Temperature] 0x%04x\n"
			"[    IPD    ] %d\n"
			"[   Switch  ] %d\n\n"
			"[    ACCE   ] %7d %7d %7d [   GYRO   ] %7d %7d %7d\n"
			"[    ACCE   ] %7d %7d %7d [   GYRO   ] %7d %7d %7d\n"
			"[    MAGN   ] %7d %7d %7d\n\n"
			"[ Frequency ] %.1lf Hz",
			g_HmdImuReport.Timestamp, g_HmdImuReport.Temperature,
			g_HmdImuReport.InterpupillaryDistance, g_HmdImuReport.ProximitySwitch,
			g_HmdImuReport.Samples[0].Acce[0], g_HmdImuReport.Samples[0].Acce[1], g_HmdImuReport.Samples[0].Acce[2],
			g_HmdImuReport.Samples[0].Gyro[0], g_HmdImuReport.Samples[0].Gyro[1], g_HmdImuReport.Samples[0].Gyro[2],
			g_HmdImuReport.Samples[1].Acce[0], g_HmdImuReport.Samples[1].Acce[1], g_HmdImuReport.Samples[1].Acce[2],
			g_HmdImuReport.Samples[1].Gyro[0], g_HmdImuReport.Samples[1].Gyro[1], g_HmdImuReport.Samples[1].Gyro[2],
			g_HmdImuReport.MagneticField[0], g_HmdImuReport.MagneticField[1], g_HmdImuReport.MagneticField[2],
			freq_hmd_imu);

		Sleep(print_interval);
	}
	return 0;
}
void test_imu_print()
{
	system("cls");
	flag_print_hmd_imu = true;
	//calculate report frequency
	HANDLE hThread1 = CreateThread(NULL, 0, thread_calc_hmd_imu_freq, NULL, 0, NULL);
	HANDLE hThread2 = CreateThread(NULL, 0, thread_print_hmd_imu_format, NULL, 0, NULL);
	wait_for_user(false);
	flag_print_hmd_imu = false;

	wait_for_thread_exit();

	CloseHandle(hThread1);
	CloseHandle(hThread2);

	freq_hmd_imu = 0;
	memset(&g_HmdImuReport, 0, sizeof(g_HmdImuReport));

	printf("\n\n\t****** Test end, press any key to exit ******\n");
	wait_for_user(false);
}
//**********************************************************************************************************
DWORD WINAPI thread_calc_hmd_light_freq(LPVOID lpParameter)
{
	uint64_t start_time = get_system_timestamp();
	uint64_t end_time;
	uint64_t start_count = report_count_hmd_light;
	while (flag_print_hmd_light)
	{
		end_time = get_system_timestamp();
		if (end_time - start_time >= 10000)
		{
			uint64_t end_count = report_count_hmd_light;
			freq_hmd_light = (double)(end_count - start_count) * 10000 / (double)(end_time - start_time);
			start_time = end_time;
			start_count = end_count;
		}
		//else
		//	Sleep(print_interval);
	}
	return 0;
}
DWORD WINAPI thread_print_hmd_light(LPVOID lpParameter)
{
	while (flag_print_hmd_light)
	{
		//system("cls");
		console_cursor_gotoxy(0, 0);
		printf("         <A-x>    <A-y>    <B-x>   <B-y>\n");
		for (int i = 0; i < HmdLightSensorNumber; i++) {
			printf("[ %2d ]", i);
			for (int j = 0; j < 4; j++)
				printf("  %7d", g_HmdLightData[j][i].MeasureResult);
			printf("\n");
		}
		printf("\n[ Frequency ] %.1lf Hz\n", freq_hmd_light);

		Sleep(print_interval);
	}

	return 0;
}

void test_light_print()
{
	system("cls");

	flag_print_hmd_light = true;
	HANDLE hThread1 = CreateThread(NULL, 0, thread_calc_hmd_light_freq, NULL, 0, NULL);
	HANDLE hThread2 = CreateThread(NULL, 0, thread_print_hmd_light, NULL, 0, NULL);
	wait_for_user(false);
	flag_print_hmd_light = false;

	wait_for_thread_exit();
	CloseHandle(hThread1);
	CloseHandle(hThread2);

	printf("\n\t****** Test end, press any key to exit ******\n");
	wait_for_user(false);
}

//**********************************************************************************************************
DWORD WINAPI thread_print_info_report(LPVOID lpParameter)
{
	while (flag_print_lth_info)
	{
		system("cls");
		printf("Press any key to stop...\n");
		printf("\n[ Report Count ] %lld\n", report_count_lth_info);
		for (int i = 0; i < 2; i++)
			printf("\nLighthouseID %d ==================\n"
			"\tAxis Sync: 0x%02x\n"
			"\tVersion  : %d.%d\n"
			"\tErrorCoe : 0x%08x\n",
			i,
			g_InfoReport[i].AxisSync,
			g_InfoReport[i].Version.Major, g_InfoReport[i].Version.Minor,
			g_InfoReport[i].ErrorCoefficient);
		Sleep(200);
	}
	return 0;
}

void test_lth_info_report()
{
	system("cls");
	flag_print_lth_info = true;
	HANDLE hThread = CreateThread(NULL, 0, thread_print_info_report, NULL, 0, NULL);
	wait_for_user(false);
	flag_print_lth_info = false;
	wait_for_thread_exit();
	CloseHandle(hThread);

	printf("\n\t****** Test end, press any key to exit ******\n");
	wait_for_user(false);
}

//**********************************************************************************************************
DWORD WINAPI thread_calc_lth_sync_freq(LPVOID lpParameter)
{
	uint64_t start_time = get_system_timestamp();
	uint64_t end_time;
	uint64_t start_count = report_count_lth_sync;
	while (flag_print_lth_sync)
	{
		end_time = get_system_timestamp();
		if (end_time - start_time >= 10000)
		{
			uint64_t end_count = report_count_lth_sync;
			freq_lth_sync = (double)(end_count - start_count) * 10000 / (double)(end_time - start_time);
			start_time = end_time;
			start_count = end_count;
		}
		//else
		//	Sleep(print_interval);
	}
	return 0;
}
DWORD WINAPI thread_print_lth_sync(LPVOID lpParameter)
{
	while (flag_print_lth_sync) {
		system("cls");
		printf("Press any key to exit...\n");
		printf("\n[ Report Count ] %lld\n", report_count_lth_sync);
		printf("[   Frequency  ] %.1lf Hz\n", freq_lth_sync);
		printf("[  Scan Number ] %d", g_LthSync.ScanNumber);
		Sleep(print_interval);
	}
	return 0;
}
void test_lth_sync_report()
{
	system("cls");

	flag_print_lth_sync = true;
	HANDLE hThread1 = CreateThread(NULL, 0, thread_calc_lth_sync_freq, NULL, 0, NULL);
	HANDLE hThread2 = CreateThread(NULL, 0, thread_print_lth_sync, NULL, 0, NULL);
	wait_for_user(false);
	flag_print_lth_sync = false;

	wait_for_thread_exit();
	CloseHandle(hThread1);
	CloseHandle(hThread2);

	printf("\n\n\t****** Test end, press any key to exit ******\n");
	wait_for_user(false);
}

//**********************************************************************************************************
void test_imu_calibration()
{
	system("cls");
	printf("Test case: IMU calibration data read & write\n");

	printf(">> Current calibration data:\n");
	CalibrationData *pCaliData = hmd.GetCalibrationData();
	if (pCaliData != NULL)
	{
		printf("GyroCorrect [Slop] %f %f %f [Intercept] %f %f %f\n",
			pCaliData->GyroCorrect.Slop[0], pCaliData->GyroCorrect.Slop[1], pCaliData->GyroCorrect.Slop[2],
			pCaliData->GyroCorrect.Intercept[0], pCaliData->GyroCorrect.Intercept[1], pCaliData->GyroCorrect.Intercept[2]);
		printf("AcceCorrect [Slop] %f %f %f [Intercept] %f %f %f\n",
			pCaliData->AcceCorrect.Slop[0], pCaliData->AcceCorrect.Slop[1], pCaliData->AcceCorrect.Slop[2],
			pCaliData->AcceCorrect.Intercept[0], pCaliData->AcceCorrect.Intercept[1], pCaliData->AcceCorrect.Intercept[2]);
	}
	else
	{
		printf(">> Calibration data read failed!\n");
		wait_for_user(TRUE);
		return;
	}

	CalibrationData offset = { 0.0f };
	printf("\nID - Test case\n"
		" 1 - Write a same value\n"
		" 2 - Write different values\n"
		" 3 - Clear calibration data\n"
		" 4 - exit test\n"
		">> Please make your choice(1, 2, 3, 4): ");
	uint8_t choice;
	std::cin >> choice;
	switch (choice)
	{
	case '1':
	{
		printf(">> Please input a new float-type value:");
		float temp;
		std::cin >> std::dec >> temp;
		for (int i = 0; i < 3; i++)
		{
			offset.AcceCorrect.Slop[i] = offset.AcceCorrect.Intercept[i] = offset.GyroCorrect.Slop[i] = offset.GyroCorrect.Intercept[i] = temp;
		}
		break;
	}
	case '2':
	{
		printf(">> Please input the Temperature drift calibration of GYRO, Slop[0]~[2], Intercept[0]~[2], 6 float-type values:\n");
		std::cin >> std::dec >> offset.GyroCorrect.Slop[0] >> offset.GyroCorrect.Slop[1] >> offset.GyroCorrect.Slop[2];
		std::cin >> std::dec >> offset.GyroCorrect.Intercept[0] >> offset.GyroCorrect.Intercept[1] >> offset.GyroCorrect.Intercept[2];
		printf(">> Please input the Temperature drift calibration of ACCE, Slop[0]~[2], Intercept[0]~[2], 6 float-type values:\n");
		std::cin >> std::dec >> offset.AcceCorrect.Slop[0] >> offset.AcceCorrect.Slop[1] >> offset.AcceCorrect.Slop[2];
		std::cin >> std::dec >> offset.AcceCorrect.Intercept[0] >> offset.AcceCorrect.Intercept[1] >> offset.AcceCorrect.Intercept[2];
		break;
	}
	case '3':
	{
		memset(&offset, 0xff, sizeof(offset));
		printf(">> All calibration data will be cleared to be 0xFF\n");
		break;
	}
	case '4':
		return;
	default:
		printf(">> Wrong choice, press any key to exit\n");
		wait_for_user(false);
		return;
	}

	printf(">> Writing data¡­¡­\n");
	int ret = hmd.SetCalibrationData(&offset);
	if (ret < 0)
	{
		printf(">> Write data failed!\n");
		wait_for_user(TRUE);
		return;
	}

	pCaliData = hmd.GetCalibrationData();
	if (pCaliData != NULL)
	{
		printf(">> Now, the new calibration data are:\n");
		printf("GyroCorrect [Slop] %f %f %f [Intercept] %f %f %f\n",
			pCaliData->GyroCorrect.Slop[0], pCaliData->GyroCorrect.Slop[1], pCaliData->GyroCorrect.Slop[2],
			pCaliData->GyroCorrect.Intercept[0], pCaliData->GyroCorrect.Intercept[1], pCaliData->GyroCorrect.Intercept[2]);
		printf("AcceCorrect [Slop] %f %f %f [Intercept] %f %f %f\n",
			pCaliData->AcceCorrect.Slop[0], pCaliData->AcceCorrect.Slop[1], pCaliData->AcceCorrect.Slop[2],
			pCaliData->AcceCorrect.Intercept[0], pCaliData->AcceCorrect.Intercept[1], pCaliData->AcceCorrect.Intercept[2]);
	}
	else
	{
		printf(">> Calibration data read failed!\n");
		wait_for_user(true);
		return;
	}

	printf("\n\t****** Test end, press any key to exit ******\n");
	wait_for_user(false);
}

//**********************************************************************************************************
void test_nv_wr()
{
	system("cls");
	printf("Test case: Read & Write None-Volitale data\n");

	printf(">> Current NV data:\n");
	NoneVolatileData *nv = hmd.GetNoneVolatileData();
	if (nv != NULL)
	{
		for (uint8_t i = 0; i < sizeof(NoneVolatileData); i++)
		{
			printf("0x%02x", nv->buf[i]);
			printf((i + 1) % 16 ? " " : "\n");
		}
	}
	else
	{
		printf(">> NV data read failed!\n");
		wait_for_user(true);
		return;
	}

	NoneVolatileData new_nv;
	printf(">> Write new NV data:\n");
	printf("\nID    - Test case\n"
		" 1    - Write a same value\n"
		" 2    - Write different values\n"
		" 3    - Exit test\n"
		"other - Clear to be 0\n"
		">> Please make your choice(1, 2, 3): ");
	uint8_t choice;
	std::cin >> choice;
	switch (choice)
	{
	case '1':
	{
		printf(">>Please input a new data(1 Byte in hexadecimal format):");
		uint32_t temp;
		std::cin >> std::hex >> temp;
		for (uint8_t i = 0; i < sizeof(NoneVolatileData); i++){
			new_nv.buf[i] = (uint8_t)temp;
		}
		break;
	}
	case '2':
	{
		for (uint8_t i = 0; i < sizeof(NoneVolatileData); i++){
			printf(">> Please input the %2d-th data(1 Byte in hexadecimal format):", i + 1);
			uint32_t temp;
			std::cin >> std::hex >> temp;
			new_nv.buf[i] = (uint8_t)temp;
		}
		break;
	}
	case '3':
	{
		return;
	}
	default:
		printf(">> All data cleared to be 0x0\n");
		for (uint8_t i = 0; i < sizeof(NoneVolatileData); i++){
			new_nv.buf[i] = 0;
		}
		break;
	}
	printf(">> Writing data...\n");
	hmd.SetNoneVolatileData(&new_nv);

	nv = hmd.GetNoneVolatileData();
	if (nv != NULL)
	{
		printf(">> The new NV data are:\n");
		for (uint8_t i = 0; i < sizeof(NoneVolatileData); i++)
		{
			printf("0x%02x", nv->buf[i]);
			printf((i + 1) % 16 ? " " : "\n");
		}
	}
	else
	{
		printf(">> NV read failed!\n");
		wait_for_user(true);
		return;
	}

	printf(">> Please close this program, and re-connect the device, then open this program to check the NV data\n");
	printf("\n\t****** Test end, press any key to exit ******\n");
	wait_for_user(false);
}

//**********************************************************************************************************
void test_hmd_version()
{
	system("cls");
	printf("Test case: Get HMD firmware version\n");
	FirmwareVersion *fmVersion = hmd.GetFirmwareVersion();
	if (fmVersion)
		printf(">> Current firmware version: v%d.%d\n", fmVersion->Major, fmVersion->Minor);
	else
	{
		printf(">> Get firmware version failed!\n");
		wait_for_user(true);
		return;
	}

	printf("\n\t****** Test end, press any key to exit ******\n");
	wait_for_user(false);
}

//**********************************************************************************************************
int main()
{
	if (hmd.Open(PROD_STR_HMD) < 0) {
		printf("\n>> Can NOT open \"HMD\", please check the USB device connection\n");
		wait_for_user(true);
		return -1;
	}

	while (true)
	{
		system("cls");
		printf("============== Tool build version: %s %s\n\n", __DATE__, __TIME__);
		printf("Current Device >>>>>> HMD (v%d.%d) <<<<<<\n\n", hmd.McuVersion.Major, hmd.McuVersion.Minor);

		printf(" ID    Testcase\n"
			" **    ***********************************\n"
			" 1  -  HMD IMU Report\n"
			" 2  -  HMD Lightsensor Report\n"
			" 3  -  LTH Information Report\n"
			" 4  -  LTH Synchronization Report\n"
			" 5  -  IMU calibration data read & write\n"
			" 6  -  NV data read & write\n"
			" 7  -  Get HMD firmware version\n"
			"\nPlease input your choice, or input \"e\" or \"E\" to exit: ");
		uint8_t choice;
		std::cin >> choice;
		if (choice == 'e' || choice == 'E')
			goto main_exit;
		switch (choice)
		{
		case '1':
			test_imu_print();
			break;
		case '2':
			test_light_print();
			break;
		case '3':
			test_lth_info_report();
			break;
		case '4':
			test_lth_sync_report();
			break;
		case '5':
			test_imu_calibration();
			break;
		case '6':
			test_nv_wr();
			break;
		case '7':
			test_hmd_version();
			break;
		default:
			printf("\n ***** Wrong choice!!! *****\n");
			wait_for_user(TRUE);
			break;
		}
	}

main_exit:
	hmd.Close();
	return 0;
}