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

#include "VRCalibration.h"
#include "TrackObjectManager.h"
#include "jsonobject.h"
#include <exception> 

#ifdef WIN32  
#include <io.h>                      //C (Windows)    access  
#else  
#include <unistd.h>                  //C (Linux)      access     
#endif  

using namespace Hypereal;
using namespace HYPEREAL;
using namespace std;

#define VR_CFG_FILENANME "\\hypereal_vrdrv_cfg.json"

namespace Hypereal{

	class VRConfig : public JsonObject {
	public:
		std::string mode;                     /// "hmd", "room"
		JsonObjectList<double> world_transparam_quat;   /// quat / trans
		JsonObjectList<double> world_transparam_trans;   /// quat / trans
		JsonObjectList<double> play_area;

		std::string last_access_drv;

		std::string debug_mode;
		float seat_height;
		float height;

		bool  init_cali_done;
		JsonObjectList<double> LHTranslation;
		JsonObjectList<double> LHRotation;

		bool haptic_en;
		bool wall_en;
		JSON_MEMBERS(mode, last_access_drv, world_transparam_quat, world_transparam_trans, \
			play_area, debug_mode, seat_height, height, haptic_en, wall_en, init_cali_done, LHTranslation, LHRotation)
	};
}

bool VRCalibration::reset()
{
	bsetalgo = false;

	return true;
}

VRCalibration::VRCalibration(VRTrackedDeviceMgr* pMgr)
{
	this->pMgr = pMgr;
	bsetalgo = false;
	json_cfg = new VRConfig;

	char *path;
	size_t len = 0;;

	strcpy_s(current_cfg_filename, "C:\\ProgramData\\HYPEREAL\\");
	CreateDirectoryA(current_cfg_filename, 0);
	strcat_s(current_cfg_filename, VR_CFG_FILENANME);
};

VRCalibration::~VRCalibration()
{
	delete json_cfg;
}

bool VRCalibration::set_calibration_cfg_to_algorithm(cfg* pcfg)
{
	// set to algorithm
	bool rc;

	if (!pMgr->is_initCalibration_done(rc, icd))
		return false;

	if (pMgr && pcfg->mode == "1" && rc)
	{
		bsetalgo = true;
		TrackObjectManager *object = TrackObjectManager::GetInstance();
		double* quat;
		double* translation;
		quat = &pcfg->world_trans_param.data[0];
		translation = &pcfg->world_trans_param.data[4];
		
		return true;
	}

	return false;
}
bool VRCalibration::set_calibration_cfg_to_file(cfg* pcfg)
{
	json_cfg->last_access_drv = pcfg->copy_right;
	json_cfg->mode = pcfg->mode;
	static bool setonce = false;

	//reset all
	{
		json_cfg->world_transparam_quat.clear();
		json_cfg->world_transparam_trans.clear();
		json_cfg->play_area.clear();
	}

	json_cfg->world_transparam_quat.append(pcfg->world_trans_param.var.i);
	json_cfg->world_transparam_quat.append(pcfg->world_trans_param.var.j);
	json_cfg->world_transparam_quat.append(pcfg->world_trans_param.var.k);
	json_cfg->world_transparam_quat.append(pcfg->world_trans_param.var.o);

	json_cfg->world_transparam_trans.append(pcfg->world_trans_param.var.x);
	json_cfg->world_transparam_trans.append(pcfg->world_trans_param.var.y);
	json_cfg->world_transparam_trans.append(pcfg->world_trans_param.var.z);

	for (int i = 0; i < 8; i++)
		json_cfg->play_area.append(pcfg->play_area[i]);

	json_cfg->debug_mode = pcfg->debug_mode;
	json_cfg->seat_height = pcfg->seat_height;
	json_cfg->height = pcfg->height;

	json_cfg->wall_en = pcfg->wall_en;

	bool ret = json_cfg->Save(current_cfg_filename);
	debug_print("----------------save cfg to %s--------------- %d\n", current_cfg_filename, ret);

	if (bsetalgo)
		save_init_calibration(&icd);

	return ret;
}

bool VRCalibration::load_calibration_cfg(cfg* pcfg)
{
	memset(pcfg, 0, sizeof(cfg));
	if (is_cfg_file_exist())
	{
		bool ret = json_cfg->Load(get_cfg_filename());
		debug_print("----------------load cfg form %s--------------- %d\n", get_cfg_filename(), ret);
		if (!ret)
		{
			debug_print("load error\n");
			return false;
		}

		pcfg->mode = json_cfg->mode;
		if (json_cfg->play_area.size() != 8) return false;
		double* play_area = json_cfg->play_area.AsArray();
		for (int i = 0; i < 8; i++)
			pcfg->play_area[i] = play_area[i];

		if (json_cfg->world_transparam_quat.size() != 4 && pcfg->mode == "1")
			return false;

		double* world_transparam_quat = json_cfg->world_transparam_quat.AsArray();
		pcfg->world_trans_param.var.i = world_transparam_quat[0];
		pcfg->world_trans_param.var.j = world_transparam_quat[1];
		pcfg->world_trans_param.var.k = world_transparam_quat[2];
		pcfg->world_trans_param.var.o = world_transparam_quat[3];

		if (json_cfg->world_transparam_trans.size() != 3 && pcfg->mode == "1")
			return false;

		double* world_transparam_trans = json_cfg->world_transparam_trans.AsArray();
		pcfg->world_trans_param.var.x = world_transparam_trans[0];
		pcfg->world_trans_param.var.y = world_transparam_trans[1];
		pcfg->world_trans_param.var.z = world_transparam_trans[2];

		pcfg->debug_mode = json_cfg->debug_mode;
		pcfg->seat_height = json_cfg->seat_height;
		pcfg->height = json_cfg->height;

		pcfg->wall_en = json_cfg->wall_en;

		debug_print("leave %s \n", __FUNCTION__);
	}
	else
		return false;

	return true;
}

void VRCalibration::CalculateWorldParams(double * inPointAtVec, double * inGravityVec, double * inCtlPosFloat, double * inCtlPosFloor, double * inHmdOrientFloor, double * outTranslation, double * outQuat)
{
	TrackObjectManager *object = TrackObjectManager::GetInstance();
}


char* VRCalibration::get_cfg_filename()
{
	return current_cfg_filename;
}

bool  VRCalibration::is_cfg_file_exist()
{
	if (0 == _access(current_cfg_filename, 0))
	{
		debug_print("cfg file exist\n");
		return true;
	}
	else
	{
		debug_print("cfg file not exist.\n");
		return false;
	}
}


bool VRCalibration::load_init_calibration(InitCalibrationData* init_cali_data)
{
	if (json_cfg->Load(get_cfg_filename()))
	{
		if (!json_cfg->init_cali_done)
			return false;

		double* temp = json_cfg->LHTranslation.AsArray();

		memcpy(init_cali_data->LHTranslation, temp, sizeof(double)*json_cfg->LHTranslation.size());

		temp = json_cfg->LHRotation.AsArray();
		memcpy(init_cali_data->LHRotation, temp, sizeof(double)*json_cfg->LHRotation.size());
	}
	else
	{
		debug_print("can't load init_calibration file");
		return false;
	}

	return true;
}

bool VRCalibration::save_init_calibration(InitCalibrationData* init_cali_data)
{
	if (!json_cfg->Load(get_cfg_filename()))
		return false;

	//reset write area
	{
		json_cfg->LHRotation.clear();
		json_cfg->LHTranslation.clear();
	}

	json_cfg->init_cali_done = true;

	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 9; j++)
			json_cfg->LHRotation.append(init_cali_data->LHRotation[i][j]);

		for (int j = 0; j < 3; j++)
			json_cfg->LHTranslation.append(init_cali_data->LHTranslation[i][j]);
	}

	json_cfg->Save(get_cfg_filename());

	return true;
}