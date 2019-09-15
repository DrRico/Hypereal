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

#ifndef VRCALIBRATION_H__
#define VRCALIBRATION_H__

#include "VRTrackedDeviceMgr.h"
#include <string>

using namespace Hypereal;

#ifdef VRTRACKDRVDLL_EXPORTS
#define VRTRACKDRVDLL_API __declspec(dllexport) 
#else
#define VRTRACKDRVDLL_API __declspec(dllimport) 
#endif

//! Configuration struct
typedef struct{
	std::string			mode;				//!< 0 = Without lighthouse,  1 = with lighthouse
	VRPositionResult	world_trans_param;	//!< World coordinate translation parameter
	double				play_area[8];		//!< [x0,z0] [x1,z1] [x2,z2] [x3,z3]
	std::string			copy_right;			//!< copy right string(not used)
	std::string			debug_mode;			//!< "record", "playback"
	float				seat_height;		//!< sitting height, unit:metre, for example 1.3m
	float				height;				//!< height, unit:metre, for example 1.7m
	bool				wall_en;			//!< Enable/disable room box(virtual wall) display
}cfg;

namespace Hypereal
{
	/*!
		\class VRCalibration
		\brief Used to calibrate HMD
	*/
	class VRTRACKDRVDLL_API VRCalibration
	{
	public:
		//! Constructed function
		/*!
			\param pMgr Pointer to VRTrackedDeviceMgr object
		*/
		VRCalibration(VRTrackedDeviceMgr* pMgr);

		//! Destructor function
		~VRCalibration();

		//! Set calibration config data to algorithm
		/*!
			\param pcfg Pointer to calibration config buffer
			\return
				- true on success
				- false on error
		*/
		bool set_calibration_cfg_to_algorithm(cfg* pcfg);

		//! Set calibration config data to file
		/*!
			\param pcfg Pointer to calibration config buffer
			\return
				- true on success
				- false on error
		*/
		bool set_calibration_cfg_to_file(cfg* pcfg);

		//! load calibration data form file
		/*!
			\param pcfg Pointer to store calibration config data
			\return
				- true on success
				- false on error
		*/
		bool load_calibration_cfg(cfg* pcfg);

		//! Calculate quat and translation of the room
		/*!
			\param inPointAtVec Input, pointer to FrontQuat
			\param inGravityVec Input, pointer to GravityVec
			\param inCtlPosFloat Input, pointer to CtlPosFloat
			\param inCtlPosFloor Input, pointer to CtlPosFloor
			\param inHmdOrientFloor Input, pointer to HmdOrientFloor
			\param outTranslation Output, pointer to Translation
			\param outQuat Output, pointer to Quat
		*/
		void  CalculateWorldParams(double * inPointAtVec, double * inGravityVec, double * inCtlPosFloat, double * inCtlPosFloor, double * inHmdOrientFloor, double * outTranslation, double * outQuat);

		//! Get calibration config file name
		/*!
			\return file name
		*/
		char* get_cfg_filename();

		//! Load calibration data from config file
		/*!
			\param init_cali_data Pointer to store calibration data
			\return
				- true on success
				- false on error
		*/
		bool  load_init_calibration(InitCalibrationData* init_cali_data);

		//! Check if the config file exist
		/*!
			\return
				- true if exist
				- false if not exist
		*/
		bool  is_cfg_file_exist();

		//! Set bsetalgo to false
		/*!
			\return always true
		*/
		bool  reset();

		//! Save calibration data
		bool  save_init_calibration(InitCalibrationData* init_cali_cfg);

		//! Current calibration configuration
		cfg current_cfg;

	private:
		//! Calibration config file name
		char  current_cfg_filename[1024];

		//! Pointer to VRConfig object
		class VRConfig* json_cfg;

		//! Calibration data
		InitCalibrationData icd;

		//! Flag indicate if need to set to algorithm
		bool bsetalgo;

		//! Pointer to VRTrackedDeviceMgr object
		VRTrackedDeviceMgr* pMgr;
	};
}
#endif //VRCALIBRATION_H__