/**
  ******************************************************************************
  * @file    calibrate.h
  * @author  Hypereal Team
  * @version V1.2.0
  * @date    19-September-2016
  * @brief   Header for calibrate.c.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CALIBRATE_H_
#define _CALIBRATE_H_

/* Includes ------------------------------------------------------------------*/
#include "imu_data.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported functions ------------------------------------------------------- */
void calibrate_init(void);

bool calibrate_apply(imu_data *data);

#endif /* _CALIBRATE_H_ */

/************************ (C) COPYRIGHT Hypereal *****END OF FILE****/