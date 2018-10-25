#ifndef __IMU_H
#define __IMU_H

#include "delay.h"
#include "LED.h"
#include "AD7689.h"
#include "HMC5983.h"

#include <math.h>
#include "Matrix.h"

#define M_PI  (float)3.1415926535
#define GRAVITY (float)13107.0  // LSB/g


extern float  pitch ,roll ,yaw;
// AHRS 解算的API
void AHRS_init(void); //初始化
void AHRS_getYawPitchRoll(float * angles); //更新姿态
#endif

//------------------End of File----------------------------
