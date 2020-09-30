#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "userlib.h"
//#include "mpu6050.h"
//#include "adis16365.h"
#include "gps.h"
#include "ms5611.h"
#include "icm20602.h"
#include "ak8975.h"
#include "lsm303d.h"
#include "l3gd20.h"
#include "ahrs.h"
//#include "command.h"
#include "motor.h"
#include "rc.h"
#include "pid.h"
#include "exit.h"
#include "transfer.h"
#include "bell.h"
//#include "sdcard.h"
//#include "LidarLite2.h"
#include "optiflow.h"
#include "PV_EKF.h"
#include "servo.h"
#include "eso.h"

typedef struct
{
//	sIMU_CFG imucfg[IMU_NUM];
//	sADIS_CFG adiscfg[ADIS_NUM];
	sGPS_CFG gpscfg[GPS_NUM];
	sMS5611_CFG ms5611cfg[MS5611_NUM];
	sICM20602_CFG icm20602[ICM20602_NUM];
	sAK8975_CFG ak8975[AK8975_NUM];
	sLSM_CFG lsm303dcfg[LSM_NUM];
	sL3G_CFG l3gd20cfg[L3G_NUM];
	sAHRS_CFG ahrscfg[AHRS_NUM];
	sMOT_CFG motcfg[MOT_NUM];
//	sCMD_CFG cmdcfg[CMD_NUM];
	sRC_CFG rccfg[RC_NUM];
	sEXIT_CFG exitcfg[EXIT_NUM];
	sTRAN_CFG trancfg[TRAN_NUM];
	sBELL_CFG bellcfg[BELL_NUM];
	sPID_CFG pidcfg[PID_NUM];
//	sSD_CFG sdcfg[SD_NUM];
//	sLIDAR_CFG lidarfg[LIDAR_NUM];
	sFLOW_CFG flowcfg[FLOW_NUM];
	
	sSERVO_CFG servocfg[SERVO_NUM];
	
	sESO_CFG esocfg[ESO_NUM];
	sPV_EKF_CFG pv_ekfcfg[PV_EKF_NUM];
}sPARA;

extern sPARA para[1];

void Para_Init(sPARA *para);

#endif

