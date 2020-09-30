#ifndef __EXIT_H
#define __EXIT_H

#include "userlib.h"
//#include "command.h"
#include "ahrs.h"
//#include "EKF.h"

#define PWM_IN_NUM 3 
#define EXIT_SLOPE_NUM 7

typedef struct
{
	char *name;
	s16  CheckNum[PWM_IN_NUM];
	s16  FiltNum[PWM_IN_NUM];
}sEXIT_CFG;

typedef struct
{	
	char *name;
	sTIM  Tim;        //计时器
	sCNT  Filt[PWM_IN_NUM];       //均值滤波器  --
	sCNT  Chk[PWM_IN_NUM];        //检测
	//用户访问数据
	bool  RxFlag[PWM_IN_NUM];
	bool  Update[PWM_IN_NUM];
	eSTA  Sta[PWM_IN_NUM];        //状态  --
	eERR  Err[PWM_IN_NUM];        //错误信息  --
	u32   Time;
	sTIM  TimP[PWM_IN_NUM];
	u16   DatRaw[PWM_IN_NUM];
	float DatRel[PWM_IN_NUM];
	float DatFil[PWM_IN_NUM];
	float DatSlope[PWM_IN_NUM];
	float DatSlopeFil[PWM_IN_NUM];
	float DatTmp[PWM_IN_NUM][EXIT_SLOPE_NUM];  //最小二乘法使用
	
	u32   height_update_time;
	
}sEXIT;

#define EXIT_NUM 1

extern sEXIT exitx[EXIT_NUM];

void Exit_Init_Para(sEXIT *ele,sEXIT_CFG *elecfg);
bool Exit_Init(sEXIT *ele);
bool Exit_Calc(sEXIT *ele);

#endif
