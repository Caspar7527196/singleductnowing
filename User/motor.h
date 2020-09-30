#ifndef __MOTOR_H
#define __MOTOR_H

#include "userlib.h"
#include "rc.h"
#include "ahrs.h"

#define PWM_OUT_NUM 8
#define INI_PWM 1500
#define Min_PWM_Out  800 //800    //us
#define Max_PWM_Out  2500//2200   //us

#define RAD_TO_PWM 501.338f			//175/0.349
//#define CS_LIMIT_MIN -175
//#define CS_LIMIT_MAX 175     //175对应20度(0.349rad)
#define CS_LIMIT_MIN -175
#define CS_LIMIT_MAX 175
#define PWM_MID_1 1605
#define PWM_MID_2 1555
#define PWM_MID_3 1413
#define PWM_MID_4 1425

typedef struct
{
	TIM_HandleTypeDef* htimA;
	TIM_HandleTypeDef* htimB;
	char *name;
	s16 PwmOff[PWM_OUT_NUM];
}sMOT_CFG;

typedef struct
{
	TIM_HandleTypeDef* htimA;
	TIM_HandleTypeDef* htimB;
	char *name;
	
	bool  Update;     //更新  --
    eSTA  Sta;        //状态  --
    eERR  Err;        //错误信息  --
	bool  UnLock;     //解锁
	
	u16 PWM[PWM_OUT_NUM];
	u16 PWM_OBS[PWM_OUT_NUM];
	s16 PwmOff[PWM_OUT_NUM];
	double p_limits;//控制舵幅值（度）
}sMOT;

#define MOT_NUM 1

extern sMOT mot[MOT_NUM];

void Mot_Init_Para(sMOT *ele,sMOT_CFG *elecfg);
bool Mot_Init(sMOT *ele);
void Mot_Ctr(sMOT *ele);

#endif
