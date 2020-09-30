#ifndef __BELL_H
#define __BELL_H

#include "userlib.h"

#define TONE_NUM 20

typedef struct
{
	ADC_HandleTypeDef* hadc;
	TIM_HandleTypeDef* htim;
	char *name;
	u32 chn;
	s16 FiltNum;
}sBELL_CFG;

typedef struct
{
	ADC_HandleTypeDef* hadc;
	TIM_HandleTypeDef* htim;
	char *name;
	u32 chn;
	bool Update;               
	bool HoldOn;               //刚报完警1s内不再报警
	u16 TONE_ARRAY[TONE_NUM];  //音调  周期us（1/频率）
	u16 TIME_ARRAY[TONE_NUM];  //持续时间  ms
	u16 IDLE_ARRAY[TONE_NUM];  //空闲时间  ms
	u16 TimeCnt;
	u8  ToneIndex;
	u8  ToneNum;
	
	sCNT  Filt;       //均值滤波器  --
	eERR  Err;        //错误信息  --
	u16 AdcRaw[2];
	float VolNorm;
	float AdcRel[2];
	float AdcFil[2];
}sBELL;

#define BELL_NUM 1
extern sBELL bell[BELL_NUM];

void Bell_Init_Para(sBELL *ele,sBELL_CFG *elecfg);
void Bell_Init(sBELL *ele);
void Bell_Loop_1ms(sBELL *ele);
void Bell_Calc(sBELL *ele);
void Bell_Sound(sBELL *ele,u16 *tone,u16 *time,u16 *idle,u8 num);
void Bell_Start(sBELL *ele);
void Bell_Error(sBELL *ele);
void Bell_Cali(sBELL *ele);

#endif
