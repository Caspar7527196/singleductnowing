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
	bool HoldOn;               //�ձ��꾯1s�ڲ��ٱ���
	u16 TONE_ARRAY[TONE_NUM];  //����  ����us��1/Ƶ�ʣ�
	u16 TIME_ARRAY[TONE_NUM];  //����ʱ��  ms
	u16 IDLE_ARRAY[TONE_NUM];  //����ʱ��  ms
	u16 TimeCnt;
	u8  ToneIndex;
	u8  ToneNum;
	
	sCNT  Filt;       //��ֵ�˲���  --
	eERR  Err;        //������Ϣ  --
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
