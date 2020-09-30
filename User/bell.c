#include "bell.h"

/***************************************************\
���ܣ�
  ������������PWM����
˵����
  1��1ms��ʱPWM����
  2����������
\***************************************************/

//do-si ���и���,1/Ƶ��=���ڣ�us��
const u16 TONE_TAB[21]={3816,3400,3030,2864,2550,2272,2024,
	1912,1704,1517,1432,1276,1136,1012,
	956, 850, 758, 716, 638, 568, 508};  

sBELL bell[BELL_NUM];	

/******************��������****************/
//void DMA2_Stream0_IRQHandler(void)
//{
//	sBELL *ele = bell;
//	DMA_CLEAR_FLAG_ALL(ele->hadc->DMA_Handle);
//}				   

/******************���ܺ���****************/
void Bell_Init_Para(sBELL *ele,sBELL_CFG *elecfg)
{
	ele->hadc = elecfg->hadc;
	ele->htim = elecfg->htim;
	ele->name = elecfg->name;
	ele->chn  = elecfg->chn;
	ele->Filt.CNT = 0;
	ele->Filt.CCR = elecfg->FiltNum;
	ele->Err = ERR_NONE;
	
	ele->Update=false;
	ele->HoldOn=false;
	ele->TimeCnt=0;
	ele->ToneIndex=0;
	ele->ToneNum=0;
	ele->VolNorm=11.1f;
}	
				   
void Bell_Init(sBELL *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	HAL_TIM_Base_Start(ele->htim);    //��ʱ��
	HAL_TIM_PWM_Start(ele->htim,ele->chn);
	
	HAL_ADC_Start_DMA(ele->hadc,(u32 *)ele->AdcRaw,1);
	HAL_Delay(10);
	Bell_Calc(ele);
	if(ele->AdcFil[0]>13.0f)
	{
		ele->VolNorm=14.8f;
		Dprintf("--4s Lipo!\r\n");
	}
	else
		Dprintf("--3s Lipo!\r\n");
	Dprintf("%s Init.\r\n",ele->name);
}

//������ʾ����һ���ſ��ٴβ��ţ�һ���ڻᱻ���ԡ�
void Bell_Loop_1ms(sBELL *ele)
{
	if(ele->Update==false)return;  //������
	if(ele->HoldOn==true)
	{
		if(--ele->TimeCnt==0)
		{
			ele->Update=false;
			ele->HoldOn=false;
		}
		return;
	}
	if(ele->TimeCnt==ele->IDLE_ARRAY[ele->ToneIndex])
		__HAL_TIM_SET_COMPARE(ele->htim,ele->chn,0);
	if(--ele->TimeCnt==0)
	{
		if(ele->ToneIndex<ele->ToneNum)
		{
			ele->TimeCnt=ele->TIME_ARRAY[ele->ToneIndex];
			ele->htim->Instance->CCR4=(ele->TONE_ARRAY[ele->ToneIndex]>>1);
			ele->htim->Instance->ARR=ele->TONE_ARRAY[ele->ToneIndex];
			ele->htim->Instance->CNT=0;   //�ָ���������ȻCNT>ARRʱ����������
			ele->ToneIndex++;
		}
		else 
		{
			ele->HoldOn=true;
			ele->TimeCnt=1000;
			__HAL_TIM_SET_COMPARE(ele->htim,ele->chn,0);
			return;
		}
	}
}

#define VOL_RES1 15.0f  //��ѹ����1
#define VOL_RES2 3.0f   //��ѹ����2	
#define VOL_NORM 11.1f
void Bell_Calc(sBELL *ele)
{
	ele->AdcRel[0] = ele->AdcRaw[0]/4096.0f*3.3f*(VOL_RES1+VOL_RES2)/VOL_RES2;
	ele->AdcRel[1] = (ele->AdcRaw[1]/1241.21f-0.76f)/0.0025f+25.f;
	//�˲�����
    SlideFilt(ele->AdcFil,ele->AdcRel,2,&ele->Filt,1);
	if(ele->AdcFil[0]<ele->VolNorm)ele->Err = ERR_SOFT;
	else ele->Err = ERR_NONE;
}

//tone:����������us�� time���������ܲ���ʱ��m��ms�� idle������β��ʱ�� num����������
void Bell_Sound(sBELL *ele,u16 *tone,u16 *time,u16 *idle,u8 num)
{
	if(num>TONE_NUM)num=TONE_NUM;
	ele->ToneIndex=0;
	ele->ToneNum=num;
	ele->TimeCnt=1;
	for(int i=0;i<num;i++)
	{
		ele->TONE_ARRAY[i]=tone[i];
		ele->TIME_ARRAY[i]=time[i];
		ele->IDLE_ARRAY[i]=idle[i];
	}
	ele->Update=true;
}

void Bell_Start(sBELL *ele)
{
	if(ele->Update==true)return;
	u16 time[6]={200,200,200,200,200,200};
	u16 idle[6]={100,100,100,100,100,100};
	u16 tone[6]={3800,3000,2200,2200,3000,3800};
	Bell_Sound(ele,tone,time,idle,6);
}

void Bell_Error(sBELL *ele)
{
	if(ele->Update==true)return;
	u16 time[2]={500,500};
	u16 idle[2]={200,200};
	u16 tone[2]={3500,2000};
	Bell_Sound(ele,tone,time,idle,2);
}

void Bell_Cali(sBELL *ele)
{
	if(ele->Update==true)return;
	u16 time[3]={300,300,300};
	u16 idle[3]={100,100,100};
	u16 tone[3]={3000,3000,3000};
	Bell_Sound(ele,tone,time,idle,3);
}

