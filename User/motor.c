#include "motor.h"

/***************************************************\
功能：
  电机电调驱动，PWM驱动
说明：
  1、5ms定时周期，进行电调控制
  2、4轴使用
\***************************************************/

sMOT mot[MOT_NUM];

/******************功能函数****************/
void Mot_Init_Para(sMOT *ele,sMOT_CFG *elecfg)
{
	ele->htimA = elecfg->htimA;
	ele->htimB = elecfg->htimB;
	ele->name = elecfg->name;
	//-----------------------------------------by mch
	ele->p_limits=(CS_LIMIT_MAX/RAD_TO_PWM)*180.0f/3.1415926535897931f;//单位：度
	//--------------------------------------------
	ele->Update = true;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	
	for(u8 i=0;i<PWM_OUT_NUM;i++)
	{
		ele->PWM[i] = INI_PWM;
		ele->PwmOff[i] = elecfg->PwmOff[i];
	}
}

bool Mot_Init(sMOT *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	HAL_TIM_Base_Start(ele->htimA); //PWM
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_1,ele->PWM[0]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_2,ele->PWM[1]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_3,ele->PWM[2]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_4,ele->PWM[3]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_4,ele->PWM[4]);
//	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_3,ele->PWM[5]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_2,ele->PWM[6]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_1,ele->PWM[7]);
	HAL_TIM_PWM_Start(ele->htimA,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(ele->htimA,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(ele->htimA,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(ele->htimA,TIM_CHANNEL_4);
	HAL_TIM_Base_Start(ele->htimB); //PWM
	HAL_TIM_PWM_Start(ele->htimB,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(ele->htimB,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(ele->htimB,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(ele->htimB,TIM_CHANNEL_4);
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

void Mot_Ctr(sMOT *ele)  
{
	if(rc->Pack < 10 || rc->Mode == RC_MOT_LOCK)ele->UnLock = false;
	if(rc->Mode == RC_MOT_UNLOCK)ele->UnLock = true;
	if(ele->UnLock == false)    //电机锁定
	{
		for(u8 i=0;i<PWM_OUT_NUM;i++)
			ele->PWM[i]=INI_PWM;
	}
	rc->Mode = RC_NONE;
	ele->PWM[0] = iConstrain(CtrlIO->mt_output[0],1090,1910);
//	ele->PWM[0] = 1090;
	
	//ele->PWM[0]=CtrlIO->input[3];
	
//		if(rc->Key[3] == 0)
//	{
		ele->PWM[1] = iConstrain(PWM_MID_1+CtrlIO->cs_output[0],PWM_MID_1+CS_LIMIT_MIN,PWM_MID_1+CS_LIMIT_MAX);
		ele->PWM[2] = iConstrain(PWM_MID_2+CtrlIO->cs_output[1],PWM_MID_2+CS_LIMIT_MIN,PWM_MID_2+CS_LIMIT_MAX);
  	ele->PWM[3] = iConstrain(PWM_MID_3+CtrlIO->cs_output[2],PWM_MID_3+CS_LIMIT_MIN,PWM_MID_3+CS_LIMIT_MAX);
	  ele->PWM[4] = iConstrain(PWM_MID_4+CtrlIO->cs_output[3],PWM_MID_4+CS_LIMIT_MIN,PWM_MID_4+CS_LIMIT_MAX);
//		ele->PWM[4]=CtrlIO->pwm_temp;
//	}
//	else 
//	{
//		ele->PWM[1] = iConstrain(PWM_MID_1+CtrlIO->cs_output[0]-100,PWM_MID_1+CS_LIMIT_MIN,PWM_MID_1+CS_LIMIT_MAX);
//		ele->PWM[3] = iConstrain(PWM_MID_2+CtrlIO->cs_output[2]+100,PWM_MID_2+CS_LIMIT_MIN,PWM_MID_2+CS_LIMIT_MAX);
//		ele->PWM[2] = iConstrain(PWM_MID_2+CtrlIO->cs_output[1]+100,PWM_MID_2+CS_LIMIT_MIN,PWM_MID_2+CS_LIMIT_MAX);
//	  ele->PWM[4] = iConstrain(PWM_MID_4+CtrlIO->cs_output[3]+100,PWM_MID_4+CS_LIMIT_MIN,PWM_MID_4+CS_LIMIT_MAX);
//	}



	//ele->PWM[5]=CtrlIO->pwm_temp;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	ele->PWM[6] = iConstrain(1528+CtrlIO->cs_output[5],1378,1678);
	ele->PWM[7] = iConstrain(1552+CtrlIO->cs_output[6],1402,1702);
	for(u8 i=0;i<PWM_OUT_NUM;i++)  //限幅
		ele->PWM_OBS[i]=ele->PWM[i]=iConstrain(ele->PWM[i]-ele->PwmOff[i],Min_PWM_Out,Max_PWM_Out);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_1,ele->PWM[0]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_2,ele->PWM[1]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_3,ele->PWM[2]);
	__HAL_TIM_SET_COMPARE(ele->htimA,TIM_CHANNEL_4,ele->PWM[3]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_4,ele->PWM[4]);
//	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_3,ele->PWM[5]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_2,ele->PWM[6]);
	__HAL_TIM_SET_COMPARE(ele->htimB,TIM_CHANNEL_1,ele->PWM[7]);
}
