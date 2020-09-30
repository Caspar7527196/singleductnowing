#include "exit.h"

/***************************************************\
���ܣ�
  �ⲿ�ж��豸��ȡ���缤����ģ��
˵����
  1���޸Ķ˿�,Ҫע�͵�HAL��ISR
  2��ÿ�ζ�ȡ��������Ҫ��Update��λ
  3��������ģ����Ҫ #defne USE_ULTRASONIC
  4��ע�͵���ʼ������
\***************************************************/

sEXIT exitx[EXIT_NUM];

/******************��������****************/
void EXTI9_5_IRQHandler(void)
{
	sEXIT *ele = exitx;
	if(__HAL_GPIO_EXTI_GET_IT(PWM_IN1_Pin) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(PWM_IN1_Pin);
		if(HAL_GPIO_ReadPin(PWM_IN1_GPIO_Port,PWM_IN1_Pin)==GPIO_PIN_RESET)
		{
			Tim_Calc(&ele->TimP[0]);
			ele->DatRaw[0] = ele->TimP[0].OUT;//ת��
			ele->RxFlag[0] = true;
		}
	}
}

void EXTI15_10_IRQHandler(void)
{
	sEXIT *ele = exitx;
	if(__HAL_GPIO_EXTI_GET_IT(PWM_IN2_Pin) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(PWM_IN2_Pin);
		Tim_Calc(&ele->TimP[1]);
		if(HAL_GPIO_ReadPin(PWM_IN2_GPIO_Port,PWM_IN2_Pin)==GPIO_PIN_RESET)
		{
			ele->DatRaw[1] = ele->TimP[1].OUT;//
			ele->RxFlag[1] = true;
		}
	}
	if(__HAL_GPIO_EXTI_GET_IT(PWM_IN3_Pin) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(PWM_IN3_Pin);
		Tim_Calc(&ele->TimP[2]);
		if(HAL_GPIO_ReadPin(PWM_IN3_GPIO_Port,PWM_IN3_Pin)==GPIO_PIN_RESET)
		{
			static s16 prePPM = 0;
			if(ele->TimP[2].OUT>100 && ele->TimP[2].OUT<10000)
			{
				if(abs(ele->TimP[2].OUT-prePPM)<1000)
				{
					ele->DatRaw[2] = ele->TimP[2].OUT;//����
					prePPM = ele->DatRaw[2];
					ele->RxFlag[2] = true;
				}
			}
		}
	}
}

/******************���ܺ���****************/
void Exit_Init_Para(sEXIT *ele,sEXIT_CFG *elecfg)
{
	for(u8 i=0;i<PWM_IN_NUM;i++)
	{
		ele->Update[i] = false;
		ele->RxFlag[i] = false;
		ele->Sta[i] = STA_INI;
		ele->Err[i] = ERR_NONE;
		ele->Chk[i].CNT = 0;
		ele->Chk[i].CCR = elecfg->CheckNum[i];
		ele->Filt[i].CNT = 0;
		ele->Filt[i].CCR = elecfg->FiltNum[i];
	}
	ele->name = elecfg->name;
	for(u8 i=0;i<PWM_IN_NUM;i++)
	{
		for(u8 j=0;j<EXIT_SLOPE_NUM;j++)
			ele->DatTmp[i][j]=0;
	}
}

bool Exit_Init(sEXIT *ele)
{
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	Dprintf("\r\n%s Init...\r\n",ele->name);
	if(ele->Sta[0] != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	


	HAL_Delay(10);  //�ȵ��ź�����

	for(u8 i=0;i<PWM_IN_NUM;i++)
	{
		ele->Sta[i] = STA_RUN;
	}
	if(Exit_Calc(ele)==false)
	{
		Dprintf("--Rcv [NO]\r\n");
		return false;
	}
	Dprintf("--Rcv [OK]\r\n");
	
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Exit_Calc(sEXIT *ele)
{

	Tim_Calc(&ele->Tim);   //��ʱ
	//
	if(ele->RxFlag[0] == true)
	{
		ele->RxFlag[0] = false;
		ele->DatRel[0] = 1000000.0f/(float)ele->DatRaw[0];    //��λ��rps
		ele->Update[0] = true;
	}
	else
	{
		if(ele->Chk[0].CNT<ele->Chk[0].CCR)ele->Chk[0].CNT++;
		else 
		{
			ele->Chk[0].CNT = 0;
			ele->Err[0] = ERR_UPDT;
			ele->DatRel[0] = 0.0f;
		}
	}
	SlideFilt(&ele->DatFil[0],&ele->DatRel[0],1,&ele->Filt[0],1);//ת��
	
	//
	if(ele->RxFlag[1] == true)
	{
		ele->RxFlag[1] = false;
		ele->DatRel[1] = ele->DatRaw[1] * 0.00017f - 0.17f;
		if(ele->DatRel[1]<3.0f && ele->DatRel[1]>0.01f)
		{
			SlideFilt(&ele->DatFil[1],&ele->DatRel[1],1,&ele->Filt[1],1);
			ele->Update[1] = true;
		}
	}
	else
	{
		if(ele->Chk[1].CNT<ele->Chk[1].CCR)ele->Chk[1].CNT++;
		else 
		{
			ele->Chk[1].CNT = 0;
			ele->Err[1] = ERR_UPDT;
		}
	}
	//
	if(ele->RxFlag[2] == true)
	{
		Tim_Calc(&ele->Tim);
		//EKF��Ҫ�ĸ���ʱ��
		ele->height_update_time = ele->Tim.CNT;
		//EKF��Ҫ�ĸ���ʱ��
		ele->RxFlag[2] = false;
		ele->DatRel[2] = ele->DatRaw[2] * 0.001f;
		SlideFilt(&ele->DatFil[2],&ele->DatRel[2],1,&ele->Filt[2],1);//�߶�DatFil[2]
		for(u8 i=1;i<EXIT_SLOPE_NUM;i++)
			ele->DatTmp[2][i-1]=ele->DatTmp[2][i];
		ele->DatTmp[2][EXIT_SLOPE_NUM-1]=ele->DatFil[2];
		float ab[2];
		LineFit(ele->DatTmp[2],EXIT_SLOPE_NUM,ab);  //��С���˷��������
		ele->DatSlope[2] = ab[0]*100.0f;            //0.01s
		SlideFilt(&ele->DatSlopeFil[2],&ele->DatSlope[2],1,&ele->Filt[2],2);//�߶��ٶ�DatSlopeFil[2]
		ele->Update[2] = true;
	}
	else
	{
		if(ele->Chk[2].CNT<ele->Chk[2].CCR)ele->Chk[2].CNT++;
		else 
		{
			ele->Chk[2].CNT = 0;
			ele->Err[2] = ERR_UPDT;
		}
	}
	if(ele->Update[0]||ele->Update[1]||ele->Update[2])return true;
	return false;
}

