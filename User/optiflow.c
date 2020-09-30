#include "optiflow.h"

/***************************************************\
功能：
  串口接收光流数据
说明：
  1、使用匿名科创光流模块
  2、接收采用DMA+空闲中断
  3、每次读取完数据需要将Update复位
\***************************************************/

sFLOW flow[FLOW_NUM];

/******************驱动程序****************/
//void UART8_IRQHandler(void)	//空闲中断+DMA
//{
//	HAL_GPIO_TogglePin(GPIOE,LED3_Pin);
//	sFLOW *ele = flow;
//	if((__HAL_UART_GET_FLAG(ele->huart,UART_FLAG_IDLE) == RESET)) return;
//	__HAL_UART_CLEAR_IDLEFLAG(ele->huart);
//	__HAL_DMA_DISABLE(ele->huart->hdmarx);
//	//DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
//	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmarx,DMA_FLAG_FEIF2_6|DMA_FLAG_DMEIF2_6|DMA_FLAG_TEIF2_6|DMA_FLAG_HTIF2_6|DMA_FLAG_TCIF2_6);
//	ele->RxRawIndex = FLOW_RX_LEN-__HAL_DMA_GET_COUNTER(ele->huart->hdmarx);
//	do{ //0xAA 0x22 0x00 [功能字] [字节数] [内容...] [校验和]
//		if(ele->RxRawDat[0]!=0xAA || ele->RxRawDat[1]!=0xAA || ele->RxRawDat[3]!=ele->RxRawIndex-5) break;
//		if(ele->LockRx == HAL_LOCKED) break;
//		ele->LockRx = HAL_LOCKED;
//		memcpy(ele->RxDat,ele->RxRawDat,ele->RxRawIndex);
//		ele->RxDat[ele->RxRawIndex] = 0;
//		ele->LockRx = HAL_UNLOCKED;
//		ele->RxFlag = true;   //收到完整一帧
//	}while(0);
//	__HAL_DMA_SET_COUNTER(ele->huart->hdmarx,FLOW_RX_LEN);
//	__HAL_DMA_ENABLE(ele->huart->hdmarx);
//}

/******************功能函数****************/
void Flow_Init_Para(sFLOW *ele, sFLOW_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	
	ele->huart = elecfg->huart;
	ele->name = elecfg->name;
	ele->LockRx = HAL_UNLOCKED;
	ele->RxFlag = false;
	ele->RxRawIndex = 0;
	
	ele->Chk.CNT = 0;
	ele->Chk.CCR = elecfg->CheckNum;
	if(Dir_Trans(ele->Dir, elecfg->Dir) == false)
		ele->Err = ERR_SOFT;
	ele->Time = 0;
	
	ele->dt = elecfg->dt;
	ele->XyzH[0] = 0;
	ele->XyzH[1] = 0;
}

bool Flow_Init(sFLOW *ele)
{
	//配置DMA    DMA1_Stream6
	__HAL_DMA_DISABLE(ele->huart->hdmarx);
	ele->huart->hdmarx->Instance->PAR = (u32)&ele->huart->Instance->RDR;
	ele->huart->hdmarx->Instance->NDTR = FLOW_RX_LEN;
	ele->huart->hdmarx->Instance->M0AR = (u32)ele->RxRawDat;
	//DMA_CLEAR_FLAG_ALL(ele->huart->hdmarx);
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmarx,DMA_FLAG_FEIF2_6|DMA_FLAG_DMEIF2_6|DMA_FLAG_TEIF2_6|DMA_FLAG_HTIF2_6|DMA_FLAG_TCIF2_6);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAR);
	
	__HAL_UART_CLEAR_IT(ele->huart,UART_CLEAR_FEF|UART_CLEAR_IDLEF|UART_CLEAR_NEF);
	__HAL_UART_ENABLE_IT(ele->huart,UART_IT_IDLE);
	
	Dprintf("\r\n%s Init...\r\n", ele->name);
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
		return false;
	}
	if(ele->Err == ERR_SOFT)
	{
		Dprintf("--Dir [NO]\r\n");
		return false;
	}
	ele->Sta = STA_RUN;
	HAL_Delay(250);  //等待信号来临
	if(ele->RxFlag == false)
	{
		Dprintf("--Rcv [NO]\r\n");
		return false;
	}
	Dprintf("--Rcv [OK]\r\n");
	Dprintf("%s Init.[OK]\r\n", ele->name);
	return true;
}

void IMU_Calc2(sFLOW *ele);
void ALT_Calc2(sFLOW *ele);
void OF_Calc2(sFLOW *ele);
void Frame_Trans(sFLOW *ele);

//按协议进行数据解析
bool Flow_Calc(sFLOW *ele)
{
	if(ele->Sta == STA_INI) return false;
	Tim_Calc(&ele->Tim);   //计时
	if(ele->RxFlag == false)
	{
		if(ele->Chk.CNT<ele->Chk.CCR) ele->Chk.CNT++;
		else ele->Err = ERR_UPDT;
		return false;      //未更新
	}
	ele->RxFlag = false;
	ele->Chk.CNT = 0;
	
	if(ele->LockRx == HAL_LOCKED) return false;
	ele->LockRx = HAL_LOCKED;
	do{
		u8 num = ele->RxDat[3]+5;	//该帧长度
		u8 RxSum = 0;
		for(u8 i=0;i<num-1;i++) RxSum += ele->RxDat[i];
		if(RxSum != ele->RxDat[num-1]) break;	//判断sum校验【放中断里较为合理】
//		if(ele->RxDat[0]!=0xAAU || ele->RxDat[1]!=0xAAU) break;	//判断帧头【这里没必要，因为中断里已判断】
		
		u8 FcnWord = ele->RxDat[2];
		switch(FcnWord)		//[功能字]
		{
			case 0x51U:		//[光流信息]
				switch(ele->RxDat[4])
				{
					case 0x00:	//原始光流信息
						ele->Quality	= *(ele->RxDat+5);
						ele->DX			= *(ele->RxDat+6);
						ele->DY			= *(ele->RxDat+7);
						ele->Light		= *(ele->RxDat+8);
						ele->OF_Mode    = 0;
						break;
					case 0x01:	//融合后光流信息
						ele->Quality	= *(ele->RxDat+5);
						ele->DX2		= (int16_t)(*(ele->RxDat+6)<<8)|*(ele->RxDat+7);
						ele->DY2		= (int16_t)(*(ele->RxDat+8)<<8)|*(ele->RxDat+9);
						ele->DX2Fix		= (int16_t)(*(ele->RxDat+10)<<8)|*(ele->RxDat+11);
						ele->DY2Fix		= (int16_t)(*(ele->RxDat+12)<<8)|*(ele->RxDat+13);
						ele->Light  	= *(ele->RxDat+14);
						ele->OF_Mode    = 1;
						break;
					default: break;
				}
				OF_Calc2(ele);
				//ele->OFUpdate = true;
				break;
			case 0x52U:		//[高度信息]
				switch(ele->RxDat[4])
				{
					case 0x00U:	//原始高度信息
						ele->Alt = (uint16_t)(*(ele->RxDat+5)<<8)|*(ele->RxDat+6);
						ele->ALT_Mode = 0;
						break;
					case 0x01U:	//融合后高度信息
						ele->Alt2 = (uint16_t)(*(ele->RxDat+5)<<8)|*(ele->RxDat+6);
						ele->ALT_Mode = 1;
						break;
					default: break;
				}
				ALT_Calc2(ele);
				//ele->AltUpdate = true;
				break;
			case 0x53U:		//[惯性数据]
				switch(ele->RxDat[4])
				{
					case 0x00U:	//原始数据
						ele->Gyr[0] = (int16_t)(*(ele->RxDat+5)<<8)|*(ele->RxDat+6);
						ele->Gyr[1] = (int16_t)(*(ele->RxDat+7)<<8)|*(ele->RxDat+8);
						ele->Gyr[2] = (int16_t)(*(ele->RxDat+9)<<8)|*(ele->RxDat+10);
						ele->Acc[0] = (int16_t)(*(ele->RxDat+11)<<8)|*(ele->RxDat+12);
						ele->Acc[1] = (int16_t)(*(ele->RxDat+13)<<8)|*(ele->RxDat+14);
						ele->Acc[2] = (int16_t)(*(ele->RxDat+15)<<8)|*(ele->RxDat+16);
						ele->IMU_Mode = 0;
						//Dprintf("IMU1");
						break;
					case 0x01U:	//滤波后数据
						ele->Gyr2[0] = (int16_t)(*(ele->RxDat+5)<<8)|*(ele->RxDat+6);
						ele->Gyr2[1] = (int16_t)(*(ele->RxDat+7)<<8)|*(ele->RxDat+8);
						ele->Gyr2[2] = (int16_t)(*(ele->RxDat+9)<<8)|*(ele->RxDat+10);
						ele->Acc2[0] = (int16_t)(*(ele->RxDat+11)<<8)|*(ele->RxDat+12);
						ele->Acc2[1] = (int16_t)(*(ele->RxDat+13)<<8)|*(ele->RxDat+14);
						ele->Acc2[2] = (int16_t)(*(ele->RxDat+15)<<8)|*(ele->RxDat+16);
						ele->IMU_Mode = 1;
						//Dprintf("IMU2");
						break;
					default: break;
				}
				IMU_Calc2(ele);
				//ele->ImuUpdate = true;
				break;
			case 0x54U:		//[姿态信息]
				switch(ele->RxDat[4])
				{
					case 0x00U:	//欧拉角格式
						ele->AttAng[0] = ((int16_t)(*(ele->RxDat+5)<<8)|*(ele->RxDat+6)) * 0.01f;
						ele->AttAng[1] = ((int16_t)(*(ele->RxDat+7)<<8)|*(ele->RxDat+8)) * 0.01f;
						ele->AttAng[2] = ((int16_t)(*(ele->RxDat+9)<<8)|*(ele->RxDat+10)) * 0.01f;
						ele->ATT_Mode = 0;
						break;
					case 0x01U:	//四元数格式
						ele->AttQtn[0] = ((int16_t)(*(ele->RxDat+5)<<8)|*(ele->RxDat+6)) * 0.0001f;
						ele->AttQtn[1] = ((int16_t)(*(ele->RxDat+7)<<8)|*(ele->RxDat+8)) * 0.0001f;
						ele->AttQtn[2] = ((int16_t)(*(ele->RxDat+9)<<8)|*(ele->RxDat+10)) * 0.0001f;
						ele->AttQtn[3] = ((int16_t)(*(ele->RxDat+11)<<8)|*(ele->RxDat+12)) * 0.0001f;
						ele->ATT_Mode = 1;
						break;
					default: break;
				}
				Frame_Trans(ele); //【】
				//ele->AttUpdate = true;
				break;
			default: break;
		}
		ele->Update = true;
	}while(0);
	
	Dprintf("%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f\r\n",ele->Quality,ele->Light,ele->DX2,ele->DX2Fix,ele->DY2,ele->DX2Fix,ele->Alt2,ele->AttAng[0],ele->AttAng[1],ele->AttAng[2],ele->Dist);
	ele->LockRx = HAL_UNLOCKED;
	
	Tim_Calc(&ele->Tim);   //计时
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}

//IMU数据处理
inline void IMU_Calc2(sFLOW *ele)
{
#ifdef __USE_IMU_DATA
	s16* gyr = ele->Gyr;
	s16* acc = ele->Acc;
	if(ele->IMU_Mode == 1)
	{
		gyr = ele->Gyr2;
		acc = ele->Acc2;
	}
	ele->GyrRot[0] =  ele->Dir[0]*(gyr[ele->Dir[1]]);	//方向变换
	ele->GyrRot[1] =  ele->Dir[2]*(gyr[ele->Dir[3]]);
	ele->GyrRot[2] =  ele->Dir[4]*(gyr[ele->Dir[5]]);
	ele->AccRot[0] = -ele->Dir[0]*(acc[ele->Dir[1]]);
	ele->AccRot[1] = -ele->Dir[2]*(acc[ele->Dir[3]]);
	ele->AccRot[2] = -ele->Dir[4]*(acc[ele->Dir[5]]);
	ele->GyrRel[0] = ele->GyrRot[0]*OF_IMU_GYR_FCT;		//实际值转换
	ele->GyrRel[1] = ele->GyrRot[1]*OF_IMU_GYR_FCT;
	ele->GyrRel[2] = ele->GyrRot[2]*OF_IMU_GYR_FCT;
	ele->AccRel[0] = ele->AccRot[0]*OF_IMU_ACC_FCT;
	ele->AccRel[1] = ele->AccRot[1]*OF_IMU_ACC_FCT;
	ele->AccRel[2] = ele->AccRot[2]*OF_IMU_ACC_FCT;
#endif
}

float XyzH2_last = 0.0f;
//高度数据处理
inline void ALT_Calc2(sFLOW *ele)
{
	static u16 timer2,timer2_last;
	int d2_timer;
	timer2 = TIM_COUNT.Instance->CNT;
	d2_timer = timer2 - timer2_last;
	ele->dt = (d2_timer<0?(65535+d2_timer):d2_timer)*0.000001f;
	timer2_last = timer2;
	//高度，单位m，方向向下
	if(ele->ALT_Mode == 0)
		ele->Dist = -ele->Alt*0.01f;
	else
		ele->Dist = -ele->Alt2*0.01f;
	
	ele->XyzH[2] = ele->Dist;
#ifndef __EN_COORDINATE_TRANS
	float cosPhi   = cosf(ele->AttAng[0]*D2R);
	float cosTheta = cosf(ele->AttAng[1]*D2R);
	ele->XyzH[2] = ele->Dist * cosTheta*cosPhi; //ele->XyzB[2]*Cb2n[2][2]
#endif
	//Z轴方向速度，单位m/s，方向向下
	ele->UvwH[2] = (ele->XyzH[2] - XyzH2_last)/ele->dt;
	XyzH2_last = ele->XyzH[2];
	
}

//光流数据处理（暂时仅考虑 OF_Mode=1 且 Dir="+X-Y-Z" 的情况）
inline void OF_Calc2(sFLOW *ele)
{
//	Tim_Calc(&ele->Tim2);  //计时
//	ele->dt = ele->Tim2.OUT * 0.000001f;
	static u16 timer1,timer1_last;
	int d_timer;
	timer1 = TIM_COUNT.Instance->CNT;
	d_timer = timer1 - timer1_last;
	ele->dt = (d_timer<0?(65535+d_timer):d_timer)*0.000001f;
	timer1_last = timer1;
	
	if(ele->OF_Mode == 1)
	{
		ele->UvwH[0] =  ele->DX2*0.01f;
		ele->UvwH[1] = -ele->DY2*0.01f;
		//【2】认为"DX2,DY2,DX2Fix,DY2Fix"是H系下的
		ele->XyzH[0] = ele->XyzH[0] + ele->DX2Fix*0.01f*ele->dt;
		ele->XyzH[1] = ele->XyzH[1] +(-ele->DY2Fix)*0.01f*ele->dt; //【float型会不会精度不够？】
	}
}

//位置、速度转换到NED系【没必要吧】
inline void Frame_Trans(sFLOW *ele)
{
#ifdef __EN_COORDINATE_TRANS
	if(ele->ATT_Mode == 0) //欧拉角格式
	{
		//欧拉角转四元数
		float sinR,cosR,sinP,cosP,cosY,sinY;
		arm_sin_cos_f32(ele->AttAng[0]/2.0f, &sinR, &cosR);
		arm_sin_cos_f32(ele->AttAng[1]/2.0f, &sinP, &cosP);
		arm_sin_cos_f32(ele->AttAng[2]/2.0f, &sinY, &cosY);
		ele->AttQtn[0] = cosY*cosP*cosR+sinY*sinP*sinR;
		ele->AttQtn[1] = cosY*cosP*sinR-sinY*sinP*cosR;
		ele->AttQtn[2] = cosY*sinP*cosR+sinY*cosP*sinR;
		ele->AttQtn[3] = sinY*cosP*cosR-cosY*sinP*sinR;
	}
	float Cb2n[3][3];
	float q0=ele->AttQtn[0], q1=ele->AttQtn[1], q2=ele->AttQtn[2], q3=ele->AttQtn[3];
	Cb2n[0][0] = 1 - 2*(q2*q2 + q3*q3); //
	Cb2n[0][1] =     2*(q1*q2 - q0*q3);
	Cb2n[0][2] =     2*(q1*q3 + q0*q2);
	Cb2n[1][0] =     2*(q1*q2 + q0*q3); //
	Cb2n[1][1] = 1 - 2*(q1*q1 + q3*q3);
	Cb2n[1][2] =     2*(q2*q3 - q0*q1);
	Cb2n[2][0] =     2*(q1*q3 - q0*q2); //
	Cb2n[2][1] =     2*(q2*q3 + q0*q1);
	Cb2n[2][2] = 1 - 2*(q1*q1 + q2*q2);
	if(ele->ATT_Mode == 1) //四元数格式
	{
		//旋转矩阵转欧拉角
		ele->AttAng[0] = atan2(Cb2n[2][1], Cb2n[2][2]) * R2D;
		ele->AttAng[1] = -asin(fConstrain(Cb2n[2][0], -1, 1)) * R2D;
		ele->AttAng[2] = atan2(Cb2n[1][0], Cb2n[0][0]) * R2D;
	}
	//NED系速度
//	ele->Uvw[0] = Cb2n[0][0]*ele->UvwB[0] + Cb2n[0][1]*ele->UvwB[1] + Cb2n[0][2]*ele->UvwB[2];
//	ele->Uvw[1] = Cb2n[1][0]*ele->UvwB[0] + Cb2n[1][1]*ele->UvwB[1] + Cb2n[1][2]*ele->UvwB[2];
//	ele->Uvw[2] = Cb2n[2][0]*ele->UvwB[0] + Cb2n[2][1]*ele->UvwB[1] + Cb2n[2][2]*ele->UvwB[2];
	//NED系位置【可改为DX2Fix转换到NED系后再积分】
//	ele->Xyz[0] = Cb2n[0][0]*ele->XyzB[0] + Cb2n[0][1]*ele->XyzB[1] + Cb2n[0][2]*ele->XyzB[2];
//	ele->Xyz[1] = Cb2n[1][0]*ele->XyzB[0] + Cb2n[1][1]*ele->XyzB[1] + Cb2n[1][2]*ele->XyzB[2];
//	ele->Xyz[2] = Cb2n[2][0]*ele->XyzB[0] + Cb2n[2][1]*ele->XyzB[1] + Cb2n[2][2]*ele->XyzB[2];
	
	float cosPhi   = cosf(ele->AttAng[0]*D2R);
	float cosTheta = cosf(ele->AttAng[1]*D2R);
//	ele->Xyz[2] = ele->XyzB[2] * cosTheta*cosPhi; //ele->XyzB[2]*Cb2n[2][2]
	ele->XyzH[2] = ele->Dist * cosTheta*cosPhi; //ele->XyzB[2]*Cb2n[2][2]
#endif
}
/***************END OF FILE************/
