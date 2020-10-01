#include "transfer.h"

/***************************************************\
功能：
  串口传输管理
说明：
  1、使用匿名科创地面站 
  2、读取采用DMA+空闲中断，发送采用DMA中断
\***************************************************/

sTRAN tran[TRAN_NUM];

void USART3_IRQHandler(void)        //空闲中断
{ 
	sTRAN *ele = tran;
	if((__HAL_UART_GET_FLAG(ele->huart, UART_FLAG_IDLE) == RESET)) return;
	__HAL_UART_CLEAR_IDLEFLAG(ele->huart);
	__HAL_DMA_DISABLE(ele->huart->hdmarx);
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmarx,DMA_FLAG_FEIF1_5|DMA_FLAG_DMEIF1_5|DMA_FLAG_TEIF1_5|DMA_FLAG_HTIF1_5|DMA_FLAG_TCIF1_5);
	ele->RxRawIndex = TRAN_RX_LEN - __HAL_DMA_GET_COUNTER(ele->huart->hdmarx);
	do{	//0xAA 0xAF [功能字] [字节数] [内容...] [校验和]
		if(ele->RxRawDat[0]!=0xAA || ele->RxRawDat[1]!=0xAF || ele->RxRawDat[3]!=ele->RxRawIndex-5) break;
		if(ele->LockRx == HAL_LOCKED) break;
		ele->LockRx = HAL_LOCKED;
		memcpy(ele->RxDat, ele->RxRawDat, ele->RxRawIndex);
		ele->RxDat[ele->RxRawIndex] = 0;
		ele->LockRx = HAL_UNLOCKED;
		ele->RxFlag = true;   //收到完整一帧
	}while(0);
	__HAL_DMA_SET_COUNTER(ele->huart->hdmarx, TRAN_RX_LEN);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
}

void DMA1_Stream3_IRQHandler(void)  //发送DMA中断
{
	sTRAN *ele = tran;
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmatx,DMA_FLAG_FEIF3_7|DMA_FLAG_DMEIF3_7|DMA_FLAG_TEIF3_7|DMA_FLAG_HTIF3_7|DMA_FLAG_TCIF3_7);
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->TxFlag = false;
}

bool USART3_Transmit_DMA(u8 *pData, u16 Size)  //DMA发送
{
	sTRAN *ele = tran;
	if(ele->TxFlag == true) return false;
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->NDTR = Size;
	ele->huart->hdmatx->Instance->M0AR = (u32)pData;
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmatx,DMA_FLAG_FEIF3_7|DMA_FLAG_DMEIF3_7|DMA_FLAG_TEIF3_7|DMA_FLAG_HTIF3_7|DMA_FLAG_TCIF3_7);
	__HAL_DMA_ENABLE(ele->huart->hdmatx);
	ele->TxFlag = true;
	return true;
}

/******************功能函数****************/
bool Send_Check(void);
bool Send_Version(void);
bool Send_Status(void);
bool Send_Sensor(void);
bool Send_Sensor2(void);
bool Send_RCData(void);
bool Send_Power(void);
bool Send_MotoPWM(void);
bool Send_PID(u8 group,sPID *pid1,sPID *pid2,sPID *pid3);
bool Send_Debug(void);
void Write_PID(sPID *pid1,sPID *pid2,sPID *pid3);
bool Send_GPS(void);
bool Send_User(void);

void Tran_Init_Para(sTRAN *ele,sTRAN_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	
	ele->huart = elecfg->huart;
	ele->LockRx = HAL_UNLOCKED;
	ele->name = elecfg->name;
	ele->RxFlag = false;
	ele->RxRawIndex = 0;
	
	ele->Chk.CNT = 0;
	ele->Chk.CCR = elecfg->CheckNum;
	ele->Time = 0;
	//----新增---
	ele->number = 0;
	
	ele->f.send_check = false;
	ele->f.send_motopwm = false;
	ele->f.send_offset = false;
	ele->f.send_pid1 = false;
	ele->f.send_pid2 = false;
	ele->f.send_pid3 = false;
	ele->f.send_pid4 = false;
	ele->f.send_pid5 = false;
	ele->f.send_pid6 = false;
	ele->f.send_rcdata = false;
	ele->f.send_offset = false;
	ele->f.send_motopwm = false;
	ele->f.send_power = false;
	ele->f.send_check = false;
	ele->f.send_gps = false;
	ele->f.send_senser = false;
	ele->f.send_senser2 = false;
	ele->f.send_user = false;
	
	ele->TxDat[0]=0xAA;
	ele->TxDat[1]=0xAA;
}

bool Tran_Init(sTRAN *ele)
{
	//配置DMA     DMA1_Stream3
	__HAL_DMA_DISABLE(ele->huart->hdmarx);
	ele->huart->hdmarx->Instance->PAR = (u32)&ele->huart->Instance->RDR;
	ele->huart->hdmarx->Instance->NDTR = TRAN_RX_LEN;
	ele->huart->hdmarx->Instance->M0AR = (u32)ele->RxRawDat;
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmarx,DMA_FLAG_FEIF2_6|DMA_FLAG_DMEIF2_6|DMA_FLAG_TEIF2_6|DMA_FLAG_HTIF2_6|DMA_FLAG_TCIF2_6);
	__HAL_DMA_ENABLE(ele->huart->hdmarx);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAR);
	
	__HAL_DMA_DISABLE(ele->huart->hdmatx);
	ele->huart->hdmatx->Instance->PAR = (u32)&ele->huart->Instance->TDR;
	ele->huart->hdmatx->Instance->NDTR = TRAN_TX_LEN;
	ele->huart->hdmatx->Instance->M0AR = (u32)ele->TxDat;
	__HAL_DMA_CLEAR_FLAG(ele->huart->hdmatx,DMA_FLAG_FEIF3_7|DMA_FLAG_DMEIF3_7|DMA_FLAG_TEIF3_7|DMA_FLAG_HTIF3_7|DMA_FLAG_TCIF3_7);
	__HAL_DMA_ENABLE_IT(ele->huart->hdmatx,DMA_IT_TC);
	SET_BIT(ele->huart->Instance->CR3, USART_CR3_DMAT);
	
	__HAL_UART_CLEAR_IT(ele->huart,UART_CLEAR_FEF|UART_CLEAR_IDLEF|UART_CLEAR_NEF);
	__HAL_UART_ENABLE_IT(ele->huart,UART_IT_IDLE);
	
	Dprintf("\r\n%s Init...\r\n",ele->name);
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

void Tran_Loop_1ms(sTRAN *ele)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 100;//250
	static u8 senser2_cnt = 120;
	static u8 status_cnt 	= 50;
	static u8 rcdata_cnt 	= 200;
	static u8 motopwm_cnt	= 150;
	static u8 power_cnt		= 200;
	static u8 gps_cnt     = 250;	
	static u8 user_cnt    = 20;
	
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		ele->f.send_senser = true;	
	
	if((cnt % senser2_cnt) == (senser2_cnt-1))
		ele->f.send_senser2 = true;
	
	if((cnt % status_cnt) == (status_cnt-1))
		ele->f.send_status = true;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		ele->f.send_rcdata = true;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		ele->f.send_motopwm = true;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		ele->f.send_power = true;	

	if((cnt % gps_cnt) == (gps_cnt-1))
		ele->f.send_gps = true;	
	
	if((cnt % user_cnt) == (user_cnt-1))
	{
		ele->f.send_user = true;
		ele->number++;
	}
	cnt++;
	
	if(ele->f.send_check)
	{
		ele->f.send_check = !Send_Check();
	}
	else if(ele->f.send_version)
	{
		ele->f.send_version = !Send_Version();
	}
	else if(ele->f.send_status)
	{
		ele->f.send_status = !Send_Status();
	}	
	else if(ele->f.send_senser)
	{
		ele->f.send_senser = !Send_Sensor();
	}	
	else if(ele->f.send_senser2)
	{
		ele->f.send_senser2 = !Send_Sensor2();
	}	
	else if(ele->f.send_rcdata)
	{
		ele->f.send_rcdata = !Send_RCData();
	}	
	else if(ele->f.send_motopwm)
	{
		ele->f.send_motopwm = !Send_MotoPWM();
	}	
	else if(ele->f.send_power)
	{
		ele->f.send_power = !Send_Power();
	}
	else if(ele->f.send_pid1)
	{
		ele->f.send_pid1 = !Send_PID(1,&pid[0],&pid[1],&pid[2]);
	}		
	else if(ele->f.send_pid2)
	{
		ele->f.send_pid2 = !Send_PID(2,&pid[3],&pid[4],&pid[5]);
	}	
	else if(ele->f.send_pid3)
	{
		ele->f.send_pid3 = !Send_PID(3,&pid[6],&pid[7],&pid[8]);
	}
	else if(ele->f.send_pid4)
	{
		ele->f.send_pid4 = !Send_PID(4,&pid[9],&pid[10],&pid[11]);
	}		
	else if(ele->f.send_pid5)
	{
		ele->f.send_pid5 = !Send_Debug();
	}	
	else if(ele->f.send_pid6)
	{
		ele->f.send_pid6 = !Send_PID(6,&pid[15],&pid[16],&pid[17]);
	}
	else if(ele->f.send_gps)
	{
		ele->f.send_gps = !Send_GPS();
	}
	else if(ele->f.send_user)
	{
		ele->f.send_user = !Send_User();
		
	}
}

bool Tran_Calc(sTRAN *ele)
{
	Tim_Calc(&ele->Tim);   //计时
	if(ele->RxFlag == false)
	{
		if(ele->Chk.CNT<ele->Chk.CCR)ele->Chk.CNT++;
		else ele->Err = ERR_UPDT;
		return false;      //未更新
	}
	ele->RxFlag = false;
	ele->Chk.CNT = 0;
	
	if(ele->LockRx == HAL_LOCKED)return false;
	ele->LockRx = HAL_LOCKED;
	do{
		u8 num = ele->RxDat[3]+5;
		ele->TxSum = 0;
		for(u8 i=0;i<num-1;i++)ele->TxSum += ele->RxDat[i];
		if(ele->TxSum != ele->RxDat[num-1])break;		//判断sum
		if(ele->RxDat[0]!=0xAAU || ele->RxDat[1]!=0xAFU)break;		//判断帧头
		ele->TxHead = ele->RxDat[2];

		switch(ele->TxHead)  //功能字
		{
			case 0x01U:
				switch(ele->RxDat[4])  //校准
				{
					case 0x01:break;   //ACC校准
					case 0x02:ICM20602->GyrCal=true;break;    //陀螺仪校准
					case 0x04:gps->MagCal=true;break;    //磁力计校准
					case 0x05:ms5611[0].Cal=true;ms5611[1].Cal=true;break;    //气压计校准
					default:  break;  //六面校准
				}
				break;
			case 0x02U:
				switch(ele->RxDat[4])  //读取
				{
					case 0x01U://读取pid
						ele->f.send_pid1 = true;
						ele->f.send_pid2 = true;
						ele->f.send_pid3 = true;
						ele->f.send_pid4 = true;
						ele->f.send_pid5 = true;
						ele->f.send_pid6 = true;
						break;    
					case 0x02U://读取飞行模式
						break;    
					case 0x21U://读取航点数量
						break;    
					case 0xA0U://读取版本
						ele->f.send_version = true;
						break;    
					case 0xA1U://恢复默认参数
						break;    
					default: break;
				}
				break;
			case 0x03U://读取遥控数据
				ele->f.send_rcdata = true;
				break;
			case 0x0AU://读取飞行模式
				break;
			case 0x10U://写入第1组pid
				Write_PID(&pid[0],&pid[1],&pid[2]);
				break;
			case 0x11U://写入第2组pid
				Write_PID(&pid[3],&pid[4],&pid[5]);
				break;
			case 0x12U://写入第3组pid
				Write_PID(&pid[6],&pid[7],&pid[8]);//地面站7，8对应X，Y速度
				break;
			case 0x13U://写入第4组pid
				Write_PID(&pid[9],&pid[10],&pid[11]);//地面站10，11，12对应X，Y，Z位置
				break;
			case 0x14U://调试信号
				CtrlIO->gyro_gain  = 0.01f*( (vs16)(*(tran->RxDat+4 )<<8)|*(tran->RxDat+5 ) );
				CtrlIO->pwm_temp =    ( (vs16)(*(tran->RxDat+6 )<<8)|*(tran->RxDat+7 ) );
				
				CtrlLpIO->VelSet = ( (vs16)(*(tran->RxDat+8 )<<8)|*(tran->RxDat+9 ) )*0.01f;
				CtrlIO->mid_trim[0]= ( (vs16)(*(tran->RxDat+10)<<8)|*(tran->RxDat+11) )-175;
				CtrlIO->mid_trim[1]= ( (vs16)(*(tran->RxDat+12)<<8)|*(tran->RxDat+13) )-175;
				CtrlIO->mid_trim[2]= ( (vs16)(*(tran->RxDat+14)<<8)|*(tran->RxDat+15) )-175;
				CtrlLpIO->ko1       = ( (vs16)(*(tran->RxDat+16)<<8)|*(tran->RxDat+17) ) ;
				//pidY->signal       = ( (vs16)(*(tran->RxDat+18)<<8)|*(tran->RxDat+19) ) * 0.001f;
				CtrlLpIO->ko2       = ( (vs16)(*(tran->RxDat+18)<<8)|*(tran->RxDat+19) )*0.001f;
				CtrlLpIO->ko3       = ( (vs16)(*(tran->RxDat+20)<<8)|*(tran->RxDat+21) )*0.001f;
				pidRolRate->signal = fConstrain(pidRolRate->signal,0.0f,40.0f)-20.0f;
				pidPitRate->signal = fConstrain(pidPitRate->signal,0.0f,40.0f)-20.0f;
				pidYawRate->signal = fConstrain(pidYawRate->signal,0.0f,40.0f)-20.0f;
				pidRol->signal     = fConstrain(pidRol->signal,0.0f,40.0f)-20.0f;
				pidPit->signal     = fConstrain(pidPit->signal,0.0f,40.0f)-20.0f;
				pidYaw->signal     = fConstrain(pidYaw->signal,0.0f,40.0f)-20.0f;
//				pidX->signal       = fConstrain(pidX->signal,0.0f,40.0f)-20.0f;
//				pidY->signal       = fConstrain(pidY->signal,0.0f,40.0f)-20.0f;
				pidZ->signal       = fConstrain(pidZ->signal,0.0f,40.0f)-20.0f;
				tran->f.send_check = !Send_Check();
				break;
			case 0x15U://写入第6组pid
				Write_PID(&pid[15],&pid[16],&pid[17]);
				break;
			case 0x20U://读取第n个航点
				break;
			case 0x21U://写入航点
				break;
		}
		ele->Update = true;
	}while(0);
	ele->LockRx = HAL_UNLOCKED;
	
	Tim_Calc(&ele->Tim);   //计时
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}

inline bool Send_Check(void)
{
	if(tran->TxFlag == true)return false;
	tran->TxDat[2]=0xEF;
	tran->TxDat[3]=2;
	tran->TxDat[4]=tran->TxHead;
	tran->TxDat[5]=tran->TxSum;

	u8 sum = 0;
	for(u8 i=0;i<6;i++)sum += tran->TxDat[i];
	tran->TxDat[6]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 7);
}

inline bool Send_Version(void)
{
	if(tran->TxFlag == true)return false;
	u8 hardware_type = 4;
	u16 hardware_ver = 300,software_ver = 100,protocol_ver = 400,bootloader_ver = 0;
	tran->TxDat[2] =0x00;
	tran->TxDat[3] =9;
	tran->TxDat[4] =hardware_type;
	tran->TxDat[5] =BYTE1(hardware_ver);
	tran->TxDat[6] =BYTE0(hardware_ver);
	tran->TxDat[7] =BYTE1(software_ver);
	tran->TxDat[8] =BYTE0(software_ver);
	tran->TxDat[9] =BYTE1(protocol_ver);
	tran->TxDat[10]=BYTE0(protocol_ver);
	tran->TxDat[11]=BYTE1(bootloader_ver);
	tran->TxDat[12]=BYTE0(bootloader_ver);
	
	u8 sum = 0;
	for(u8 i=0;i<13;i++)sum += tran->TxDat[i];
	tran->TxDat[13]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 14);
}

inline bool Send_Status(void)
{
	if(tran->TxFlag == true)return false;
	vs16 _temp;	
	vs32 _temp2;
	tran->TxDat[2]=0x01;
	tran->TxDat[3]=12;
	_temp = (s16)(ahrs[1].Ang[0]*R2D*100.0f);
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
	_temp = (s16)(ahrs[1].Ang[1]*R2D*100.0f);
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	_temp = (s16)(ahrs[1].Ang[2]*R2D*100.0f);
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	_temp2 = (s32)(ms5611->AltFil*100.0f);
	tran->TxDat[10]=BYTE3(_temp2);
	tran->TxDat[11]=BYTE2(_temp2);
	tran->TxDat[12]=BYTE1(_temp2);
	tran->TxDat[13]=BYTE0(_temp2);
	tran->TxDat[14] = MODE_MANUAL;
	tran->TxDat[15] = mot->UnLock;
	
	u8 sum = 0;
	for(u8 i=0;i<16;i++)sum += tran->TxDat[i];
	tran->TxDat[16]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 17);
}

inline bool Send_Sensor(void)
{
	if(tran->TxFlag == true)return false;
	vs16 _temp;
	tran->TxDat[2]=0x02;
	tran->TxDat[3]=18;
	_temp = ICM20602->AccFil_3nd[0]*100;
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
	_temp = ICM20602->AccFil_3nd[1]*100;
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	_temp = ICM20602->AccFil_3nd[2]*100;	
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	_temp = ICM20602->GyrFil_2nd[0]*100;
	tran->TxDat[10]=BYTE1(_temp);
	tran->TxDat[11]=BYTE0(_temp);
	_temp = ICM20602->GyrFil_2nd[1]*100;
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
	_temp = ICM20602->GyrFil_2nd[2]*100;	
	tran->TxDat[14]=BYTE1(_temp);
	tran->TxDat[15]=BYTE0(_temp);
	_temp = gps->MagRot[0];
	tran->TxDat[16]=BYTE1(_temp);
	tran->TxDat[17]=BYTE0(_temp);
	_temp = gps->MagRot[1];	
	tran->TxDat[18]=BYTE1(_temp);
	tran->TxDat[19]=BYTE0(_temp);
	_temp = rc->Key[3];	
	tran->TxDat[20]=BYTE1(_temp);
	tran->TxDat[21]=BYTE0(_temp);
	
	
	

	u8 sum = 0;
	for(u8 i=0;i<22;i++)sum += tran->TxDat[i];
	tran->TxDat[22] = sum;
	
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 23);
}

inline bool Send_Sensor2(void)
{
	if(tran->TxFlag == true)return false;
	vs32 _temp;
	vs16 _temp2;
	tran->TxDat[2]=0x07;
	tran->TxDat[3]=6;
	_temp = exitx->DatRel[1]*100.0f;	
	tran->TxDat[4]=BYTE3(_temp);
	tran->TxDat[5]=BYTE2(_temp);
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	_temp2 = exitx->DatRel[2]*100.0f;	
	tran->TxDat[8]=BYTE1(_temp2);
	tran->TxDat[9]=BYTE0(_temp2);

	u8 sum = 0;
	for(u8 i=0;i<10;i++)sum += tran->TxDat[i];
	tran->TxDat[10] = sum;
	
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 11);
}

inline bool Send_RCData(void)
{	
	if(tran->TxFlag == true)return false;
	tran->TxDat[2]=0x03;
	tran->TxDat[3]=20;
	tran->TxDat[4]=BYTE1(rc->PPM[2]);
	tran->TxDat[5]=BYTE0(rc->PPM[2]);
	tran->TxDat[6]=BYTE1(rc->PPM[0]);
	tran->TxDat[7]=BYTE0(rc->PPM[0]);
	tran->TxDat[8]=BYTE1(rc->PPM[3]);
	tran->TxDat[9]=BYTE0(rc->PPM[3]);
	tran->TxDat[10]=BYTE1(rc->PPM[1]);
	tran->TxDat[11]=BYTE0(rc->PPM[1]);
	tran->TxDat[12]=BYTE1(rc->PPM[4]);
	tran->TxDat[13]=BYTE0(rc->PPM[4]);
	tran->TxDat[14]=BYTE1(rc->PPM[5]);
	tran->TxDat[15]=BYTE0(rc->PPM[5]);
	tran->TxDat[16]=BYTE1(rc->PPM[6]);
	tran->TxDat[17]=BYTE0(rc->PPM[6]);
	tran->TxDat[18]=BYTE1(rc->PPM[7]);
	tran->TxDat[19]=BYTE0(rc->PPM[7]);
	tran->TxDat[20]=0x03;
	tran->TxDat[21]=0xE8;
	tran->TxDat[22]=0x03;
	tran->TxDat[23]=0xE8;

	u8 sum = 0;
	for(u8 i=0;i<24;i++)sum += tran->TxDat[i];
	tran->TxDat[24]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 25);
}
inline bool Send_Power(void)
{
	if(tran->TxFlag == true)return false;
	u16 temp;

	tran->TxDat[2]=0x05;
	tran->TxDat[3]=4;
	temp = bell->AdcFil[0]*100.0f;
	tran->TxDat[4]=BYTE1(temp);
	tran->TxDat[5]=BYTE0(temp);
	temp = bell->AdcFil[1]*100.0f;
	tran->TxDat[6]=BYTE1(temp);
	tran->TxDat[7]=BYTE0(temp);

	u8 sum = 0;
	for(u8 i=0;i<8;i++)sum += tran->TxDat[i];
	tran->TxDat[8]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 9);
}

inline bool Send_MotoPWM(void)
{
	if(tran->TxFlag == true)return false;
	tran->TxDat[2]=0x06;
	tran->TxDat[3]=16;
	
	tran->TxDat[4] =BYTE1(mot->PWM_OBS[0]);
	tran->TxDat[5] =BYTE0(mot->PWM_OBS[0]);
	tran->TxDat[6] =BYTE1(mot->PWM_OBS[1]);
	tran->TxDat[7] =BYTE0(mot->PWM_OBS[1]);
	tran->TxDat[8] =BYTE1(mot->PWM_OBS[2]);
	tran->TxDat[9] =BYTE0(mot->PWM_OBS[2]);
	tran->TxDat[10]=BYTE1(mot->PWM_OBS[3]);
	tran->TxDat[11]=BYTE0(mot->PWM_OBS[3]);
	tran->TxDat[12]=BYTE1(mot->PWM_OBS[4]);
	tran->TxDat[13]=BYTE0(mot->PWM_OBS[4]);
	tran->TxDat[14]=BYTE1(mot->PWM_OBS[5]);
	tran->TxDat[15]=BYTE0(mot->PWM_OBS[5]);
	tran->TxDat[16]=BYTE1(mot->PWM_OBS[6]);
	tran->TxDat[17]=BYTE0(mot->PWM_OBS[6]);
	tran->TxDat[18]=BYTE1(mot->PWM_OBS[7]);
	tran->TxDat[19]=BYTE0(mot->PWM_OBS[7]);
	
	u8 sum = 0;
	for(u8 i=0;i<20;i++)sum += tran->TxDat[i];
	tran->TxDat[20]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 21);
}

inline bool Send_PID(u8 group,sPID *pid1,sPID *pid2,sPID *pid3)
{
	if(tran->TxFlag == true)return false;
	vs16 _temp;
	
	tran->TxDat[2]=0x10+group-1;
	tran->TxDat[3]=18;
	
	_temp = pid1->Kp * 1000;
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
	_temp = pid1->Ki  * 1000;
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	_temp = pid1->Kd  * 1000;
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	_temp = pid2->Kp  * 1000;
	tran->TxDat[10]=BYTE1(_temp);
	tran->TxDat[11]=BYTE0(_temp);
	_temp = pid2->Ki  * 1000;
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
	_temp = pid2->Kd  * 1000;
	tran->TxDat[14]=BYTE1(_temp);
	tran->TxDat[15]=BYTE0(_temp);
	_temp = pid3->Kp  * 1000;
	tran->TxDat[16]=BYTE1(_temp);
	tran->TxDat[17]=BYTE0(_temp);
	_temp = pid3->Ki  * 1000;
	tran->TxDat[18]=BYTE1(_temp);
	tran->TxDat[19]=BYTE0(_temp);
	_temp = pid3->Kd  * 1000;
	tran->TxDat[20]=BYTE1(_temp);
	tran->TxDat[21]=BYTE0(_temp);
	
	u8 sum = 0;
	for(u8 i=0;i<22;i++)sum += tran->TxDat[i];
	tran->TxDat[22]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 23);
}

inline bool Send_Debug(void)
{
	if(tran->TxFlag == true)return false;
	vs16 _temp;
	
	tran->TxDat[2]=0x14;
	tran->TxDat[3]=18;
	
	_temp = CtrlIO->gyro_gain*100;
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
	
	_temp = CtrlIO->pwm_temp;
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	//_temp = pidThrust->Ki*1000;
	_temp = CtrlLpIO->VelSet*100;
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	_temp = CtrlIO->mid_trim[0]+175;
	tran->TxDat[10]=BYTE1(_temp);
	tran->TxDat[11]=BYTE0(_temp);
	_temp = CtrlIO->mid_trim[1]+175;
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
	_temp = CtrlIO->mid_trim[2]+175;
	tran->TxDat[14]=BYTE1(_temp);
	tran->TxDat[15]=BYTE0(_temp);
	
//	_temp = pidX->signal + 20;
//	tran->TxDat[16]=BYTE1(_temp);
//	tran->TxDat[17]=BYTE0(_temp);
//	_temp = pidY->signal + 20;
//	tran->TxDat[18]=BYTE1(_temp);
//	tran->TxDat[19]=BYTE0(_temp);
	_temp = CtrlLpIO->ko1;					//临时用作姿态部分的PI参数
	tran->TxDat[16]=BYTE1(_temp);
	tran->TxDat[17]=BYTE0(_temp);
	//_temp = 0;
	_temp = CtrlLpIO->ko2*1000.0f;
	tran->TxDat[18]=BYTE1(_temp);
	tran->TxDat[19]=BYTE0(_temp);
	
	_temp = CtrlLpIO->ko3*1000.0f;	
	tran->TxDat[20]=BYTE1(_temp);
	tran->TxDat[21]=BYTE0(_temp);
	
	u8 sum = 0;
	for(u8 i=0;i<22;i++)sum += tran->TxDat[i];
	tran->TxDat[22]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 23);
}

inline void Write_PID(sPID *pid1,sPID *pid2,sPID *pid3)
{
	pid1->Kp = 0.001f*( (vs16)(*(tran->RxDat+4 )<<8)|*(tran->RxDat+5 ) );
	pid1->Ki = 0.001f*( (vs16)(*(tran->RxDat+6 )<<8)|*(tran->RxDat+7 ) );
	pid1->Kd = 0.001f*( (vs16)(*(tran->RxDat+8 )<<8)|*(tran->RxDat+9 ) );
	pid2->Kp = 0.001f*( (vs16)(*(tran->RxDat+10)<<8)|*(tran->RxDat+11) );
	pid2->Ki = 0.001f*( (vs16)(*(tran->RxDat+12)<<8)|*(tran->RxDat+13) );
	pid2->Kd = 0.001f*( (vs16)(*(tran->RxDat+14)<<8)|*(tran->RxDat+15) );
	pid3->Kp = 0.001f*( (vs16)(*(tran->RxDat+16)<<8)|*(tran->RxDat+17) );
	pid3->Ki = 0.001f*( (vs16)(*(tran->RxDat+18)<<8)|*(tran->RxDat+19) );
	pid3->Kd = 0.001f*( (vs16)(*(tran->RxDat+20)<<8)|*(tran->RxDat+21) );
	tran->f.send_check = !Send_Check();
}

inline bool Send_GPS(void)
{
	if(tran->TxFlag == true)return false;
	vs32 _temp;
	vs16 _temp2;
	
	tran->TxDat[2]=0x04;
	tran->TxDat[3]=12;

	tran->TxDat[4]=gps->status;
	tran->TxDat[5]=gps->star;
	_temp = gps->lng *10000000.0;
	tran->TxDat[6]=BYTE3(_temp);
	tran->TxDat[7]=BYTE2(_temp);
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	_temp = gps->lat *10000000.0;
	tran->TxDat[10]=BYTE3(_temp);
	tran->TxDat[11]=BYTE2(_temp);
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
//	_temp2 = gps->vtg_dir * 10.0f;
	_temp2 = (s16)rc->Pack * 100;
	tran->TxDat[14]=BYTE1(_temp2);
	tran->TxDat[15]=BYTE0(_temp2);
	
	u8 sum = 0;
	for(u8 i=0;i<16;i++)sum += tran->TxDat[i];
	tran->TxDat[16]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 17);
}

inline bool Send_User1(void)
{
	if(tran->TxFlag == true)
	{
		return false;
	}
	vs16 _temp;

	tran->TxDat[2]=0xF1;
	tran->TxDat[3]=40;//20*2
	
	_temp = gps->MagRaw[0];    //userdata1		???
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
	
	_temp = gps->MagRaw[1];		//userdata2		???
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	
	_temp = gps->MagRaw[2];	//userdata3	   ???
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	
	_temp = CtrlIO->mt_output[0];//userdata4		X????
	//_temp = adis->GyrFil_2nd[0]*R2D*100;
	tran->TxDat[10]=BYTE1(_temp);
	tran->TxDat[11]=BYTE0(_temp);
	
	_temp = ahrs[1].w * 1000;//userdata5	  X??
	//_temp = adis->GyrFil_2nd[1]*R2D*100;
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
	
	_temp = ahrs[1].x * 1000;//userdata6			Y????
	//_temp = adis->GyrFil_2nd[2]*R2D*100;
	tran->TxDat[14]=BYTE1(_temp);
	tran->TxDat[15]=BYTE0(_temp);
	
	_temp = ahrs[1].y * 1000;//userdata7	  	Y??
	tran->TxDat[16]=BYTE1(_temp);
	tran->TxDat[17]=BYTE0(_temp);

	
	_temp = ahrs[1].z * 1000;//userdata8   X??
	tran->TxDat[18]=BYTE1(_temp);
	tran->TxDat[19]=BYTE0(_temp);
	
	_temp = ICM20602->AccFil_3nd2[2] * 1000;//userdata9	   Y??
	tran->TxDat[20]=BYTE1(_temp);
	tran->TxDat[21]=BYTE0(_temp);
	
	_temp = ahrs[0].yaw_mag * R2D * 100;//userdata10	   Z??
	tran->TxDat[22]=BYTE1(_temp);
	tran->TxDat[23]=BYTE0(_temp);
	
	_temp = CtrlIO->mt_output[0];//userdata11   x????
	tran->TxDat[24]=BYTE1(_temp);
	tran->TxDat[25]=BYTE0(_temp);
	
	//_temp = CtrlLpIO->Z_command[0]*100;//userdata12    y????
	_temp = rc->RxFlag * 100; //ned????
	tran->TxDat[26]=BYTE1(_temp);
	tran->TxDat[27]=BYTE0(_temp);
	
	_temp = CtrlIO->rc_check * 100;//userdata13		z????
	tran->TxDat[28]=BYTE1(_temp);
	tran->TxDat[29]=BYTE0(_temp);
  
	_temp = gps->NED[1] * 100;//userdata14    x?? 
	tran->TxDat[30]=BYTE1(_temp);
	tran->TxDat[31]=BYTE0(_temp);
	                 
	_temp = gps->NED[2] * 100;//userdata15     y??
	tran->TxDat[32]=BYTE1(_temp);
	tran->TxDat[33]=BYTE0(_temp);	
	                       
//	_temp=	euler_EKF1[0]*100;//userdata16			z??
//	tran->TxDat[34]=BYTE1(_temp);
//	tran->TxDat[35]=BYTE0(_temp);	
//	               
//	_temp=	euler_EKF1[1]*100;//userdata17     x????
//	tran->TxDat[36]=BYTE1(_temp);
//	tran->TxDat[37]=BYTE0(_temp);	
//	            
//	_temp=	euler_EKF1[2]*100;//userdata18	    y????
//	tran->TxDat[38]=BYTE1(_temp);
//	tran->TxDat[39]=BYTE0(_temp);	
	
	_temp=gps->gpsPosAccuracy * 100;	//userdata19      z????
	//_temp = -adis->AccFil_3nd[2]*100;
	tran->TxDat[40]=BYTE1(_temp);
	tran->TxDat[41]=BYTE0(_temp);	
	
	_temp= gps->gpsSpdAccuracy * 100;//userdata20      ??   
 	//_temp=mot->PWM[0];
	tran->TxDat[42]=BYTE1(_temp);
	tran->TxDat[43]=BYTE0(_temp);	
	
	u8 sum = 0;
	for(u8 i=0;i<44;i++)sum += tran->TxDat[i];
	tran->TxDat[44]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 45);
}

inline bool Send_User(void)
{
	if(tran->TxFlag == true)
	{
		return false;
	}
	vs16 _temp;

	tran->TxDat[2]=0xF1;
	tran->TxDat[3]=40;//20*2
		
///////////////////////////////////////////////////////////////////////////	
////	_temp = mot->PWM[0];             //userdata1		
//	_temp = (gps[0].lat*1000-(long)(gps[0].lat*1000))*1000;
//	tran->TxDat[4]=BYTE1(_temp);
//	tran->TxDat[5]=BYTE0(_temp);

//	//	_temp = mot->PWM[1];  					//userdata2	
//	_temp = (gps[0].lat*1000-(long)(gps[0].lat*1000))*1000;
//	tran->TxDat[6]=BYTE1(_temp);
//	tran->TxDat[7]=BYTE0(_temp);
//	
////	_temp = mot->PWM[2];  							//userdata3	  
//	_temp = ((output_data_new.Pos[0]*R2D_D)*1000-(long)((output_data_new.Pos[0]*R2D_D)*1000))*1000;
//	tran->TxDat[8]=BYTE1(_temp);
//	tran->TxDat[9]=BYTE0(_temp);

/////////////////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////////////////	
////	_temp = mot->PWM[3]; 			//userdata4		
//	_temp = (gps[0].lng*1000-(long)(gps[0].lng*1000))*1000;
//	tran->TxDat[10]=BYTE1(_temp);
//	tran->TxDat[11]=BYTE0(_temp);
//	
////	_temp = mot->PWM[4];  						//userdata5	  
//	_temp = (gps[0].lng*1000-(long)(gps[0].lng*1000))*1000;
//	tran->TxDat[12]=BYTE1(_temp);
//	tran->TxDat[13]=BYTE0(_temp);
//	
////	_temp = mot->PWM[5];  					//userdata6			
//	_temp = ((output_data_new.Pos[1]*R2D_D)*1000-(long)((output_data_new.Pos[1]*R2D_D)*1000))*1000;	
//	tran->TxDat[14]=BYTE1(_temp);
//	tran->TxDat[15]=BYTE0(_temp);
//	
/////////////////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////////////////	
//	
////	_temp = mot->PWM[6];								//userdata7	  	
//	_temp = gps[0].alti*100;
//	tran->TxDat[16]=BYTE1(_temp);
//	tran->TxDat[17]=BYTE0(_temp);

//	
////	_temp = mot->PWM[7];           //userdata8   
//	_temp = gps[0].alti*100;
//	tran->TxDat[18]=BYTE1(_temp);
//	tran->TxDat[19]=BYTE0(_temp);
//	
////	_temp = mot->PWM[8];         //userdata9	   
//	_temp = output_data_new.Pos[2]*100;
//	tran->TxDat[20]=BYTE1(_temp);
//	tran->TxDat[21]=BYTE0(_temp);

/////////////////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////////////////	
////	_temp = mot->PWM[9];									//userdata10	   
//	_temp = gps[0].NED_spd[0]*100;
//	tran->TxDat[22]=BYTE1(_temp);
//	tran->TxDat[23]=BYTE0(_temp);
//	
//	//_temp =  eso->x_k[4]*R2D*100;							//userdata11   
//	_temp = gps[0].NED_spd[0]*100;
//	tran->TxDat[24]=BYTE1(_temp);
//	tran->TxDat[25]=BYTE0(_temp);
//	
//	//_temp =  eso->x_k[5]*R2D*100;							//userdata12   
//	_temp = output_data_new.Vel[0]*100;
//	tran->TxDat[26]=BYTE1(_temp);
//	tran->TxDat[27]=BYTE0(_temp);

/////////////////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////////////////	

////	_temp =  eso->x_k[6]*100;							   //userdata13	
//	_temp = gps[0].NED_spd[1]*100;
//	tran->TxDat[28]=BYTE1(_temp);
//	tran->TxDat[29]=BYTE0(_temp);

////	_temp =  eso->x_k[7]*100;								//userdata14   
//	_temp = gps[0].NED_spd[1]*100;
//	tran->TxDat[30]=BYTE1(_temp);
//	tran->TxDat[31]=BYTE0(_temp);
//	                 
////	_temp =  eso->x_k[8]*100;								//userdata15   
//	_temp = output_data_new.Vel[1]*100;
//	tran->TxDat[32]=BYTE1(_temp);
//	tran->TxDat[33]=BYTE0(_temp);	

/////////////////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////////////////	

//	//_temp = ahrs[1].pqr[0]*R2D*100;						//userdata16
//	_temp = gps[0].NED_spd[2]*100;
//	tran->TxDat[34]=BYTE1(_temp);
//	tran->TxDat[35]=BYTE0(_temp);	
//	               
//	//_temp = ahrs[1].pqr[1]*R2D*100;						//userdata17 
//	_temp = gps[0].NED_spd[2]*100;
//	tran->TxDat[36]=BYTE1(_temp);
//	tran->TxDat[37]=BYTE0(_temp);	
//	            
//	//_temp = ahrs[1].pqr[2]*R2D*100;						//userdata18
//	_temp = output_data_new.Vel[2]*100;
//	tran->TxDat[38]=BYTE1(_temp);
//	tran->TxDat[39]=BYTE0(_temp);	

/////////////////////////////////////////////////////////////////////////	

/////////////////////////////////////////////////////////////////////////	
////	
//	//_temp=CtrlFbck->Z[0]*100;								//userdata19 
//	_temp= gps[0].gpsPosAccuracy * 100;
////	_temp= dV_inter[2] * 100;
//	tran->TxDat[40]=BYTE1(_temp);
//	tran->TxDat[41]=BYTE0(_temp);	
//	
//	_temp=  gps[0].gpsPosAccuracy * 100;					//userdata20   
//	//_temp = CtrlFbck->Z[0]*100;                 
//	tran->TxDat[42]=BYTE1(_temp);
//	tran->TxDat[43]=BYTE0(_temp);	
//	
//	
	_temp = CtrlLpIO->X_command[1]*100;             //userdata1		滚转角
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
	
	_temp = CtrlLpIO->Y_command[1]*100;  					//userdata2		俯仰角
//	_temp = -ICM20602->AccFil[0]* 100;	
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	
	_temp = CtrlLpIO->Z_command[1]*100;  							//userdata3	   偏航角
	//_temp = -ICM20602->AccFil_3nd2[0]*100;
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	
	_temp = CtrlFbck->XH[1] * 100; 			//userdata4		X速度给定
	tran->TxDat[10]=BYTE1(_temp);
	tran->TxDat[11]=BYTE0(_temp);
	
	_temp = CtrlFbck->YH[1] * 100;  						//userdata5	  X速度
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
	
	_temp = CtrlFbck->Z[1] * 100;  					//userdata6			Y速度给定
	tran->TxDat[14]=BYTE1(_temp);
	tran->TxDat[15]=BYTE0(_temp);
	
	_temp = CtrlLpIO->Z_command[0] * 100;								//userdata7	  	Y速度
	tran->TxDat[16]=BYTE1(_temp);
	tran->TxDat[17]=BYTE0(_temp);

	
	_temp = CtrlFbck->Z[0] * 100;           //userdata8   X位置
	tran->TxDat[18]=BYTE1(_temp);
	tran->TxDat[19]=BYTE0(_temp);
	
	_temp = CtrlFbck->X[0] * 100;         //userdata9	   Y位置
	tran->TxDat[20]=BYTE1(_temp);
	tran->TxDat[21]=BYTE0(_temp);
	
	_temp = CtrlFbck->Y[0] * 100;									//userdata10	   Z位置
	tran->TxDat[22]=BYTE1(_temp);
	tran->TxDat[23]=BYTE0(_temp);
	
	//_temp =  eso->x_k[4]*R2D*100;												//userdata11   x位置测量
	_temp = CtrlFbck->Ang[0]*R2D*100;
	tran->TxDat[24]=BYTE1(_temp);
	tran->TxDat[25]=BYTE0(_temp);
	
	//_temp =  eso->x_k[5]*R2D*100;												//userdata12    y位置测量
	_temp = CtrlFbck->Ang[1]*R2D*100;
	tran->TxDat[26]=BYTE1(_temp);
	tran->TxDat[27]=BYTE0(_temp);
	
//	_temp =  eso->x_k[6]*100;													//userdata13		z位置测量
	//_temp=CtrlIO->cs_output[0]*100;
//	_temp = CtrlFbck->Ang[2]*R2D * 100;
  _temp = CtrlMRAC2->uc2[0]*R2D*100;
	tran->TxDat[28]=BYTE1(_temp);
	tran->TxDat[29]=BYTE0(_temp);

//	_temp =  eso->x_k[7]*100;												//userdata14    x速度 
	_temp = CtrlLpIO->roll_command[0] *R2D* 100;
	tran->TxDat[30]=BYTE1(_temp);
	tran->TxDat[31]=BYTE0(_temp);
	                 
//	_temp =  eso->x_k[8]*100;												//userdata15     y速度
//	_temp = CtrlLpIO->pitch_command[0] *R2D* 100;
   _temp = CtrlMRAC2->uc2_Tilt[0]*R2D*100;
//  _temp = pid[8].dout2*100;
	tran->TxDat[32]=BYTE1(_temp);
	tran->TxDat[33]=BYTE0(_temp);	
	                       
	//_temp = ahrs[1].pqr[0]*R2D*100;												//userdata16			z速度
	_temp = -(gps->alti - gps->alti_off)*100;
	tran->TxDat[34]=BYTE1(_temp);
	tran->TxDat[35]=BYTE0(_temp);	
	               
	//_temp = ahrs[1].pqr[1]*R2D*100;												//userdata17     x速度测量
	_temp = gps->gpsPosAccuracy * 100;
	tran->TxDat[36]=BYTE1(_temp);
	tran->TxDat[37]=BYTE0(_temp);	
	            
	//_temp = ahrs[1].pqr[2]*R2D*100;												//userdata18	    y速度测量
	_temp= gps->gpsHeiAccuracy * 100;
	tran->TxDat[38]=BYTE1(_temp);
	tran->TxDat[39]=BYTE0(_temp);	
	
	//_temp=CtrlFbck->Z[0]*100;											//userdata19      z速度测量
	_temp= CtrlIO->mt_output[0];
	tran->TxDat[40]=BYTE1(_temp);
	tran->TxDat[41]=BYTE0(_temp);	
	
	_temp= CtrlFbck->engine_speed;									//userdata20      转速
	//_temp = CtrlFbck->Z[0]*100;                  //userdata13   高度
	tran->TxDat[42]=BYTE1(_temp);
	tran->TxDat[43]=BYTE0(_temp);	
	
	u8 sum = 0;
	for(u8 i=0;i<44;i++)sum += tran->TxDat[i];
	tran->TxDat[44]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 45);
}
inline bool Send_User2(void)
{
	if(tran->TxFlag == true)
	{
		return false;
	}
	vs16 _temp;

	tran->TxDat[2]=0xF1;
	tran->TxDat[3]=40;//20*2
	
//	
_temp =  ICM20602->AccRel[0] * 1000;//userdata1
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
_temp =  ICM20602->AccRel[1] * 1000;//userdata2
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
_temp =  ICM20602->AccRel[2] * 1000;//userdata3
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
_temp =  ICM20602->GyrRel[0] * 1000;//userdata4
	tran->TxDat[10]=BYTE1(_temp);
	tran->TxDat[11]=BYTE0(_temp);
_temp =  ICM20602->GyrRel[1] * 1000;//userdata5
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
_temp =  ICM20602->GyrRel[2] * 1000;//userdata6
	tran->TxDat[14]=BYTE1(_temp);
	tran->TxDat[15]=BYTE0(_temp);
	_temp = CtrlFbck->pqr[0]*R2D*100;//userdata7
	tran->TxDat[16]=BYTE1(_temp);
	tran->TxDat[17]=BYTE0(_temp);
	_temp = CtrlFbck->pqr[1]*R2D*100;//userdata8
	tran->TxDat[18]=BYTE1(_temp);
	tran->TxDat[19]=BYTE0(_temp);
	_temp = CtrlFbck->pqr[2]*R2D*100;//userdata9
	tran->TxDat[20]=BYTE1(_temp);
	tran->TxDat[21]=BYTE0(_temp);
	_temp = CtrlLpIO->pqr_command[0]*R2D*100;//userdata10
	tran->TxDat[22]=BYTE1(_temp);
	tran->TxDat[23]=BYTE0(_temp);
	_temp = CtrlLpIO->pqr_command[1]*R2D*100;//userdata11
	tran->TxDat[24]=BYTE1(_temp);
	tran->TxDat[25]=BYTE0(_temp);
	_temp = CtrlLpIO->pqr_command[2]*R2D*100;//userdata12
	tran->TxDat[26]=BYTE1(_temp);
	tran->TxDat[27]=BYTE0(_temp);
	
//	_temp =  eso->x_k[6]*100;													//userdata13		z位置测量
	//_temp=CtrlIO->cs_output[0]*100;
//	_temp = CtrlFbck->Ang[2]*R2D * 100;
  _temp = CtrlIO->output[0]*R2D*100;
	tran->TxDat[28]=BYTE1(_temp);
	tran->TxDat[29]=BYTE0(_temp);

//	_temp =  eso->x_k[7]*100;												//userdata14    x速度 
	_temp = CtrlIO->output[1]*R2D*100;
	tran->TxDat[30]=BYTE1(_temp);
	tran->TxDat[31]=BYTE0(_temp);
	                 
//	_temp =  eso->x_k[8]*100;												//userdata15     y速度
//	_temp = CtrlLpIO->pitch_command[0] *R2D* 100;
   _temp = CtrlIO->output[2]*R2D*100;
//  _temp = pid[8].dout2*100;
	tran->TxDat[32]=BYTE1(_temp);
	tran->TxDat[33]=BYTE0(_temp);	
	                       
	//_temp = ahrs[1].pqr[0]*R2D*100;												//userdata16			z速度
	_temp = CtrlLpIO->Phase;
	tran->TxDat[34]=BYTE1(_temp);
	tran->TxDat[35]=BYTE0(_temp);	
	               
	//_temp = ahrs[1].pqr[1]*R2D*100;												//userdata17     x速度测量
	_temp = gps->gpsPosAccuracy * 100;
	tran->TxDat[36]=BYTE1(_temp);
	tran->TxDat[37]=BYTE0(_temp);	
	            
	//_temp = ahrs[1].pqr[2]*R2D*100;												//userdata18	    y速度测量
	_temp= gps->gpsHeiAccuracy * 100;
	tran->TxDat[38]=BYTE1(_temp);
	tran->TxDat[39]=BYTE0(_temp);	
	
	//_temp=CtrlFbck->Z[0]*100;											//userdata19      z速度测量
	_temp= CtrlIO->mt_output[0];
	tran->TxDat[40]=BYTE1(_temp);
	tran->TxDat[41]=BYTE0(_temp);	
	
	_temp= CtrlFbck->engine_speed;									//userdata20      转速
	//_temp = CtrlFbck->Z[0]*100;                  //userdata13   高度
	tran->TxDat[42]=BYTE1(_temp);
	tran->TxDat[43]=BYTE0(_temp);	
	
	u8 sum = 0;
	for(u8 i=0;i<44;i++)sum += tran->TxDat[i];
	tran->TxDat[44]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 45);
}
inline bool Send_User0(void)
{
	if(tran->TxFlag == true)
	{
		return false;
	}
	vs16 _temp;

	tran->TxDat[2]=0xF1;
	tran->TxDat[3]=40;//20*2
	
//	
_temp =  ahrs[1].Ang[0]*R2D*100;//userdata1
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
	_temp =  ahrs[1].Ang[1]*R2D*100;//userdata2
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	_temp =  ahrs[1].Ang[2]*R2D*100;//userdata3
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	_temp = CtrlLpIO->roll_command[0]*R2D *100;//userdata4
	tran->TxDat[10]=BYTE1(_temp);
	tran->TxDat[11]=BYTE0(_temp);
	_temp = CtrlLpIO->pitch_command[0]*R2D *100;//userdata5
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
	_temp = CtrlLpIO->yaw_command[0]*R2D *100;//userdata6
	tran->TxDat[14]=BYTE1(_temp);
	tran->TxDat[15]=BYTE0(_temp);
	_temp = CtrlFbck->pqr[0]*R2D*100;//userdata7
	tran->TxDat[16]=BYTE1(_temp);
	tran->TxDat[17]=BYTE0(_temp);
	_temp = CtrlFbck->pqr[1]*R2D*100;//userdata8
	tran->TxDat[18]=BYTE1(_temp);
	tran->TxDat[19]=BYTE0(_temp);
	_temp = CtrlFbck->pqr[2]*R2D*100;//userdata9
	tran->TxDat[20]=BYTE1(_temp);
	tran->TxDat[21]=BYTE0(_temp);
	_temp = CtrlLpIO->pqr_command[0]*R2D*100;//userdata10
	tran->TxDat[22]=BYTE1(_temp);
	tran->TxDat[23]=BYTE0(_temp);
	_temp = CtrlLpIO->pqr_command[1]*R2D*100;//userdata11
	tran->TxDat[24]=BYTE1(_temp);
	tran->TxDat[25]=BYTE0(_temp);
	_temp = CtrlLpIO->pqr_command[2]*R2D*100;//userdata12
	tran->TxDat[26]=BYTE1(_temp);
	tran->TxDat[27]=BYTE0(_temp);
	
//	_temp =  eso->x_k[6]*100;													//userdata13		z位置测量
	//_temp=CtrlIO->cs_output[0]*100;
//	_temp = CtrlFbck->Ang[2]*R2D * 100;
  _temp = CtrlMRAC2->uc2[0]*R2D*100;
	tran->TxDat[28]=BYTE1(_temp);
	tran->TxDat[29]=BYTE0(_temp);

//	_temp =  eso->x_k[7]*100;												//userdata14    x速度 
	_temp = CtrlLpIO->roll_command[0] *R2D* 100;
	tran->TxDat[30]=BYTE1(_temp);
	tran->TxDat[31]=BYTE0(_temp);
	                 
//	_temp =  eso->x_k[8]*100;												//userdata15     y速度
//	_temp = CtrlLpIO->pitch_command[0] *R2D* 100;
   _temp = CtrlMRAC2->uc2_Tilt[0]*R2D*100;
//  _temp = pid[8].dout2*100;
	tran->TxDat[32]=BYTE1(_temp);
	tran->TxDat[33]=BYTE0(_temp);	
	                       
	//_temp = ahrs[1].pqr[0]*R2D*100;												//userdata16			z速度
	_temp = CtrlLpIO->Phase;
	tran->TxDat[34]=BYTE1(_temp);
	tran->TxDat[35]=BYTE0(_temp);	
	               
	//_temp = ahrs[1].pqr[1]*R2D*100;												//userdata17     x速度测量
	_temp = gps->gpsPosAccuracy * 100;
	tran->TxDat[36]=BYTE1(_temp);
	tran->TxDat[37]=BYTE0(_temp);	
	            
	//_temp = ahrs[1].pqr[2]*R2D*100;												//userdata18	    y速度测量
	_temp= gps->gpsHeiAccuracy * 100;
	tran->TxDat[38]=BYTE1(_temp);
	tran->TxDat[39]=BYTE0(_temp);	
	
	//_temp=CtrlFbck->Z[0]*100;											//userdata19      z速度测量
	_temp= CtrlIO->mt_output[0];
	tran->TxDat[40]=BYTE1(_temp);
	tran->TxDat[41]=BYTE0(_temp);	
	
	_temp= CtrlFbck->engine_speed;									//userdata20      转速
	//_temp = CtrlFbck->Z[0]*100;                  //userdata13   高度
	tran->TxDat[42]=BYTE1(_temp);
	tran->TxDat[43]=BYTE0(_temp);	
	
	u8 sum = 0;
	for(u8 i=0;i<44;i++)sum += tran->TxDat[i];
	tran->TxDat[44]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 45);
}
inline bool Send_User3(void)
{
	if(tran->TxFlag == true)
	{
		return false;
	}
	vs16 _temp;

	tran->TxDat[2]=0xF1;
	tran->TxDat[3]=40;//20*2
	
//	
_temp = CtrlLpIO->X_command[1]*100;             //userdata1		滚转角
	tran->TxDat[4]=BYTE1(_temp);
	tran->TxDat[5]=BYTE0(_temp);
	
	_temp = CtrlLpIO->Y_command[1]*100;  					//userdata2		俯仰角
//	_temp = -ICM20602->AccFil[0]* 100;	
	tran->TxDat[6]=BYTE1(_temp);
	tran->TxDat[7]=BYTE0(_temp);
	
	_temp = CtrlLpIO->Z_command[1]*100;  							//userdata3	   偏航角
	//_temp = -ICM20602->AccFil_3nd2[0]*100;
	tran->TxDat[8]=BYTE1(_temp);
	tran->TxDat[9]=BYTE0(_temp);
	
	_temp = CtrlFbck->XH[1] * 100; 			//userdata4		X速度给定
	tran->TxDat[10]=BYTE1(_temp);
	tran->TxDat[11]=BYTE0(_temp);
	
	_temp = CtrlFbck->YH[1] * 100;  						//userdata5	  X速度
	tran->TxDat[12]=BYTE1(_temp);
	tran->TxDat[13]=BYTE0(_temp);
	
	_temp = CtrlFbck->Z[1] * 100;  					//userdata6			Y速度给定
	tran->TxDat[14]=BYTE1(_temp);
	tran->TxDat[15]=BYTE0(_temp);
	
	_temp = CtrlLpIO->Z_command[0] * 100;								//userdata7	  	Y速度
	tran->TxDat[16]=BYTE1(_temp);
	tran->TxDat[17]=BYTE0(_temp);

	
	_temp = CtrlFbck->Z[0] * 100;           //userdata8   X位置
	tran->TxDat[18]=BYTE1(_temp);
	tran->TxDat[19]=BYTE0(_temp);
	
	_temp = CtrlFbck->X[0] * 100;         //userdata9	   Y位置
	tran->TxDat[20]=BYTE1(_temp);
	tran->TxDat[21]=BYTE0(_temp);
	
	_temp = CtrlFbck->Y[0] * 100;									//userdata10	   Z位置
	tran->TxDat[22]=BYTE1(_temp);
	tran->TxDat[23]=BYTE0(_temp);
	
	_temp = CtrlFbck->Ang[0]*R2D*100;
	tran->TxDat[24]=BYTE1(_temp);
	tran->TxDat[25]=BYTE0(_temp);
	
	//_temp =  eso->x_k[5]*R2D*100;												//userdata12    y位置测量
	_temp = CtrlFbck->Ang[1]*R2D*100;
	tran->TxDat[26]=BYTE1(_temp);
	tran->TxDat[27]=BYTE0(_temp);
	
//	_temp =  eso->x_k[6]*100;													//userdata13		z位置测量
	//_temp=CtrlIO->cs_output[0]*100;
//	_temp = CtrlFbck->Ang[2]*R2D * 100;
  _temp = CtrlLpIO->roll_command[0] *R2D* 100;
	tran->TxDat[28]=BYTE1(_temp);
	tran->TxDat[29]=BYTE0(_temp);

//	_temp =  eso->x_k[7]*100;												//userdata14    x速度 
	_temp = CtrlLpIO->pitch_command[0] *R2D* 100;
	tran->TxDat[30]=BYTE1(_temp);
	tran->TxDat[31]=BYTE0(_temp);
	
	_temp = CtrlLpIO->position_mode*100;
//  _temp = pid[8].dout2*100;
	tran->TxDat[32]=BYTE1(_temp);
	tran->TxDat[33]=BYTE0(_temp);	
	                       
	//_temp = ahrs[1].pqr[0]*R2D*100;												//userdata16			z速度
	_temp = CtrlLpIO->brake_mode*100;
	tran->TxDat[34]=BYTE1(_temp);
	tran->TxDat[35]=BYTE0(_temp);	
	               
	//_temp = ahrs[1].pqr[1]*R2D*100;												//userdata17     x速度测量
	_temp = gps->gpsPosAccuracy * 100;
	tran->TxDat[36]=BYTE1(_temp);
	tran->TxDat[37]=BYTE0(_temp);	
	            
	//_temp = ahrs[1].pqr[2]*R2D*100;												//userdata18	    y速度测量
	_temp= gps->gpsHeiAccuracy * 100;
	tran->TxDat[38]=BYTE1(_temp);
	tran->TxDat[39]=BYTE0(_temp);	
	
	//_temp=CtrlFbck->Z[0]*100;											//userdata19      z速度测量
	_temp= CtrlIO->control_mode;
	tran->TxDat[40]=BYTE1(_temp);
	tran->TxDat[41]=BYTE0(_temp);	
	
	_temp= CtrlFbck->engine_speed;									//userdata20      转速
	//_temp = CtrlFbck->Z[0]*100;                  //userdata13   高度
	tran->TxDat[42]=BYTE1(_temp);
	tran->TxDat[43]=BYTE0(_temp);	
	
	u8 sum = 0;
	for(u8 i=0;i<44;i++)sum += tran->TxDat[i];
	tran->TxDat[44]=sum;
	return USART3_Transmit_DMA((u8 *)tran->TxDat, 45);
}

/***************END OF FILE************/
