#include "ms5611.h"
//#include "bell.h"
#include "pid.h"

/***************************************************\
功能：
  ma5611 气压计SPI读取
说明：
  1、修改端口,配置成8bit模式，high 2edge（模式3）或者 low 1edge
  2、每次读取完数据需要将Update复位
  3、气压计可考虑升级为M8378
\***************************************************/

sMS5611 ms5611[MS5611_NUM];

void TIM5_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
	  Ms5611_Loop_1ms(&ms5611[0]);
		//Ms5611_Loop_1ms(&ms5611[1]);
  }
}

/******************驱动程序****************/
//spi只发送和接收数据，至于数据怎么处理，相应哪一部分是什么意义，是由器件决定的。
//ms5611接收到的数据为命令值，其表见头文件，有复位\转换\读取等指令（8bit），
//若发送的命令为读取数据，则ms5611发送对应数据（长度可能大于8bit）

#define MS5611_SPI2_CS_CLR	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET)
#define MS5611_SPI2_CS_SET	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET)

void Ms5611_Spi_Tran(SPI_HandleTypeDef *hspi,u8 Cmd)
{
	MS5611_SPI2_CS_CLR;
	HAL_SPI_Transmit(hspi,&Cmd,1,10);
	MS5611_SPI2_CS_SET;
}

void Ms5611_Spi_TranRecv(SPI_HandleTypeDef *hspi,u8 Cmd,u8* ReadData,u8 num)
{
	MS5611_SPI2_CS_CLR;
	HAL_SPI_Transmit(hspi,&Cmd,1,10);
	HAL_SPI_Receive(hspi,ReadData,num,10);
	MS5611_SPI2_CS_SET;
}

/******************功能函数****************/
void Ms5611_Init_Para(sMS5611 *ele,sMS5611_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	ele->Cal = false;
	
    ele->hspi = elecfg->hspi;
	ele->name = elecfg->name;
    ele->Filt.CNT = 0;
    ele->Filt.CCR = elecfg->FiltNum;
	ele->Delay = elecfg->Delay;
	ele->Time = 0;
	ele->StaCnt = 0;
	ele->AltOff = 0.0f;
	for(u8 i=0;i<MS_SLOPE_NUM;i++)
		ele->AltTmp[i]=0;
	
	ele->RxFlag = false;
}

bool Ms5611_Init(sMS5611 *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
    //配置
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	Ms5611_Spi_Tran(ele->hspi,MS5611_CMD_RESET);
	HAL_Delay(100);				// 延时	
    //检测
    ele->Sta = STA_CHK;
	u8 rxbuf[2] = { 0, 0 };  
	for(u8 i=0;i<7;i++)
	{
		Ms5611_Spi_TranRecv(ele->hspi,MS5611_CMD_PROM_RD+i*2,rxbuf,2);
		ele->PROM[i] = ((u16)rxbuf[0]<<8) | rxbuf[1];
		if(ele->PROM[i]==0x00||ele->PROM[i]==0xFFFF)ele->Err=ERR_LOST;
	}
	if(ele->Err==ERR_LOST)  //0x3FED
	{
		Dprintf("--Chk [NO]\r\n");
        ele->Err=ERR_LOST;
        return false;
    }
    Dprintf("--Chk [OK]\r\n");
    //初始化完毕
    ele->Sta = STA_RUN;
	ele->Cal = true;
    Dprintf("%s Init.[OK]\r\n",ele->name);
    return true;
}

static bool Ms5611_Cali(sMS5611 *ele)
{
	static sMS5611 *ready = NULL;
	if(ready==NULL)ready=ele;
	else if(ready!=ele)return false;
	
	static bool Ms5611IsCali = false;
	if(ele->Cal==true)
	{
		ele->Cal = false;
		ele->Sta = STA_CAL;
		Ms5611IsCali = true;
		Dprintf("\r\n%s Cali\r\n",ele->name);
	}
	if(Ms5611IsCali==false)return true;
	static u16 Cali=0;   //下次进来就不用等待了
	if(Cali<500){Cali++;return false;}     //忽略前面500次
	#define SAMPLE_NUM 100  
	static u16 SmpCnt=0;
	static float AltSum=0.0f;
	if(SmpCnt++<SAMPLE_NUM)    
	{
		AltSum+=ele->AltRel;
		return false;
	}
	SmpCnt=0;
	ele->AltOff = AltSum/SAMPLE_NUM;
	AltSum=0.0f;
	Dprintf("\r\n%s Cali--Bias is:%4.2f\r\n",ele->name,ele->AltOff);
	ele->Sta=STA_RUN;
	Ms5611IsCali = false;
	ready = NULL;
	return true;
}

bool Ms5611_Calc(sMS5611 *ele)
{
	if(ele->Sta != STA_CAL && ele->Sta != STA_RUN)return false;
	Tim_Calc(&ele->Tim);   //计时 
	if(ele->RxFlag == false)return false;
	ele->RxFlag = false;
	s32 dT, TEMP, P, ALT;
	s64 OFF1,SENS,T2=0,OFF2=0,SENS2=0,Delt;
	u32 ADC_TMP;  // ADC result of temperature measurement
	u32 ADC_PRS;  // ADC result of pressure measurement
		Tim_Calc(&ele->Tim);
		//EKF需要的更新时间
		ele->height_update_time = ele->Tim.CNT;
		ele->gps_ins_update_flag = true;
		//EKF需要的更新时间
	ADC_TMP = ((u32)ele->TmpRaw[0] << 16) | ((u32)ele->TmpRaw[1] << 8) | ele->TmpRaw[2];
	ADC_PRS = ((u32)ele->PrsRaw[0] << 16) | ((u32)ele->PrsRaw[1] << 8) | ele->PrsRaw[2];

	if(ADC_TMP==0||ADC_TMP==0xFFFFFF||ADC_PRS==0||ADC_PRS==0xFFFFFF)
	{
		ele->Err=ERR_LOST;
		return false;
	}
	//计算温度
	dT   = (s32)ADC_TMP - ((s32)ele->PROM[5] << 8);	
	TEMP = 2000 + (((int64_t)dT * ele->PROM[6]) >> 23);  //0.01°C
	//计算气压的温度补偿
	OFF1 = ((s64)ele->PROM[2] << 16) + (((s64)dT * ele->PROM[4]) >> 7);
	SENS = ((s64)ele->PROM[1] << 15) + (((s64)dT * ele->PROM[3]) >> 8);
	//温度低于20度的二次补偿
	if(TEMP < 2000)	// 温度低于20度
	{
		T2    = (((s64)dT)*dT) >> 31;
		Delt  = TEMP - 2000;
		Delt  = Delt * Delt;
		OFF2  = (5 * Delt) >> 1;
		SENS2 = (5 * Delt) >> 2;
		if(TEMP < -1500) // 温度低于-15度
		{
			Delt = TEMP + 1500;
			Delt = Delt * Delt;
			OFF2  += 7 * Delt;
			SENS2 += (11 * Delt) >> 1;
		}
	}
	OFF1 -= OFF2;
	SENS -= SENS2;
	//最终的温度和气压值
	TEMP -= T2;
	P = (((ADC_PRS * SENS) >> 21) - OFF1) >> 15;    //0.01mbar = Pa
	//计算高度
	ALT = (s32)((1.0f - pow(P/MSLP, 0.190295f))*4433000.0f);  //海拔，单位：cm
	
	//实际值解算
	ele->TmpRel = (float)TEMP/100.0f;
	ele->PrsRel = (float)P/100.0f;
	ele->AltRel = (float)ALT/100.0f;
	Ms5611_Cali(ele);
	ele->AltRel -= ele->AltOff;
	//滤波
    SlideFilt(&ele->TmpFil,&ele->TmpRel,1,&ele->Filt,1);
	SlideFilt(&ele->PrsFil,&ele->PrsRel,1,&ele->Filt,2);
	SlideFilt(&ele->AltFil,&ele->AltRel,1,&ele->Filt,2);
	
	for(u8 i=1;i<MS_SLOPE_NUM;i++) ele->AltTmp[i-1]=ele->AltTmp[i];
	ele->AltTmp[MS_SLOPE_NUM-1]=ele->AltRel;
	float ab[2];
	LineFit(ele->AltTmp,MS_SLOPE_NUM,ab);  //最小二乘法拟合速率
	ele->AltSlope = ab[0]*50.0f;           //0.02s
	
	if(ele->Sta==STA_RUN)ele->Update = true;
	Tim_Calc(&ele->Tim);   //计时 
	ele->Time = ele->Tim.OUT;
	return true;
}

//气压计状态机，一毫秒调用一次
void Ms5611_Loop_1ms(sMS5611 *ele)
{
	if(ele->Delay>0){ele->Delay--;return;}
	if(ele->StaCnt<30)ele->StaCnt++;
	else ele->StaCnt=0;
	switch(ele->StaCnt)
	{
		case 0:Ms5611_Spi_TranRecv(ele->hspi,MS5611_CMD_ADC_READ,ele->PrsRaw,3); //读取气压
					 Ms5611_Spi_Tran(ele->hspi,MS5611_CMD_D2_4096);              //开始采集温度
			     ele->RxFlag = true;break;
		case 15:Ms5611_Spi_TranRecv(ele->hspi,MS5611_CMD_ADC_READ,ele->TmpRaw,3);//读取温度
		        Ms5611_Spi_Tran(ele->hspi,MS5611_CMD_D1_4096);break;             //开始采集气压
		default:break;
	}
}
void Ms5611_Ctr(sMS5611 *ele)   //控制气压计温度
{
	pidTmp->feedback=ms5611[0].TmpRel;
	bell->htim->Instance->CNT=0;
	bell->htim->Instance->CCR1=(u16)Pid_tmp(pidTmp);   //CCR1 max为999     1ms
	 
}
/*********************************** END OF FILE ***********************************/

