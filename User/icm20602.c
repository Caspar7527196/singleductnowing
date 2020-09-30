#include "icm20602.h"
sICM20602 ICM20602[ICM20602_NUM];

//驱动程序：spi读写ICM20602底层程序
#define ICM20602_SPI4_CS_CLR	HAL_GPIO_WritePin(SPI4_NSS_AG_GPIO_Port, SPI4_NSS_AG_Pin, GPIO_PIN_RESET)
#define ICM20602_SPI4_CS_SET	HAL_GPIO_WritePin(SPI4_NSS_AG_GPIO_Port, SPI4_NSS_AG_Pin, GPIO_PIN_SET)

static void Icm20602_Spi_Tran(SPI_HandleTypeDef *hspi,u8 add,u8 dat)
{
	ICM20602_SPI4_CS_CLR;
	HAL_SPI_Transmit(hspi,&add,1,10);
	HAL_SPI_Transmit(hspi,&dat,1,10);
	ICM20602_SPI4_CS_SET;
}

static u8 Icm20602_Spi_TranRecv(SPI_HandleTypeDef *hspi,u8 TxData)
{
	u8 RxData;
	ICM20602_SPI4_CS_CLR;
	HAL_SPI_Transmit(hspi,&TxData,1,10);
	HAL_SPI_Receive(hspi,&RxData,1,10);
	ICM20602_SPI4_CS_SET;
	return RxData;
}
//驱动程序：spi读写ICM20602底层程序

//功能函数
void ICM20602_Init_Para(sICM20602 *ele,sICM20602_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;

	ele->hspi = elecfg->hspi;
	ele->name = elecfg->name;
	ele->Filt.CNT = 0;
	ele->Filt.CCR = elecfg->FiltNum;
	if(Dir_Trans(ele->Dir,elecfg->Dir)==false)ele->Err = ERR_SOFT;
	ele->OneG = 9.788f;
	for(u8 i = 0;i < 3;i++)
	{
		ele->AccOff[i] = elecfg->AccOff[i];
//		ele->GyrOff[i] = elecfg->GyrOff[i];
	}
	ele->imuPreSampleTime_ms = 0;
	ele->imuSampleTime_ms = 0;
	
	ele->delAng_acc_dt = 0.0f;
	ele->delVel_acc_dt = 0.0f;
}

bool Icm20602_Init(sICM20602 *ele)
{

	Dprintf("\r\n%s Init...\r\n",ele->name);
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_PWR_MGMT_1,0x80);
	HAL_Delay(10);
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_PWR_MGMT_1,0x01);
	HAL_Delay(10);
	
	ele->Sta = STA_CHK;
	u8 ID;
	ID = Icm20602_Spi_TranRecv(ele->hspi,MPUREG_WHOAMI|0x80);		//读取传感器的ID号
	Dprintf("ID:%d\r\n",ID);
	if(ID != MPU_WHOAMI_20602)
	{
		Dprintf("--Chk [NO]\r\n");
    ele->Err=ERR_LOST;
    return false;
	}
	Dprintf("--Chk [OK]\r\n");
	
	/*复位加速度计和温度计reg*/
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_SIGNAL_PATH_RESET,0x03);
	HAL_Delay(10);
  /*复位陀螺仪reg*/
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_USER_CTRL,0x01);	
	HAL_Delay(10);

	Icm20602_Spi_Tran(ele->hspi,0x70,0x40);//dmp ？？？？？
	HAL_Delay(10);
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_PWR_MGMT_2,0x00);
	HAL_Delay(10);
	/*设置采样频率为1KHz*/
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_SMPLRT_DIV,0);
	HAL_Delay(10);

	Icm20602_Spi_Tran(ele->hspi,MPU_RA_CONFIG,ICM20602_LPF_20HZ);
	HAL_Delay(10);
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_GYRO_CONFIG,(3 << 3));	//陀螺仪量程正负2000dps
	HAL_Delay(10);
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_ACCEL_CONFIG,(2 << 3));	//加速度计量程正负8g
	HAL_Delay(10);
	/*加速度计LPF 10HZ*/
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_ACCEL_CONFIG2,0x05);
	HAL_Delay(10);
	/*关闭低功耗*/
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_LP_MODE_CFG,0x00);
	HAL_Delay(10);
	/*关闭FIFO*/
	Icm20602_Spi_Tran(ele->hspi,MPU_RA_FIFO_EN,0x00);
	HAL_Delay(10);
	
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);

	return true;

}

static bool Icm20602_Cali(sICM20602 *ele)
{
	
	static sICM20602 *ready = NULL;
	if(ready==NULL)ready=ele;
	else if(ready!=ele)return false;
	
	static bool GyrIsCali = false;
	if(ele->GyrCal==true)
	{
		ele->GyrCal = false;
		ele->Sta = STA_CAL;
		GyrIsCali = true;
		Dprintf("\r\n%s Gyr Cali\r\n",ele->name);
	}
	if(GyrIsCali!=true)return true;
	#define GYR_SAMPLE_NUM 300  //3s     
	static float  GyrSum[3] = {0.0f, 0.0f, 0.0f};
	static u16 SmpCnt = 0;
	static u8 OneSmpCnt = 0;
	if(OneSmpCnt++<2)return false;  //2个周期采一次数据
	if(SmpCnt++<GYR_SAMPLE_NUM)    //采集由采集函数进行
	{
		OneSmpCnt=0;
		GyrSum[0] += (float)ele->GyrRaw[0];
		GyrSum[1] += (float)ele->GyrRaw[1];
		GyrSum[2] += (float)ele->GyrRaw[2];
		return false;
	}
	SmpCnt = 0;
	ele->GyrOff[0] = GyrSum[0]/(float)GYR_SAMPLE_NUM;
	ele->GyrOff[1] = GyrSum[1]/(float)GYR_SAMPLE_NUM;
	ele->GyrOff[2] = GyrSum[2]/(float)GYR_SAMPLE_NUM;
	GyrSum[0] = 0;
	GyrSum[1] = 0;
	GyrSum[2] = 0;
	
	Dprintf("\r\n%s Cali--Bias is:%d %d %d\r\n",ele->name,ele->GyrOff[0],ele->GyrOff[1],ele->GyrOff[2]);
	ele->Sta = STA_RUN;
	GyrIsCali = false;
	ready = NULL;
	return true;	

}

bool Icm20602_Read(sICM20602 *ele)
{
	u8 icm_addr[14]={
		MPU_RA_ACCEL_XOUT_H,MPU_RA_ACCEL_XOUT_L,MPU_RA_ACCEL_YOUT_H,MPU_RA_ACCEL_YOUT_L,
		MPU_RA_ACCEL_ZOUT_H,MPU_RA_ACCEL_ZOUT_L,MPU_RA_TEMP_OUT_H,MPU_RA_TEMP_OUT_L,
		MPU_RA_GYRO_XOUT_H,MPU_RA_GYRO_XOUT_L,MPU_RA_GYRO_YOUT_H,MPU_RA_GYRO_YOUT_L,
		MPU_RA_GYRO_ZOUT_H,MPU_RA_GYRO_ZOUT_L
	};
	
	u8 icm_dat[14];
	for(int i = 0;i < 14;i++)
	{
		icm_dat[i] = Icm20602_Spi_TranRecv(ele->hspi,icm_addr[i]|0x80);
	}
	
	ele->AccRaw[0] = (icm_dat[0]<<8) + icm_dat[1];
	ele->AccRaw[1] = (icm_dat[2]<<8) + icm_dat[3];
	ele->AccRaw[2] = (icm_dat[4]<<8) + icm_dat[5];
	ele->TmpRaw[0] = (icm_dat[6]<<8) + icm_dat[7];
	ele->GyrRaw[0] = (icm_dat[8]<<8) + icm_dat[9];
	ele->GyrRaw[1] = (icm_dat[10]<<8) + icm_dat[11];
	ele->GyrRaw[2] = (icm_dat[12]<<8) + icm_dat[13];

//	Dprintf("AccRaw:%d,%d,%d ,GyrRaw:%d,%d,%d,TmpRaw:%d, Tim:%d\r\n",ele->AccRaw[0],ele->AccRaw[1],ele->AccRaw[2],ele->GyrRaw[0],ele->GyrRaw[1],ele->GyrRaw[2],ele->TmpRaw[0],ele->Time);
	return true;
}



	float _tmp_acc[3] = {0};
	float _tmp_gyr[3] = {0};
	
void LowPassFilt (sICM20602 *ele,float filt_para)
{
	ele->AccFil[0] += filt_para * (ele->AccRel[0] - ele->AccFil[0]);
	ele->AccFil[1] += filt_para * (ele->AccRel[1] - ele->AccFil[1]);
	ele->AccFil[2] += filt_para * (ele->AccRel[2] - ele->AccFil[2]);
}

void Butterworth_2nd(sICM20602 *ele)
{
	//15HZ..19.10.28
	#define a0 0.2928932188134524f 
	#define a1 0.5857864376269048f
	#define a2 0.2928932188134524f 
	#define b1 -0.00000000000000013007f
	#define b2  0.1715728752538099588509f

	static float A_last[3] = {0,0,0};
	static float A_last_last[3] = {0,0,0};
	static float B_last[3] = {0,0,0};
	static float B_last_last[3] = {0,0,0};
	
	for(int i=0;i<3;i++)
	{
		ele->GyrFil_2nd[i] = a0 * ele->GyrRel[i] + a1 * A_last[i] + a2 * A_last_last[i] - b1 * B_last[i] - b2 * B_last_last[i];
	}
	
	for(int i=0;i<3;i++)
	{
		A_last_last[i] = A_last[i];
		A_last[i] = ele->GyrRel[i];
		
		B_last_last[i] = B_last[i];
		B_last[i] = ele->GyrFil_2nd[i];
	}
}

void Butterworth_3nd(sICM20602 *ele)
{
	//XY 10HZ   Z 10HZ   19.10.30
	#define a0_3nd 0.002898194633721429703393512866682613094f  
	#define a1_3nd 0.008694583901164289543861407594249612885f 
	#define a2_3nd 0.008694583901164289543861407594249612885f 
	#define a3_3nd 0.002898194633721429703393512866682613094f
	#define b1_3nd -2.374094743709352250959909724770113825798f
	#define b2_3nd  1.929355669091215252919369049777742475271f
	#define b3_3nd -0.532075368312091900868665561574744060636f
	static float A_last_3nd[3] = {0,0,0};
	static float A_last_last_3nd[3] = {0,0,0};
	static float A_last_last_last_3nd[3] = {0,0,0};
	static float B_last_3nd[3] = {0,0,0};
	static float B_last_last_3nd[3] = {0,0,0};
	static float B_last_last_last_3nd[3] = {0,0,0};
	#define a0_3nd2 0.002898194633721429703393512866682613094f  
	#define a1_3nd2 0.008694583901164289543861407594249612885f 
	#define a2_3nd2 0.008694583901164289543861407594249612885f 
	#define a3_3nd2 0.002898194633721429703393512866682613094f
	#define b1_3nd2 -2.374094743709352250959909724770113825798f
	#define b2_3nd2  1.929355669091215252919369049777742475271f
	#define b3_3nd2 -0.532075368312091900868665561574744060636f
	for(int i=0;i<2;i++)
	{
		ele->AccFil_3nd[i] = a0_3nd * ele->AccRel[i] + a1_3nd * A_last_3nd[i] + a2_3nd * A_last_last_3nd[i] + a3_3nd * A_last_last_last_3nd[i] - b1_3nd * B_last_3nd[i] - b2_3nd * B_last_last_3nd[i] - b3_3nd * B_last_last_last_3nd[i];
	}
		ele->AccFil_3nd[2] = a0_3nd2 * ele->AccRel[2] + a1_3nd2 * A_last_3nd[2] + a2_3nd2 * A_last_last_3nd[2] + a3_3nd2 * A_last_last_last_3nd[2] - b1_3nd2 * B_last_3nd[2] - b2_3nd2 * B_last_last_3nd[2] - b3_3nd2 * B_last_last_last_3nd[2];

	for(int i=0;i<3;i++)
	{
		A_last_last_last_3nd[i] = A_last_last_3nd[i];
		A_last_last_3nd[i] = A_last_3nd[i];
		A_last_3nd[i] = ele->AccRel[i];
		
		B_last_last_last_3nd[i] = B_last_last_3nd[i];
		B_last_last_3nd[i] = B_last_3nd[i];
		B_last_3nd[i] = ele->AccFil_3nd[i];
	}
}

void Butterworth_3nd2(sICM20602 *ele)
{
	#define a0_3nd3 0.000003756838019f
	#define a1_3nd3 0.0000112705140592f
	#define a2_3nd3 0.0000112705140592f
	#define a3_3nd3 0.0000037568380197f
	#define b1_3nd3 -2.937170728449890f
	#define b2_3nd3  2.876299723479331f
	#define b3_3nd3 -0.939098940325282f
	static float A_last_3nd2[3] = {0,0,0};
	static float A_last_last_3nd2[3] = {0,0,0};
	static float A_last_last_last_3nd2[3] = {0,0,0};
	static float B_last_3nd2[3] = {0,0,0};
	static float B_last_last_3nd2[3] = {0,0,0};
	static float B_last_last_last_3nd2[3] = {0,0,0};
	#define a0_3nd4 0.000029146494465697653702745614778812921f
	#define a1_3nd4 0.000087439483397092957720105055319237408f
	#define a2_3nd4 0.000087439483397092957720105055319237408f
	#define a3_3nd4 0.000029146494465697653702745614778812921f
	#define b1_3nd4 -2.874356892677484509590613015461713075638f
	#define b2_3nd4  2.756483195225695403962618001969531178474f
	#define b3_3nd4 -0.881893130592485419150250436359783634543f
	for(int i=0;i<2;i++)
	{
		ele->AccFil_3nd2[i] = a0_3nd3 * ele->AccRel[i] + a1_3nd3 * A_last_3nd2[i] + a2_3nd3 * A_last_last_3nd2[i] + a3_3nd3 * A_last_last_last_3nd2[i] - b1_3nd3 * B_last_3nd2[i] - b2_3nd3 * B_last_last_3nd2[i] - b3_3nd3 * B_last_last_last_3nd2[i];
	}
		ele->AccFil_3nd2[2] = a0_3nd4 * ele->AccRel[2] + a1_3nd4 * A_last_3nd2[2] + a2_3nd4 * A_last_last_3nd2[2] + a3_3nd4 * A_last_last_last_3nd2[2] - b1_3nd4 * B_last_3nd2[2] - b2_3nd4 * B_last_last_3nd2[2] - b3_3nd4 * B_last_last_last_3nd2[2];

	for(int i=0;i<3;i++)
	{
		A_last_last_last_3nd2[i] = A_last_last_3nd2[i];
		A_last_last_3nd2[i] = A_last_3nd2[i];
		A_last_3nd2[i] = ele->AccRel[i];
		
		B_last_last_last_3nd2[i] = B_last_last_3nd2[i];
		B_last_last_3nd2[i] = B_last_3nd2[i];
		B_last_3nd2[i] = ele->AccFil_3nd2[i];
	}
}
bool Icm20602_Calc(sICM20602 *ele)
{
	if(ele->Sta == STA_INI)return false;
	Tim_Calc(&ele->Tim); 
	//获取寄存器数据
	Icm20602_Read(&ICM20602[0]);
	Icm20602_Cali(&ICM20602[0]);
	//获取温度和温飘值
	ele->TmpRel[0]=(float)ele->TmpRaw[0]/326.8f+25.0f;
	ele->AccRot[0] =  -ele->Dir[0]*(ele->AccRaw[ele->Dir[1]]);
	ele->AccRot[1] =  -ele->Dir[2]*(ele->AccRaw[ele->Dir[3]]);
	ele->AccRot[2] =  -ele->Dir[4]*(ele->AccRaw[ele->Dir[5]]-ele->AccOff[ele->Dir[5]]);
	ele->GyrRot[0] =  ele->Dir[0]*(ele->GyrRaw[ele->Dir[1]]);
	ele->GyrRot[1] =  ele->Dir[2]*(ele->GyrRaw[ele->Dir[3]]);
	ele->GyrRot[2] =  ele->Dir[4]*(ele->GyrRaw[ele->Dir[5]]);

	ele->AccRel[0] = ele->AccRot[0]*ICM_ACC_FCT;
	ele->AccRel[1] = ele->AccRot[1]*ICM_ACC_FCT;
	ele->AccRel[2] = ele->AccRot[2]*ICM_ACC_FCT;

	ele->GyrRel[0] = ele->GyrRot[0]*ICM_GYR_FCT;
	ele->GyrRel[1] = ele->GyrRot[1]*ICM_GYR_FCT;
	ele->GyrRel[2] = ele->GyrRot[2]*ICM_GYR_FCT;
	
  ele->AccRel[0] = 0.989733f * (ele->AccRel[0] + 0.083872f);
	ele->AccRel[1] = 0.988220f * (ele->AccRel[1] - 0.033627f);
	ele->AccRel[2] = 1.000008f * (ele->AccRel[2] - 0.195307f);
	
	ele->GyrRel[0] = ele->GyrRel[0] + 0.0181f;
	ele->GyrRel[1] = ele->GyrRel[1] + 0.0271f;
	ele->GyrRel[2] = ele->GyrRel[2] - 0.0099f;

	//滤波
	SlideFilt(ele->AccFil,ele->AccRel,3,&ele->Filt,1);
	SlideFilt(ele->GyrFil,ele->GyrRel,3,&ele->Filt,2);
	SlideFilt(ele->TmpFil,ele->TmpRel,1,&ele->Filt,2);
	
	Butterworth_2nd(ele);
	Butterworth_3nd2(ele);
	Butterworth_3nd(ele);

	
//	ele->GyrFil_2nd[0] = ele->GyrFil_2nd[0] + 0.0181f;
//	ele->GyrFil_2nd[1] = ele->GyrFil_2nd[1] + 0.0271f;
//	ele->GyrFil_2nd[2] = ele->GyrFil_2nd[2] - 0.0099f;
	
	/**************加速度计校正****************/
//	ele->AccFil_3nd[0] = 0.989733f * (ele->AccFil_3nd[0] + 0.083872f);
//	ele->AccFil_3nd[1] = 0.988220f * (ele->AccFil_3nd[1] - 0.033627f);
//	ele->AccFil_3nd[2] = 1.000008f * (ele->AccFil_3nd[2] - 0.195307f);
//	
//	ele->AccFil_3nd2[0] = 0.989733f * (ele->AccFil_3nd2[0] + 0.083872f);
//	ele->AccFil_3nd2[1] = 0.988220f * (ele->AccFil_3nd2[1] - 0.033627f);
//	ele->AccFil_3nd2[2] = 1.000008f * (ele->AccFil_3nd2[2] - 0.195307f);
//	
//  ele->AccRel[0] = 0.989733f * (ele->AccRel[0] + 0.083872f);
//	ele->AccRel[1] = 0.988220f * (ele->AccRel[1] - 0.033627f);
//	ele->AccRel[2] = 1.000008f * (ele->AccRel[2] - 0.195307f);
//--------------------------------------------------------------------------


	ele->Update=true;
	ele->Update_time = ele->Tim.CNT;
	/****计算采集时间****/
	float dt_sec;
	ele->imuSampleTime_ms = 0.001f * ele->Update_time;
	if(ele->imuPreSampleTime_ms == 0) ele->imuPreSampleTime_ms = ele->imuSampleTime_ms;
	dt_sec = 0.001f * (ele->imuSampleTime_ms - ele->imuPreSampleTime_ms);//计算采集间隔时间
	ele->imuPreSampleTime_ms = ele->imuSampleTime_ms;//上次IMU采样时间
	/****计算采集时间****/
//	


	/****速度、角速度叠加****/
	static float last_gyro[3] = {0.0f,0.0f,0.0f};
	static float last_acc[3] = {0.0f,0.0f,0.0f};
	static float last_delAng[3] = {0.0f,0.0f,0.0f};//上一时刻欧拉角增量	
	float delAng_Coning[3] = {0.0f,0.0f,0.0f};//圆锥补偿	
	for(u8 i= 0;i<3;i++){
		ele->delAng[i] = 0.5f * (last_gyro[i] + ele->GyrFil_2nd[i]) * dt_sec;
		delAng_Coning[i] = (ele->delAng_acc[i] + 1.0f/6.0f * last_delAng[i]);//计算圆锥补偿增量，原：delAng_Coning[i] = 1.0f/6.0f*(ele->delAng_acc[i] + last_delAng[i]);
		ele->delVel[i] = -0.5f * (last_acc[i] + ele->AccFil_3nd[i]) * dt_sec;
	}
	crossMulti3X3(delAng_Coning,ele->delAng);//叉乘
	for(u8 i = 0;i<3;i++){
		delAng_Coning[i] = 0.5f*delAng_Coning[i];
	}
	
	for(u8 i = 0;i<3;i++){
		ele->delAng_acc[i] += delAng_Coning[i] + ele->delAng[i];//叠加
		ele->delVel_acc[i] += ele->delVel[i];
	}
	ele->delAng_acc_dt += dt_sec;
	ele->delVel_acc_dt += dt_sec;
	
	for(u8 i=0;i<3;i++){
		last_delAng[i] = ele->delAng[i];
		last_gyro[i] = ele->GyrFil_2nd[i];
		last_acc[i] = ele->AccFil_3nd[i];
	}
	/****速度、角速度叠加****/
	if(ele->delAng_acc_dt>0.015f){
		for(u8 i = 0;i<3;i++)
		{
			ele->delAng_acc[i] = 0.0f;
			ele->delVel_acc[i] = 0.0f;
		}
		ele->delAng_acc_dt = 0;
		ele->delVel_acc_dt = 0;
	}
	
	Tim_Calc(&ele->Tim);
	ele->Time = ele->Tim.OUT;
//	Dprintf("AccFil:%f,%f,%f ,GyrFil:%f,%f,%f, TmpFil:%f, Tim:%d\r\n",ele->AccFil[0],ele->AccFil[1],ele->AccFil[2],ele->GyrFil[0],ele->GyrFil[1],ele->GyrFil[2],ele->TmpFil[0],ele->Time);
	return true;
}


