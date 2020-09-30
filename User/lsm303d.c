#include "lsm303d.h"

/***************************************************\
功能：
  lsm303d 磁力计与加速度计的I2C读取
说明：
  1、修改端口
  2、每次读取完数据需要将Update复位
  3、注意需要修改i2c库的timeout改成较小的值，否则硬件错误时会等待很长一段时间。
\***************************************************/

sLSM lsm[LSM_NUM];

//驱动程序：spi读写lsm303d底层程序
#define LSM303D_SPI4_CS_CLR	HAL_GPIO_WritePin(SPI4_NSS_AM_GPIO_Port, SPI4_NSS_AM_Pin, GPIO_PIN_RESET)
#define LSM303D_SPI4_CS_SET	HAL_GPIO_WritePin(SPI4_NSS_AM_GPIO_Port, SPI4_NSS_AM_Pin, GPIO_PIN_SET)

static void Lsm303d_Spi_Tran(SPI_HandleTypeDef *hspi,u8 add,u8 dat)
{
	LSM303D_SPI4_CS_CLR;
	HAL_SPI_Transmit(hspi,&add,1,10);
	HAL_SPI_Transmit(hspi,&dat,1,10);
	LSM303D_SPI4_CS_SET;
}

static u8 Lsm303d_Spi_TranRecv(SPI_HandleTypeDef *hspi,u8 TxData)
{
	u8 RxData;
	LSM303D_SPI4_CS_CLR;
	HAL_SPI_Transmit(hspi,&TxData,1,10);
	HAL_SPI_Receive(hspi,&RxData,1,10);
	LSM303D_SPI4_CS_SET;
	return RxData;
}
//驱动程序：spi读写lsm303d底层程序
void Lsm_Init_Para(sLSM *ele, sLSM_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	ele->AccCal = false;
	ele->MagCal = false;
	ele->AccCalFlag = 0;
	
	ele->hspi = elecfg->hspi;
	ele->name = elecfg->name;
	ele->Filt.CNT = 0;
	ele->Filt.CCR = elecfg->FiltNum;
	ele->Time = 0;
	
	if(Dir_Trans(ele->Dir, elecfg->Dir) == false)
		ele->Err = ERR_SOFT;
	
	ele->OneG = elecfg->OneG;
	for(u8 i=0; i<3; i++)
	{
		ele->AccOff[i] = elecfg->AccOff[i];
		ele->MagOff[i] = elecfg->MagOff[i];
	}
	arm_copy_f32(elecfg->MagMat, ele->MagMat, 9);
}

bool Lsm_Init(sLSM *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	//配置
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
	bool I2cSta = true;
	Lsm303d_Spi_Tran(ele->hspi, LSM_CTRL1, 0x67); //持续模式，100HZ
	Lsm303d_Spi_Tran(ele->hspi, LSM_CTRL2, 0xC8); //50Hz带宽，+-4g
	Lsm303d_Spi_Tran(ele->hspi, LSM_CTRL5, 0xF0); //50Hz带宽
	Lsm303d_Spi_Tran(ele->hspi, LSM_CTRL6, 0x00); //+-2gauss
	Lsm303d_Spi_Tran(ele->hspi, LSM_CTRL7, 0x80);
	if(I2cSta == false)
	{
		Dprintf("--Cfg [NO]\r\n");
		ele->Err = ERR_HARD;
	return false;
	}
	Dprintf("--Cfg [OK]\r\n");
	//检测
	u8 ID = 0;
	ID = Lsm303d_Spi_TranRecv(ele->hspi, LSM_WHO_AM_I|0x80);
	if(ID != 0x49)
	{
		Dprintf("--Chk [NO]\r\n");
		ele->Err = ERR_LOST;
		return false;
	}
	Dprintf("--Chk [OK]\r\n");
	//初始化完毕
	ele->MagCal = false;
	ele->AccCal = false;
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n", ele->name);
	return true;
}

bool Lsm_Read(sLSM *ele)
{
	u8 lsm_addr[14]={
		LSM_OUT_X_L_M,LSM_OUT_X_H_M,LSM_OUT_Y_L_M,LSM_OUT_Y_H_M,
		LSM_OUT_Z_L_M,LSM_OUT_Z_H_M,LSM_TEMP_OUT_L,LSM_TEMP_OUT_H,
		LSM_OUT_X_L_A,LSM_OUT_X_H_A,LSM_OUT_Y_L_A,LSM_OUT_Y_H_A,
		LSM_OUT_Z_L_A,LSM_OUT_Z_H_A
	};
	
	u8 lsm_dat[14];
	for(int i = 0;i < 14;i++)
	{
		lsm_dat[i] = Lsm303d_Spi_TranRecv(ele->hspi,lsm_addr[i]|0x80);
	}
	ele->MagRaw[0] = (lsm_dat[1]<<8) + lsm_dat[0];
	ele->MagRaw[1] = (lsm_dat[3]<<8) + lsm_dat[2];
	ele->MagRaw[2] = (lsm_dat[5]<<8) + lsm_dat[4];
	ele->TmpRaw[0] = (lsm_dat[7]<<8) + lsm_dat[6];
	ele->AccRaw[0] = (lsm_dat[9]<<8) + lsm_dat[8];
	ele->AccRaw[1] = (lsm_dat[11]<<8) + lsm_dat[10];
	ele->AccRaw[2] = (lsm_dat[13]<<8) + lsm_dat[12];
	return true;
}

static bool Lsm_MagCali(sLSM *ele)  //八字立体校准
{
	static sLSM *ready = NULL;
	if(ready == NULL) ready = ele;
	else if(ready != ele) return false;
	
	static bool MagIsCali = false;
	if(ele->MagCal == true)
	{
		ele->MagCal = false;
		ele->Sta = STA_CAL;
		MagIsCali = true;
		Dprintf("\r\n%s Mag Cali\r\n", ele->name);
	}
	if(MagIsCali == false) return true;
	#define SAMPLE_NUM 600  //30s
	static float d[SAMPLE_NUM][3];
	static u16 SmpCnt = 0;
	static u8 OneSmpCnt = 0;
	if(OneSmpCnt++ < 10) return false;  //10个周期采一次数据
	if(SmpCnt < SAMPLE_NUM)
	{
		OneSmpCnt = 0;
		d[SmpCnt][0] = (float)ele->MagRaw[0];
		d[SmpCnt][1] = (float)ele->MagRaw[1];
		d[SmpCnt][2] = (float)ele->MagRaw[2];
		SmpCnt++;
		return false;
	}
	SmpCnt = 0;
	u16 Pop[2][3];
	float Origin[3], Radius;
	sphereFit(d, SAMPLE_NUM, 100, 0.0f, Pop, Origin, &Radius);
	ele->MagOff[0] = Origin[0];
	ele->MagOff[1] = Origin[1];
	ele->MagOff[2] = Origin[2];
	Dprintf("\r\n%s Cali--Bias is:%4.2f %4.2f %4.2f, Radius is:%4.2f\r\n", 
			ele->name, Origin[0], Origin[1], Origin[2], Radius);
	ele->Sta = STA_RUN;
	MagIsCali = false;
	ready = NULL;
	return true;
}

static bool Lsm_AccCali(sLSM *ele)
{
	static sLSM *ready = NULL;
	if(ready == NULL) ready=ele;
	else if(ready != ele) return false;
	
	static u8 Index = 0;
	static bool AccIsCali = false;
	if(ele->AccCal == true)
	{
		ele->AccCal = false;
		ele->Sta = STA_CAL;
		AccIsCali = true;
		Index = ele->AccCalFlag;
		Dprintf("\r\n%s Acc Cali:%d\r\n", ele->name, Index);
	}
	if(AccIsCali != true) return true;
	#define ACC_SAMPLE_NUM 50  //0.5s
	static float  AccSum[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	static u16 SmpCnt = 0;
	static u8 OneSmpCnt = 0;
	if(++OneSmpCnt < 2) return false;	//2个周期采一次数据
	if(++SmpCnt < ACC_SAMPLE_NUM)		//采集由采集函数进行
	{
		OneSmpCnt = 0;
		switch(Index)  //6面校准法
		{
			case 0:
				for(u8 i=0; i<6; i++)
					AccSum[i] = 0.0f;
				AccIsCali = false;
				SmpCnt = 0;
				break;
			case 1:
				AccSum[0] += -ele->Dir[4]*(ele->AccRaw[ele->Dir[5]]);  //+Z
				break;
			case 2:
				AccSum[1] +=  ele->Dir[0]*(ele->AccRaw[ele->Dir[1]]);  //-X
				break;
			case 3:
				AccSum[2] += -ele->Dir[2]*(ele->AccRaw[ele->Dir[3]]);  //+Y
				break;
			case 4:
				AccSum[3] +=  ele->Dir[2]*(ele->AccRaw[ele->Dir[3]]);  //-Y
				break;
			case 5:
				AccSum[4] += -ele->Dir[0]*(ele->AccRaw[ele->Dir[1]]);  //+X
				break;
			case 6:
				AccSum[5] +=  ele->Dir[4]*(ele->AccRaw[ele->Dir[5]]);  //-Z
				break;
		}
		return false;
	}
	SmpCnt = 0;
	AccSum[Index-1] /= (float)ACC_SAMPLE_NUM;
	Dprintf("%s Acc Cali:%d--G:%.2f\r\n", ele->name, Index, AccSum[Index-1]);  //正常值8250
	if(Index == 6)
	{
		ele->AccOff[0] = -ele->Dir[0]*(AccSum[4]-AccSum[1])/2.0f;
		ele->AccOff[1] = -ele->Dir[2]*(AccSum[2]-AccSum[3])/2.0f;
		ele->AccOff[2] = -ele->Dir[4]*(AccSum[0]-AccSum[5])/2.0f;
		ele->OneG = (AccSum[0]+AccSum[1]+AccSum[2]+AccSum[3]+AccSum[4]+AccSum[5])/6.0f*LSM_ACC_FCT;
		Dprintf("\r\n%s Cali--Bias is:%d %d %d,OneG is:%.2f\r\n",
				ele->name, ele->AccOff[0], ele->AccOff[1], ele->AccOff[2], ele->OneG);
	}
	ele->Sta = STA_RUN;
	AccIsCali = false;
	ready = NULL;
	return true;
}

bool Lsm_Calc(sLSM *ele)
{
	if(ele->Sta == STA_INI) return false;
	Tim_Calc(&ele->Tim);  //计时  
	if(Lsm_Read(ele) == false) return false;
	Lsm_MagCali(ele);
	Lsm_AccCali(ele);
	//获取温度和温漂值
	ele->TmpRel[0] = (float)ele->TmpRaw[0]/340.0f + 36.53f;
	//方向变换
	ele->AccRot[0] = -ele->Dir[0]*(ele->AccRaw[ele->Dir[1]] - ele->AccOff[ele->Dir[1]]);
	ele->AccRot[1] = -ele->Dir[2]*(ele->AccRaw[ele->Dir[3]] - ele->AccOff[ele->Dir[3]]);
	ele->AccRot[2] = -ele->Dir[4]*(ele->AccRaw[ele->Dir[5]] - ele->AccOff[ele->Dir[5]]);
	//实际值转换
	ele->AccRel[0] = ele->AccRot[0]*LSM_ACC_FCT;
	ele->AccRel[1] = ele->AccRot[1]*LSM_ACC_FCT;
	ele->AccRel[2] = ele->AccRot[2]*LSM_ACC_FCT;
	
	//【2017-11-2】磁力计校准程序
	float MagTmp[3] = {0.0f};
	//数据预处理：球心移至原点，再缩小10000倍
	MagTmp[0] = (ele->MagRaw[0] - ele->MagOff[0])*0.0001f;
	MagTmp[1] = (ele->MagRaw[1] - ele->MagOff[1])*0.0001f;
	MagTmp[2] = (ele->MagRaw[2] - ele->MagOff[2])*0.0001f;
	//把椭球面沿着三个主轴方向进行放缩，化为球面（半径约为1）
	arm_matrix_instance_f32 m_MagMat, m_MagTmp;
	arm_mat_init_f32(&m_MagMat, 3, 3, ele->MagMat);
	arm_mat_init_f32(&m_MagTmp, 3, 1, MagTmp);
	arm_mat_mult_f32(&m_MagMat, &m_MagTmp, &m_MagTmp);
	//实际值转换（此值仅作参考）
	arm_scale_f32(MagTmp, 550.0f, MagTmp, 3);
	//方向变换
	ele->MagRel[0] =  ele->Dir[0]*MagTmp[ele->Dir[1]];
	ele->MagRel[1] =  ele->Dir[2]*MagTmp[ele->Dir[3]];
	ele->MagRel[2] =  ele->Dir[4]*MagTmp[ele->Dir[5]];
	//逆向求取旋转值（此值也仅作参考）
	ele->MagRot[0] = ele->MagRel[0] / LSM_MAG_FCT;
	ele->MagRot[1] = ele->MagRel[1] / LSM_MAG_FCT;
	ele->MagRot[2] = ele->MagRel[2] / LSM_MAG_FCT;
	
	//滤波处理
	SlideFilt(ele->AccFil, ele->AccRel, 3, &ele->Filt, 1);
	SlideFilt(ele->MagFil, ele->MagRel, 3, &ele->Filt, 2);
	SlideFilt(ele->TmpFil, ele->TmpRel, 1, &ele->Filt, 2);
	ele->Update = true;
	Tim_Calc(&ele->Tim);
	ele->Time = ele->Tim.OUT;
//	Dprintf("AccFil:%f,%f,%f ,MagFil:%f,%f,%f, TmpFil:%f, Tim:%d\r\n",ele->AccFil[0],ele->AccFil[1],ele->AccFil[2],ele->MagFil[0],ele->MagFil[1],ele->MagFil[2],ele->TmpFil[0],ele->Time);
	return true;
}
/******************************END OF FILE************************************/
