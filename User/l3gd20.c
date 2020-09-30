#include "l3gd20.h"

/***************************************************\
功能：
  L3GD20 陀螺仪I2C读取
说明：
  1、修改端口
  2、每次读取完数据需要将Update复位
  3、注意需要修改i2c库的timeout改成较小的值，否则硬件错误时会等待很长一段时间。
  I2C_TIMEOUT_BUSY或I2C_TIMEOUT_BUSY_FLAG
\***************************************************/

sL3G l3g[L3G_NUM];

/******************驱动程序****************/
//驱动程序：spi读写l3gd20底层程序
#define L3GD20_SPI4_CS_CLR	HAL_GPIO_WritePin(SPI4_NSS_G_GPIO_Port, SPI4_NSS_G_Pin, GPIO_PIN_RESET)
#define L3GD20_SPI4_CS_SET	HAL_GPIO_WritePin(SPI4_NSS_G_GPIO_Port, SPI4_NSS_G_Pin, GPIO_PIN_SET)

static void L3gd20_Spi_Tran(SPI_HandleTypeDef *hspi,u8 add,u8 dat)
{
	L3GD20_SPI4_CS_CLR;
	HAL_SPI_Transmit(hspi,&add,1,10);
	HAL_SPI_Transmit(hspi,&dat,1,10);
	L3GD20_SPI4_CS_SET;
}

static u8 L3gd20_Spi_TranRecv(SPI_HandleTypeDef *hspi,u8 TxData)
{
	u8 RxData;
	L3GD20_SPI4_CS_CLR;
	HAL_SPI_Transmit(hspi,&TxData,1,10);
	HAL_SPI_Receive(hspi,&RxData,1,10);
	L3GD20_SPI4_CS_SET;
	return RxData;
}
//驱动程序：spi读写l3gd20底层程序


/******************L3G功能函数****************/
void L3g_Init_Para(sL3G *ele,sL3G_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	ele->GyrCal = false;
	
    ele->hspi = elecfg->hspi;
	ele->name = elecfg->name;
    ele->Filt.CNT = 0;
    ele->Filt.CCR = elecfg->FiltNum;
	ele->Time = 0;
    
    if(Dir_Trans(ele->Dir,elecfg->Dir)==false)ele->Err = ERR_SOFT;
	for(u8 i=0;i<3;i++)
	{
		ele->GyrOffO[i] = elecfg->GyrOffO[i];
		ele->GyrOffT[i] = elecfg->GyrOffT[i];
	}
}

bool L3g_Init(sL3G *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
    //配置
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
    if(ele->Err==ERR_SOFT)
    {
        Dprintf("--Dir [NO]\r\n");
        return false;
    }
    bool I2cSta=true;
		L3gd20_Spi_Tran(ele->hspi,L3G_CTRL_REG1,0x3F);//使能轴输出，输出频率95Hz，带宽25
    L3gd20_Spi_Tran(ele->hspi,L3G_CTRL_REG2,0x00);//高通滤波器选择，截止频率7.2Hz
    L3gd20_Spi_Tran(ele->hspi,L3G_CTRL_REG3,0x00);//中断配置，无需
    L3gd20_Spi_Tran(ele->hspi,L3G_CTRL_REG4,0x10);//持续输出，输出范围+-500dps。
    L3gd20_Spi_Tran(ele->hspi,L3G_CTRL_REG5,0x10);//使能高通滤波器，禁用FIFO
    if(I2cSta==false)
    {
        Dprintf("--Cfg [NO]\r\n");
        ele->Err=ERR_HARD;
        return false;
    }
    Dprintf("--Cfg [OK]\r\n");
    //检测
    u8 ID=0;
    ID = L3gd20_Spi_TranRecv(ele->hspi,L3G_WHO_AM_I|0x80);
    if(ID!=0xD4)
    {
        Dprintf("--Chk [NO]\r\n");
        ele->Err=ERR_LOST;
        return false;
    }
    Dprintf("--Chk [OK]\r\n");
    //初始化完毕
	ele->GyrCal = false;
    ele->Sta = STA_RUN;
    Dprintf("%s Init.[OK]\r\n",ele->name);
    return true;
}
//读取L3G数据，并进行旋转变换
bool L3g_Read(sL3G *ele)
{
	u8 l3g_addr[7]={
		L3G_OUT_X_L,L3G_OUT_X_H,L3G_OUT_Y_L,
		L3G_OUT_Y_H,L3G_OUT_Z_L,L3G_OUT_Z_H,L3G_OUT_TEMP
	};
	
	u8 lsm_dat[7];
	for(int i = 0;i < 7;i++)
	{
		lsm_dat[i] = L3gd20_Spi_TranRecv(ele->hspi,l3g_addr[i]|0x80);
	}

	ele->GyrRaw[0] = (lsm_dat[1]<<8) + lsm_dat[0];
	ele->GyrRaw[1] = (lsm_dat[3]<<8) + lsm_dat[2];
	ele->GyrRaw[2] = (lsm_dat[5]<<8) + lsm_dat[4];
	
	ele->TmpRaw[0] = lsm_dat[6];
	return true;
}

static bool L3g_Cali(sL3G *ele) //Compute MPU6050 Temperature Compensation Bias
{
	static sL3G *ready = NULL;
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
	static float  GyrSum[2][3] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	static float  ImuTmp[2][1] = {0.0f, 0.0f};
	static u16 SmpCnt = 0;
	static u8 OneSmpCnt = 0;
	if(OneSmpCnt++<2)return false;  //2个周期采一次数据
	if(SmpCnt++<GYR_SAMPLE_NUM)     //采集由采集函数进行
	{
		OneSmpCnt=0;
		GyrSum[1][0] += (float)ele->GyrRaw[0];
		GyrSum[1][1] += (float)ele->GyrRaw[1];
		GyrSum[1][2] += (float)ele->GyrRaw[2];
		ImuTmp[1][0] += (float)ele->TmpRaw[0]/340.0f+36.53f;
		return false;
	}
	SmpCnt = 0;
	GyrSum[1][0] /= (float)GYR_SAMPLE_NUM;
	GyrSum[1][1] /= (float)GYR_SAMPLE_NUM;
	GyrSum[1][2] /= (float)GYR_SAMPLE_NUM;
	ImuTmp[1][0] /= (float)GYR_SAMPLE_NUM;
	if(ImuTmp[0][0]!=0.0f && ImuTmp[1][0]-ImuTmp[0][0]>2.0f && ImuTmp[1][0]-ImuTmp[0][0]<-2.0f)//相差2度以上
	{
		ele->GyrOffT[0] = (GyrSum[1][0]-GyrSum[0][0])/(ImuTmp[1][0]-ImuTmp[0][0]);
		ele->GyrOffT[1] = (GyrSum[1][1]-GyrSum[0][1])/(ImuTmp[1][0]-ImuTmp[0][0]);
		ele->GyrOffT[2] = (GyrSum[1][2]-GyrSum[0][2])/(ImuTmp[1][0]-ImuTmp[0][0]);
		ele->GyrOffO[0] = GyrSum[1][0]-(ImuTmp[1][0]-25.0f)*ele->GyrOffT[0];
		ele->GyrOffO[1] = GyrSum[1][1]-(ImuTmp[1][0]-25.0f)*ele->GyrOffT[1];
		ele->GyrOffO[2] = GyrSum[1][2]-(ImuTmp[1][0]-25.0f)*ele->GyrOffT[2];
	}
	else
	{
		ele->GyrOffT[0] = 0.0f;
		ele->GyrOffT[1] = 0.0f;
		ele->GyrOffT[2] = 0.0f;
		ele->GyrOffO[0] = GyrSum[1][0];
		ele->GyrOffO[1] = GyrSum[1][1];
		ele->GyrOffO[2] = GyrSum[1][2];
	}
	GyrSum[0][0] = GyrSum[1][0];
	GyrSum[0][1] = GyrSum[1][1];
	GyrSum[0][2] = GyrSum[1][2];
	ImuTmp[0][0] = ImuTmp[1][0];
	GyrSum[1][0] = 0.0f;
	GyrSum[1][1] = 0.0f;
	GyrSum[1][2] = 0.0f;
	ImuTmp[1][0] = 0.0f;
	Dprintf("\r\n%s Cali--Temp is:%.2f,Zero Bias is:%.2f %.2f %.2f,Tmp Bias is:%.2f %.2f %.2f\r\n",ele->name,
		ImuTmp[0][0],ele->GyrOffO[0],ele->GyrOffO[1],ele->GyrOffO[2],ele->GyrOffT[0],ele->GyrOffT[1],ele->GyrOffT[2]);
	ele->Sta = STA_RUN;
	GyrIsCali = false;
	ready = NULL;
	return true;
}

//读取数据并去除温飘，以旋转后的数据为基准
bool L3g_Calc(sL3G *ele)
{
	if(ele->Sta == STA_INI)return false;
	Tim_Calc(&ele->Tim);   //计时  
    if(L3g_Read(ele)==false)return false;
	L3g_Cali(ele);
    //获取温度和温漂值
    ele->TmpRel[0]=(float)ele->TmpRaw[0];
	ele->GyrOff[0] = (ele->TmpRel[0]-25.0f)*ele->GyrOffT[0]+ele->GyrOffO[0];
	ele->GyrOff[1] = (ele->TmpRel[0]-25.0f)*ele->GyrOffT[1]+ele->GyrOffO[1];
	ele->GyrOff[2] = (ele->TmpRel[0]-25.0f)*ele->GyrOffT[2]+ele->GyrOffO[2];
    //方向变换
    ele->GyrRot[0] =  ele->Dir[0]*(ele->GyrRaw[ele->Dir[1]]-ele->GyrOff[ele->Dir[1]]);
    ele->GyrRot[1] =  ele->Dir[2]*(ele->GyrRaw[ele->Dir[3]]-ele->GyrOff[ele->Dir[3]]);
    ele->GyrRot[2] =  ele->Dir[4]*(ele->GyrRaw[ele->Dir[5]]-ele->GyrOff[ele->Dir[5]]);
    //实际值转换
    ele->GyrRel[0] = ele->GyrRot[0]*L3G_GYR_FCT;
    ele->GyrRel[1] = ele->GyrRot[1]*L3G_GYR_FCT;
    ele->GyrRel[2] = ele->GyrRot[2]*L3G_GYR_FCT;
    //滤波处理
    SlideFilt(ele->GyrFil,ele->GyrRel,3,&ele->Filt,1);
    SlideFilt(ele->TmpFil,ele->TmpRel,1,&ele->Filt,2);
    ele->Update=true;
	Tim_Calc(&ele->Tim);
	ele->Time = ele->Tim.OUT;
//	Dprintf("GyrFil:%f,%f,%f, TmpFil:%f, Tim:%d\r\n",ele->GyrFil[0],ele->GyrFil[1],ele->GyrFil[2],ele->TmpFil[0],ele->Time);
    return true;
}
/******************************END OF FILE************************************/

