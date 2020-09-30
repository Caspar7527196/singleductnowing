#include "ak8975.h"

sAK8975 ak8975[AK8975_NUM];


#define AK8975_SPI4_CS_CLR	HAL_GPIO_WritePin(SPI4_NSS_M_GPIO_Port, SPI4_NSS_M_Pin, GPIO_PIN_RESET)
#define AK8975_SPI4_CS_SET	HAL_GPIO_WritePin(SPI4_NSS_M_GPIO_Port, SPI4_NSS_M_Pin, GPIO_PIN_SET)

static void Ak8975_Spi_Tran(SPI_HandleTypeDef *hspi,u8 addr,u8 dat)
{
	AK8975_SPI4_CS_CLR;
	HAL_SPI_Transmit(hspi,&addr,1,10);
	HAL_SPI_Transmit(hspi,&dat,1,10);
	AK8975_SPI4_CS_SET;
}

static u8 Ak8975_Spi_TranRecv(SPI_HandleTypeDef *hspi,u8 addr)
{
	u8 ReadData;
	AK8975_SPI4_CS_CLR;
	HAL_SPI_Transmit(hspi,&addr,1,10);  
	HAL_SPI_Receive(hspi,&ReadData,1,10);
	AK8975_SPI4_CS_SET;
	return ReadData;
}


void Ak8975_Init_Para(sAK8975 *ele,sAK8975_CFG *elecfg)
{
	ele->Update = false;
	ele->Sta = STA_INI;
	ele->Err = ERR_NONE;
	ele->Ready = false;
	
	ele->hspi = elecfg->hspi;
	ele->name = elecfg->name;
	ele->Filt.CNT = 0;
	ele->Filt.CCR = elecfg->FiltNum;
	if(Dir_Trans(ele->Dir,elecfg->Dir)==false)ele->Err = ERR_SOFT;
	for(u8 i = 0;i < 3;i++)
	{
		ele->MagOff[i] = elecfg->MagOff[i];
	}
	
}


bool Ak8975_Init(sAK8975 *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	if(ele->Sta != STA_INI)
	{
		Dprintf("--Par [NO]\r\n");
        return false;
	}
	
	ele->Sta = STA_CHK;
	u8 ID;
	u8 info;
	ID = Ak8975_Spi_TranRecv(ele->hspi,AK8975_WIA_REG|0x80);		//读取传感器的ID号
	ID = Ak8975_Spi_TranRecv(ele->hspi,AK8975_WIA_REG|0x80);		//读取传感器的ID号
	info = Ak8975_Spi_TranRecv(ele->hspi,AK8975_INFO_REG|0x80);		//读取传感器的INFO
	Dprintf("AK8975_ID:%d ,INFO:%d\r\n",ID,info);
	if(ID != AK8975_ID)
	{
		Dprintf("--Chk [NO]\r\n");
    ele->Err=ERR_LOST;
    return false;
	}
	Dprintf("--Chk [OK]\r\n");
	
	Ak8975_Spi_Tran(ele->hspi,AK8975_CNTL_REG,0x01);
	
	ele->Sta = STA_RUN;
	Dprintf("%s Init.[OK]\r\n",ele->name);

	return true;

}


bool Ak8975_Read(sAK8975 *ele)
{
	u8 ak_dat[6];
	u8 ak_addr[6] = {
						AK8975_HXL_REG,AK8975_HXH_REG,AK8975_HYL_REG,
						AK8975_HYH_REG,AK8975_HZL_REG,AK8975_HZH_REG
	};
	ele->Ready = 0x01&(Ak8975_Spi_TranRecv(ele->hspi,AK8975_ST1_REG|0x80));
	
	if(ele->Ready)
	{
		for(u8 i = 0;i < 6;i++)
		{
			ak_dat[i] = Ak8975_Spi_TranRecv(ele->hspi,ak_addr[i]|0x80);
		}
		ele->MagRaw[0] = (ak_dat[1]<<8) + ak_dat[0];
		ele->MagRaw[1] = (ak_dat[3]<<8) + ak_dat[2];
		ele->MagRaw[2] = (ak_dat[5]<<8) + ak_dat[4];	
		
		Ak8975_Spi_Tran(ele->hspi,AK8975_CNTL_REG,0x01);
		
//	Dprintf("MagFil:%d,%d,%d \r\n",ele->MagRaw[0],ele->MagRaw[1],ele->MagRaw[2]);
	}
	else
		return false;
	
	return true;
	
}
bool Ak8975_Calc(sAK8975 *ele)
{
	if(ele->Sta == STA_INI)return false;
	Tim_Calc(&ele->Tim);
	
	if(Ak8975_Read(&ak8975[0]) == false)
		return false;
	
//	ele->MagRot[0] = ele->MagRaw[0] - ele->MagOff[0];
//	ele->MagRot[1] = ele->MagRaw[1] - ele->MagOff[1];
//	ele->MagRot[2] = ele->MagRaw[2] - ele->MagOff[2];
	
	ele->MagRot[0] = ele->Dir[0]*(ele->MagRaw[ele->Dir[1]]-ele->MagOff[ele->Dir[1]]);//校正磁力计时令MagOff=0，然后采集MagRot
	ele->MagRot[1] = ele->Dir[2]*(ele->MagRaw[ele->Dir[3]]-ele->MagOff[ele->Dir[3]]);
	ele->MagRot[2] = ele->Dir[4]*(ele->MagRaw[ele->Dir[5]]-ele->MagOff[ele->Dir[5]]);
	
	
//	ele->MagRot[0] = 0.707*ele->MagRot[0]+0.707*ele->MagRot[1];
//	ele->MagRot[1] = -0.707*ele->MagRot[0]+0.707*ele->MagRot[1];
//	ele->MagRot[2] = ele->MagRot[2];
	
	ele->MagRel[0] = (float)ele->MagRot[0];
	ele->MagRel[1] = (float)ele->MagRot[1];
	ele->MagRel[2] = (float)ele->MagRot[2];
	
	SlideFilt(ele->MagFil,ele->MagRel,3,&ele->Filt,1);
	ele->Update=true;
	Tim_Calc(&ele->Tim);
	ele->Time = ele->Tim.OUT;
//	Dprintf("MagFil:%f,%f,%f , Tim:%d\r\n",ele->MagFil[0],ele->MagFil[1],ele->MagFil[2],ele->Time);
	return true;

}



