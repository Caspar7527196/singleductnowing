#ifndef _AK8975_H_
#define	_AK8975_H_

#include "userlib.h"

#define AK8975_ID					0x48

#define AK8975_WIA_REG          0X00 
#define AK8975_INFO_REG         0X01 
#define AK8975_ST1_REG          0X02 
#define AK8975_HXL_REG          0X03 
#define AK8975_HXH_REG          0X04
#define AK8975_HYL_REG          0X05
#define AK8975_HYH_REG          0X06
#define AK8975_HZL_REG          0X07
#define AK8975_HZH_REG          0X08
#define AK8975_ST2_REG          0X09 
#define AK8975_CNTL_REG         0X0A 
#define AK8975_RSV_REG          0X0B
#define AK8975_ASTC_REG         0X0C 
#define AK8975_TS1_REG          0X0D
#define AK8975_TS2_REG          0X0E
#define AK8975_I2CDIS_REG       0X0F 
#define AK8975_ASAX_REG         0X10 
#define AK8975_ASAY_REG         0X11
#define AK8975_ASAZ_REG         0X12

typedef struct
{
	SPI_HandleTypeDef *hspi;
	char *name;
	s16 FiltNum; 
	char *Dir; 
	s16   MagOff[3];
}sAK8975_CFG;

typedef struct
{
	SPI_HandleTypeDef *hspi;  //端口  --
	char *name;       
	sTIM  Tim;        //计时器
	sCNT  Filt;       //均值滤波器  --
	bool  MagCal;     //校准
	s8    Dir[6];     //方向  --
	s16   MagOff[3];
	s16   MagRaw[3];  //原始值
	s16   MagRot[3];  //旋转值
	float MagRel[3];  //实际值
	//用户访问数据
	bool  Update;     //更新  --
	bool	Ready;
	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	u32   Time;       //计时
	float MagFil[3];  //滤波值
}sAK8975;

#define AK8975_NUM 1
extern sAK8975 ak8975[AK8975_NUM];

void Ak8975_Init_Para(sAK8975 *ele,sAK8975_CFG *elecfg);
bool Ak8975_Init(sAK8975 *ele);
bool Ak8975_Read(sAK8975 *ele);
bool Ak8975_Calc(sAK8975 *ele);

#endif

