#ifndef __L3GD20_H
#define __L3GD20_H
#include "userlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define L3G_WHO_AM_I  0x0FU
#define L3G_CTRL_REG1  0x20U
#define L3G_CTRL_REG2  0x21U
#define L3G_CTRL_REG3  0x22U
#define L3G_CTRL_REG4  0x23U
#define L3G_CTRL_REG5  0x24U
#define L3G_REFERENCE  0x25U
#define L3G_OUT_TEMP   0x26U
#define L3G_STATUS_REG 0x27U
#define L3G_OUT_X_L    0x28U
#define L3G_OUT_X_H    0x29U
#define L3G_OUT_Y_L    0x2AU
#define L3G_OUT_Y_H    0x2BU
#define L3G_OUT_Z_L    0x2CU
#define L3G_OUT_Z_H    0x2DU
#define L3G_FIFO_CTRL  0x2EU
#define L3G_FIFO_SRC   0x2FU
#define L3G_INT1_CFG   0x30U
#define L3G_INT1_SRC   0x31U
#define L3G_INT1_TSH_XH  0x32U
#define L3G_INT1_TSH_XL  0x33U
#define L3G_INT1_TSH_YH  0x34U
#define L3G_INT1_TSH_YL  0x35U
#define L3G_INT1_TSH_ZH  0x36U
#define L3G_INT1_TSH_ZL  0x37U
#define L3G_INT1_DURATION  0x38U

#define L3GD20_ADDR 0xD4U
#define L3G_GYR_FCT 0.0175f     //精度

//实际值：温度单位为摄氏度，陀螺仪为°/s,加速度计为m/s^2，电压为V，高度为m。
typedef struct
{
    SPI_HandleTypeDef *hspi;
	char *name;
    s16 FiltNum;
    char *Dir;       
    float GyrOffO[3]; //25°
	float GyrOffT[3]; 
}sL3G_CFG;

typedef struct
{
	SPI_HandleTypeDef *hspi;  //端口  --
	char *name;       
    sTIM  Tim;        //计时器
    sCNT  Filt;       //均值滤波器  --
    s8    Dir[6];     //方向  --
	s16   GyrOff[3];  //漂移量  --
    float GyrOffO[3]; //零飘  --
	float GyrOffT[3]; //温飘  --
    s16   GyrRaw[3];  //原始值
    s16   TmpRaw[1];  //温度原始值
    s16   GyrRot[3];  //旋转值
    float GyrRel[3];  //实际值
    float TmpRel[1];     //温度实际值
	//用户访问数据
	bool  Update;     //更新  --
    eSTA  Sta;        //状态  --
    eERR  Err;        //错误信息  --
	u32   Time;       //耗时
	bool  GyrCal;     //校准  --
    float GyrFil[3];  //滤波值
    float TmpFil[1];  //滤波值
}sL3G;

#define L3G_NUM 1
extern sL3G l3g[L3G_NUM];

void L3g_Init_Para(sL3G *ele,sL3G_CFG *elecfg);
bool L3g_Init(sL3G *ele);
bool L3g_Read(sL3G *ele);
bool L3g_Calc(sL3G *ele);


#ifdef __cplusplus
}
#endif
	
#endif
