#ifndef __LSM303D_H
#define __LSM303D_H
#include "userlib.h"

#ifdef __cplusplus
extern "C" {
#endif

//Register Address Map
#define LSM_TEMP_OUT_L		0x05U
#define LSM_TEMP_OUT_H		0x06U
#define LSM_STATUS_M		0x07U
#define LSM_OUT_X_L_M		0x08U
#define LSM_OUT_X_H_M		0x09U
#define LSM_OUT_Y_L_M		0x0AU
#define LSM_OUT_Y_H_M		0x0BU
#define LSM_OUT_Z_L_M		0x0CU
#define LSM_OUT_Z_H_M		0x0DU
#define LSM_WHO_AM_I		0x0FU	//0x49
#define LSM_INT_CTRL_M		0x12U	//0xE8  磁力计中断配置，不启用0x00，感觉默认值应该是0x00
#define LSM_INT_SRC_M		0x13U	//阈值触发配置
#define LSM_INT_THS_L_M		0x14U	//阈值配置
#define LSM_INT_THS_H_M		0x15U
#define LSM_OFFSET_X_L_M	0x16U	//漂移量设置
#define LSM_OFFSET_X_H_M	0x17U
#define LSM_OFFSET_Y_L_M	0x18U
#define LSM_OFFSET_Y_H_M	0x19U
#define LSM_OFFSET_Z_L_M	0x1AU
#define LSM_OFFSET_Z_H_M	0x1BU
#define LSM_REFERENCE_X		0x1CU	//与加速度数据进行高通滤波的参考值
#define LSM_REFERENCE_Y		0x1DU
#define LSM_REFERENCE_Z		0x1EU
#define LSM_CTRL0			0x1FU	//0x00  高通滤波器启用与FIFO启用 0x00
#define LSM_CTRL1			0x20U	//0x07  加速度计频率 加速度各轴使能与更新方式 0x37启用所有，频率为12.5Hz
#define LSM_CTRL2			0x21U	//0x00  加速度计带宽与范围以及自检 0xC8不自检 0xCA自检   50Hz带宽，+-4g量程
#define LSM_CTRL3			0x22U	//0x00  中断配置 0x00
#define LSM_CTRL4			0x23U	//0x00  中断配置 0x00
#define LSM_CTRL5			0x24U	//0x18  温度检测使能 磁力计精度与频率，0xE8 启用温度检测，频率为12.5Hz
#define LSM_CTRL6			0x25U	//0x20  磁力计范围 0x20默认4gauss
#define LSM_CTRL7			0x26U	//0x01  特殊模式 磁力计连续转换 0x80
#define LSM_STATUS_A		0x27U
#define LSM_OUT_X_L_A		0x28U
#define LSM_OUT_X_H_A		0x29U
#define LSM_OUT_Y_L_A		0x2AU
#define LSM_OUT_Y_H_A		0x2BU
#define LSM_OUT_Z_L_A		0x2CU
#define LSM_OUT_Z_H_A		0x2DU
#define LSM_FIFO_CTRL		0x2EU	//FIFO配置
#define LSM_FIFO_SRC		0x2FU	//FIFO状态
#define LSM_IG_CFG1			0x30U	//IG均为中断相关
#define LSM_IG_SRC1			0x31U
#define LSM_IG_THS1			0x32U
#define LSM_IG_DUR1			0x33U
#define LSM_IG_CFG2			0x34U
#define LSM_IG_SRC2			0x35U
#define LSM_IG_THS2			0x36U
#define LSM_IG_DUR2			0x37U
#define LSM_CLICK_CFG		0x38U	//中断相关
#define LSM_CLICK_SRC		0x39U 
#define LSM_CLICK_THS		0x3AU
#define LSM_TIME_LIMIT		0x3BU
#define LSM_TIME_LATENCY	0x3CU
#define LSM_TIME_WINDOW		0x3DU
#define LSM_Act_THS			0x3EU
#define LSM_Act_DUR			0x3FU

#define LSM303D_ADDR		0x3C

#define LSM_ACC_FCT			0.00119708f		//单位换算为：m/s^2，重力是9.8065
#define LSM_MAG_FCT			0.160f			//单位换算为：mgauss，磁场是500~600mguass

typedef struct
{
	SPI_HandleTypeDef *hspi;
	char  *name;
	char  *Dir;
	s16   FiltNum;
	float OneG;
	s16   AccOff[3];
	s16   MagOff[3];
	float *MagMat;
}sLSM_CFG;

//实际值：温度单位为摄氏度，陀螺仪为°/s,加速度计为m/s^2，电压为V，高度为m。
typedef struct
{
	SPI_HandleTypeDef *hspi; //端口  --
	char *name;       
	sTIM  Tim;			//计时器
	sCNT  Filt;			//均值滤波器  --
	s8    Dir[6];		//方向  --
	u8    AccCalFlag;
	s16   AccOff[3];	//漂移量  --
	s16   MagOff[3];	//偏移量  --
	float MagMat[9];	//校准矩阵，用于把椭球修正为球
	s16   AccRaw[3];	//原始值
	s16   MagRaw[3];	//原始值
	s16   TmpRaw[1];	//原始值
	s16   AccRot[3];	//旋转值
	s16   MagRot[3];	//旋转值
	float AccRel[3];	//实际值
	float MagRel[3];	//实际值
	float TmpRel[1];	//实际值
	//用户访问数据
	bool  Update;		//更新  --
	eSTA  Sta;			//状态  --
	eERR  Err;			//错误信息  --
	u32   Time;			//耗时
	bool  MagCal;		//校准  --
	bool  AccCal;		//校准  --
	float OneG;			//重力  --
	float AccFil[3];	//滤波值
	float MagFil[3];	//滤波值
	float TmpFil[1];	//滤波值
}sLSM;

#define LSM_NUM			1

extern sLSM lsm[LSM_NUM];

void Lsm_Init_Para(sLSM *ele, sLSM_CFG *elecfg);
bool Lsm_Init(sLSM *ele);
bool Lsm_Read(sLSM *ele);
bool Lsm_Calc(sLSM *ele);

#ifdef __cplusplus
}
#endif

#endif 
/******************************END OF FILE************************************/
