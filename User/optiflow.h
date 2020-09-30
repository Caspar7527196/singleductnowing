#ifndef __OPTIFLOW_H
#define __OPTIFLOW_H
#include "userlib.h"
#include "arm_math.h"

#define FLOW_RX_LEN		50	//20
//#define __USE_IMU_DATA
//#define __EN_COORDINATE_TRANS
#define __FLOW_DATA_CTRL	//控制器是否使用光流数据

//量化比例（重力加速度G取9.8m/s^2 ）
#define OF_IMU_ACC_FCT	0.00239258f	//加速度计量程：±8G，FCT = (16/65536)*G (m/s^2)
#define OF_IMU_GYR_FCT	0.00106526f	//陀螺仪量程：±2000°/s，FCT = 4000/65536*pi/180 (rad/s)
#define OF_IMU_GYR_FCT2	0.06103516f	//陀螺仪量程：±2000°/s，FCT = 4000/65536 (°/s)

typedef struct 
{
	UART_HandleTypeDef *huart;
	char *name;
	char *Dir;
	s16   CheckNum;
	float dt;
}sFLOW_CFG;

typedef struct
{
	UART_HandleTypeDef *huart;
	HAL_LockTypeDef LockRx;
	char *name;
	s8    Dir[6];		//方向  --
	bool  RxFlag;		//接收标志位
	u8    RxRawDat[FLOW_RX_LEN]; //数据
	u16   RxRawIndex;
	u8    RxDat[FLOW_RX_LEN];
	sTIM  Tim;			//计时器
	sTIM  Tim2;			//计时器（用于积分计算位置的计时）
	sCNT  Chk;			//检测
	float dt;
	
	//用户访问数据
	bool  Update;		//更新  --
	eSTA  Sta;			//状态  --
	eERR  Err;			//错误信息  --
	u32   Time;			//计时
	//光流模块数据（传感器自身坐标系） ------------------------------
	u8    Quality;		//光流信息质量（数值越大，表示质量越好）
	u8    Light;		//光照强度
	s8    DX, DY;		//原始光流信息，对应移动的速度（像素移动速率）
	s16   DX2, DY2;		//融合后的光流信息，对应移动的速度（cm/s）
	s16   DX2Fix,DY2Fix;//光流速度修正值（cm/s）
	u16   Alt, Alt2;	//原始高度信息和融合后高度信息（单位：cm）
	s16   Gyr[3];		//原始陀螺仪数据
	s16   Gyr2[3];		//滤波后陀螺仪数据
	s16   Acc[3];		//原始加速度数据
	s16   Acc2[3];		//滤波后加速度数据
	float AttAng[3];	//欧拉角格式的姿态数据，ROL,PIT,YAW（单位：°）
	float AttQtn[4];	//四元数格式的姿态数据
	float Dist;			//由Alt或Alt2转换单位得来
	//
	u8    OF_Mode;
	u8    ALT_Mode;
	u8    IMU_Mode;
	u8    ATT_Mode;
	bool  OFUpdate;
	bool  AltUpdate;
	bool  ImuUpdate;
	bool  AttUpdate;
	
	//机体坐标系 ----------------------------------------------------
	s16   GyrRot[3];	//旋转值
	s16   AccRot[3];	//旋转值
	float GyrRel[3];	//实际值
	float AccRel[3];	//实际值
	//【1】认为"DX2,DY2,DX2Fix,DY2Fix"是机体坐标系下的（简称B系）
//	float UvwB[3];		//由DX2,DY2转换单位得来（单位：m/s）
//	float UvwBFix[3];	//由DX2Fix,DY2Fix转换单位得来（单位：m/s）
//	float XyzB[3];		//X,Y分别由DX2Fix,DY2Fix积分得来，而Z即Dist（单位：m）
	
	//H系 -----------------------------------------------------------
	//【2】认为"DX2,DY2,DX2Fix,DY2Fix"是H系下的（机体水平导航坐标系，简称H系）
	float UvwH[3];		//由DX2,DY2转换单位得来（单位：m/s）
//	float UvwHFix[3];	//由DX2Fix,DY2Fix转换单位得来（单位：m/s）
	float XyzH[3];		//X,Y分别由DX2Fix,DY2Fix积分得来，而Z即Dist*cos(θ)*cos(Φ)（单位：m）
	
	//NED系 ---------------------------------------------------------
//	float Uvw[3];
//	float Xyz[3];
}sFLOW;

/* 说明：
1.融合前的光流数据"DX,DY"：
  速度计算公式：移动速度 = 光流原始数据 * 3.8 * 0.02 * 相对高度/秒。
2.融合后的光流数据"DX2,DY2"：
  直接为速度数据，单位为cm/s，噪声小，有零偏，适合做速度环控制。
3.光流速度修正值"DX2Fix, DY2Fix"：
  积分修正后的速度数据，单位为cm/s，噪声较大，积分后偏移幅度小，适合用于积分计算。
4.更多信息参考光流模块手册。
*/

#define FLOW_NUM	1
extern sFLOW flow[FLOW_NUM];

void Flow_Init_Para(sFLOW *ele, sFLOW_CFG *elecfg);
bool Flow_Init(sFLOW *ele);
bool Flow_Calc(sFLOW *ele);

#endif
