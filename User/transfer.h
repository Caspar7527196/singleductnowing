#ifndef __TRANSFER_H
#define __TRANSFER_H

#include "userlib.h"
//#include "command.h"
#include "icm20602.h"
#include "ak8975.h"
#include "gps.h"
#include "ms5611.h"
#include "ahrs.h"
#include "pid.h"
#include "rc.h"
#include "motor.h"
#include "exit.h"
#include "bell.h"
#include "optiflow.h"
//#include "EKF.h"
//#include "mpu6050.h"
#include "gps_ins_EKF.h"


#ifdef __cplusplus
 extern "C" {
#endif

#define TRAN_RX_LEN		60
#define TRAN_TX_LEN		60

typedef struct 
{
	bool send_version;
	bool send_status;
	bool send_senser;
	bool send_senser2;
	bool send_gps;
	bool send_pid1;
	bool send_pid2;
	bool send_pid3;
	bool send_pid4;
	bool send_pid5;
	bool send_pid6;
	bool send_rcdata;
	bool send_offset;
	bool send_motopwm;
	bool send_power;
	bool send_check;
	bool send_user;
}sSND_FLAG;

typedef struct
{
	UART_HandleTypeDef *huart;
	char *name;
	s16   CheckNum;
}sTRAN_CFG;

typedef struct
{
	UART_HandleTypeDef *huart;
	HAL_LockTypeDef LockRx;
	char *name;
	bool  RxFlag;		//接收标志位
	u8    RxRawState;
	u16   RxRawIndex;
	u8    RxRawDat[TRAN_RX_LEN];	//数据
	u8    RxDat[TRAN_RX_LEN];
	bool  TxFlag;		//接收标志位
	u8    TxHead;
	u8    TxSum;
	u8    TxDat[TRAN_TX_LEN];
	
	sTIM  Tim;			//计时器
	sCNT  Chk;			//检测
	//用户数据
	bool  Update;		//更新  --
	eSTA  Sta;			//状态  --
	eERR  Err;			//错误信息  --
	u32   Time;			//计时
	sSND_FLAG f;
	//------新增------
	int number;
}sTRAN;

#define TRAN_NUM	1
extern sTRAN tran[TRAN_NUM];

void Tran_Init_Para(sTRAN *ele,sTRAN_CFG *elecfg);
bool Tran_Init(sTRAN *ele);
void Tran_Loop_1ms(sTRAN *ele);
bool Tran_Calc(sTRAN *ele);
bool USART3_Transmit_DMA(u8 *pData, u16 Size);

#ifdef __cplusplus
}
#endif

#endif
