#ifndef __PPM_H
#define __PPM_H

#include "userlib.h"


//#include "command.h"

#define RC_RX_LEN 200
#define RC_TX_LEN 30
#define PPM_NUM 27   
#define PWM_RC_NUM 8   

#define RC_FUN_MIN			1450   //判别用，不是实际的
#define RC_FUN_MAX			1600
#define RC_PPM_MIN          1000   //实际
#define RC_PPM_MID          1500   //实际
#define RC_PPM_MAX          2000   //实际

typedef enum
{
	RC_NONE=0xFFU,    //无行为
	RC_FUN0=0x00U,    //右下 右下
	RC_MOT_UNLOCK=0x01U,  //右下 左下  解锁
	RC_FUN2=0x02U,    //右上 右下
	RC_FUN3=0x03U,    //右上 左下
	RC_FUN4=0x04U,    //右下 右上
	RC_FUN5=0x05U,    //右下 左上
	RC_FUN6=0x06U,    //右上 右上
	RC_FUN7=0x07U,    //右上 左上
	RC_MOT_LOCK=0x08U,    //左下 右下  上锁
	RC_FUN9=0x09U,    //左下 左下
	RC_FUN10=0x0AU,   //左上 右下
	RC_FUN11=0x0BU,   //左上 左下
	RC_FUN12=0x0CU,   //左下 右上
	RC_FUN13=0x0DU,   //左下 左上
	RC_FUN14=0x0EU,   //左上 右上
	RC_FUN15=0x0FU,   //左上 左上	
}eRC_MODE;

typedef struct
{
	UART_HandleTypeDef *huart;
	char *name;
	s16   CheckNum;
	s16   ModeNum;
	s16   HIG_THR;    //油门高位点
	s16   MID_THR;    //油门中位点
	s16   LOW_THR;    //油门低位点
}sRC_CFG;

typedef struct
{
	UART_HandleTypeDef *huart;
	HAL_LockTypeDef LockRx;
	char *name;
	char RxRawDat[RC_RX_LEN];   //数据
	u16  RxRawIndex;
	bool RxRawHead;
	bool RxFlag;      //接收标志位
	char RxDat[RC_RX_LEN];
	bool TxFlag;      //接收标志位
	u8   TxDat[RC_TX_LEN];
	
	sTIM  Tim;        //计时器
	sCNT  Chk;        //检测
	//用户访问数
	bool  Update;     //更新  --
	eSTA  Sta;        //状态  --
	eERR  Err;        //错误信息  --
	u32   Time;       //计时
	u8    Pack;       //收包个数，1s
	eRC_MODE  Mode;   //行为模式
	sCNT ModeChk;
	u16  PPM[PWM_RC_NUM];
	float Val[4];     //roll pitch yaw high
	float Ang[2];     //遥控器角度 roll pitch
	float dAng;       //yaw角速度
	float Thr;        //遥控器油门
	float Uvw[3];     //遥控器Uvw
	float a,b,c;      //油门2次曲线
	s16   HIG_THR;    //油门高位点
	s16   MID_THR;    //油门中位点
	s16   LOW_THR;    //油门低位点
	u8    Key[4];     //4个开关
	
	unsigned int buffer[200];
	unsigned int rc_check;
	unsigned int cnt;
}sRC;

#define RC_NUM 1
extern sRC rc[RC_NUM];

bool UART4_Transmit_DMA(u8 *pData, u16 Size);

void Rc_Init_Para(sRC *ele,sRC_CFG *elecfg);
bool Rc_Init(sRC *ele);
bool Rc_Calc(sRC *ele);

#endif
