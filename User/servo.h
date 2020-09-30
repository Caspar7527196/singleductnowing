#ifndef __SERVO_H
#define __SERVO_H


#include "userlib.h"
#include "motor.h"

#define SERVO_TX_LEN		12

typedef struct
{
	UART_HandleTypeDef *huart;
	char *name;
	s16   CheckNum;
}sSERVO_CFG;

typedef struct
{
	UART_HandleTypeDef *huart;
	HAL_LockTypeDef LockRx;
	char *name;

	bool  TxFlag;		//���ձ�־λ
	u8    TxHead;
	u8    TxSum;
	u8    TxDat[SERVO_TX_LEN];
	
	sTIM  Tim;			//��ʱ��
	sCNT  Chk;			//���
	//�û�����
	bool  Update;		//����  --
	eSTA  Sta;			//״̬  --
	eERR  Err;			//������Ϣ  --
	u32   Time;			//��ʱ
	//------����------
}sSERVO;

#define SERVO_NUM	1
extern sSERVO servo[SERVO_NUM];

void Servo_Init_Para(sSERVO *ele,sSERVO_CFG *elecfg);
bool Servo_Init(sSERVO *ele);
bool Servo_Calc(sSERVO *ele);
bool UART3_Transmit_DMA(u8 *pData, u16 Size);





#endif
