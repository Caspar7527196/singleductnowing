#ifndef __PPM_H
#define __PPM_H

#include "userlib.h"


//#include "command.h"

#define RC_RX_LEN 200
#define RC_TX_LEN 30
#define PPM_NUM 27   
#define PWM_RC_NUM 8   

#define RC_FUN_MIN			1450   //�б��ã�����ʵ�ʵ�
#define RC_FUN_MAX			1600
#define RC_PPM_MIN          1000   //ʵ��
#define RC_PPM_MID          1500   //ʵ��
#define RC_PPM_MAX          2000   //ʵ��

typedef enum
{
	RC_NONE=0xFFU,    //����Ϊ
	RC_FUN0=0x00U,    //���� ����
	RC_MOT_UNLOCK=0x01U,  //���� ����  ����
	RC_FUN2=0x02U,    //���� ����
	RC_FUN3=0x03U,    //���� ����
	RC_FUN4=0x04U,    //���� ����
	RC_FUN5=0x05U,    //���� ����
	RC_FUN6=0x06U,    //���� ����
	RC_FUN7=0x07U,    //���� ����
	RC_MOT_LOCK=0x08U,    //���� ����  ����
	RC_FUN9=0x09U,    //���� ����
	RC_FUN10=0x0AU,   //���� ����
	RC_FUN11=0x0BU,   //���� ����
	RC_FUN12=0x0CU,   //���� ����
	RC_FUN13=0x0DU,   //���� ����
	RC_FUN14=0x0EU,   //���� ����
	RC_FUN15=0x0FU,   //���� ����	
}eRC_MODE;

typedef struct
{
	UART_HandleTypeDef *huart;
	char *name;
	s16   CheckNum;
	s16   ModeNum;
	s16   HIG_THR;    //���Ÿ�λ��
	s16   MID_THR;    //������λ��
	s16   LOW_THR;    //���ŵ�λ��
}sRC_CFG;

typedef struct
{
	UART_HandleTypeDef *huart;
	HAL_LockTypeDef LockRx;
	char *name;
	char RxRawDat[RC_RX_LEN];   //����
	u16  RxRawIndex;
	bool RxRawHead;
	bool RxFlag;      //���ձ�־λ
	char RxDat[RC_RX_LEN];
	bool TxFlag;      //���ձ�־λ
	u8   TxDat[RC_TX_LEN];
	
	sTIM  Tim;        //��ʱ��
	sCNT  Chk;        //���
	//�û�������
	bool  Update;     //����  --
	eSTA  Sta;        //״̬  --
	eERR  Err;        //������Ϣ  --
	u32   Time;       //��ʱ
	u8    Pack;       //�հ�������1s
	eRC_MODE  Mode;   //��Ϊģʽ
	sCNT ModeChk;
	u16  PPM[PWM_RC_NUM];
	float Val[4];     //roll pitch yaw high
	float Ang[2];     //ң�����Ƕ� roll pitch
	float dAng;       //yaw���ٶ�
	float Thr;        //ң��������
	float Uvw[3];     //ң����Uvw
	float a,b,c;      //����2������
	s16   HIG_THR;    //���Ÿ�λ��
	s16   MID_THR;    //������λ��
	s16   LOW_THR;    //���ŵ�λ��
	u8    Key[4];     //4������
	
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
