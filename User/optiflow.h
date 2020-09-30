#ifndef __OPTIFLOW_H
#define __OPTIFLOW_H
#include "userlib.h"
#include "arm_math.h"

#define FLOW_RX_LEN		50	//20
//#define __USE_IMU_DATA
//#define __EN_COORDINATE_TRANS
#define __FLOW_DATA_CTRL	//�������Ƿ�ʹ�ù�������

//�����������������ٶ�Gȡ9.8m/s^2 ��
#define OF_IMU_ACC_FCT	0.00239258f	//���ٶȼ����̣���8G��FCT = (16/65536)*G (m/s^2)
#define OF_IMU_GYR_FCT	0.00106526f	//���������̣���2000��/s��FCT = 4000/65536*pi/180 (rad/s)
#define OF_IMU_GYR_FCT2	0.06103516f	//���������̣���2000��/s��FCT = 4000/65536 (��/s)

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
	s8    Dir[6];		//����  --
	bool  RxFlag;		//���ձ�־λ
	u8    RxRawDat[FLOW_RX_LEN]; //����
	u16   RxRawIndex;
	u8    RxDat[FLOW_RX_LEN];
	sTIM  Tim;			//��ʱ��
	sTIM  Tim2;			//��ʱ�������ڻ��ּ���λ�õļ�ʱ��
	sCNT  Chk;			//���
	float dt;
	
	//�û���������
	bool  Update;		//����  --
	eSTA  Sta;			//״̬  --
	eERR  Err;			//������Ϣ  --
	u32   Time;			//��ʱ
	//����ģ�����ݣ���������������ϵ�� ------------------------------
	u8    Quality;		//������Ϣ��������ֵԽ�󣬱�ʾ����Խ�ã�
	u8    Light;		//����ǿ��
	s8    DX, DY;		//ԭʼ������Ϣ����Ӧ�ƶ����ٶȣ������ƶ����ʣ�
	s16   DX2, DY2;		//�ںϺ�Ĺ�����Ϣ����Ӧ�ƶ����ٶȣ�cm/s��
	s16   DX2Fix,DY2Fix;//�����ٶ�����ֵ��cm/s��
	u16   Alt, Alt2;	//ԭʼ�߶���Ϣ���ںϺ�߶���Ϣ����λ��cm��
	s16   Gyr[3];		//ԭʼ����������
	s16   Gyr2[3];		//�˲�������������
	s16   Acc[3];		//ԭʼ���ٶ�����
	s16   Acc2[3];		//�˲�����ٶ�����
	float AttAng[3];	//ŷ���Ǹ�ʽ����̬���ݣ�ROL,PIT,YAW����λ���㣩
	float AttQtn[4];	//��Ԫ����ʽ����̬����
	float Dist;			//��Alt��Alt2ת����λ����
	//
	u8    OF_Mode;
	u8    ALT_Mode;
	u8    IMU_Mode;
	u8    ATT_Mode;
	bool  OFUpdate;
	bool  AltUpdate;
	bool  ImuUpdate;
	bool  AttUpdate;
	
	//��������ϵ ----------------------------------------------------
	s16   GyrRot[3];	//��תֵ
	s16   AccRot[3];	//��תֵ
	float GyrRel[3];	//ʵ��ֵ
	float AccRel[3];	//ʵ��ֵ
	//��1����Ϊ"DX2,DY2,DX2Fix,DY2Fix"�ǻ�������ϵ�µģ����Bϵ��
//	float UvwB[3];		//��DX2,DY2ת����λ��������λ��m/s��
//	float UvwBFix[3];	//��DX2Fix,DY2Fixת����λ��������λ��m/s��
//	float XyzB[3];		//X,Y�ֱ���DX2Fix,DY2Fix���ֵ�������Z��Dist����λ��m��
	
	//Hϵ -----------------------------------------------------------
	//��2����Ϊ"DX2,DY2,DX2Fix,DY2Fix"��Hϵ�µģ�����ˮƽ��������ϵ�����Hϵ��
	float UvwH[3];		//��DX2,DY2ת����λ��������λ��m/s��
//	float UvwHFix[3];	//��DX2Fix,DY2Fixת����λ��������λ��m/s��
	float XyzH[3];		//X,Y�ֱ���DX2Fix,DY2Fix���ֵ�������Z��Dist*cos(��)*cos(��)����λ��m��
	
	//NEDϵ ---------------------------------------------------------
//	float Uvw[3];
//	float Xyz[3];
}sFLOW;

/* ˵����
1.�ں�ǰ�Ĺ�������"DX,DY"��
  �ٶȼ��㹫ʽ���ƶ��ٶ� = ����ԭʼ���� * 3.8 * 0.02 * ��Ը߶�/�롣
2.�ںϺ�Ĺ�������"DX2,DY2"��
  ֱ��Ϊ�ٶ����ݣ���λΪcm/s������С������ƫ���ʺ����ٶȻ����ơ�
3.�����ٶ�����ֵ"DX2Fix, DY2Fix"��
  ������������ٶ����ݣ���λΪcm/s�������ϴ󣬻��ֺ�ƫ�Ʒ���С���ʺ����ڻ��ּ��㡣
4.������Ϣ�ο�����ģ���ֲᡣ
*/

#define FLOW_NUM	1
extern sFLOW flow[FLOW_NUM];

void Flow_Init_Para(sFLOW *ele, sFLOW_CFG *elecfg);
bool Flow_Init(sFLOW *ele);
bool Flow_Calc(sFLOW *ele);

#endif
