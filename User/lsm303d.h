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
#define LSM_INT_CTRL_M		0x12U	//0xE8  �������ж����ã�������0x00���о�Ĭ��ֵӦ����0x00
#define LSM_INT_SRC_M		0x13U	//��ֵ��������
#define LSM_INT_THS_L_M		0x14U	//��ֵ����
#define LSM_INT_THS_H_M		0x15U
#define LSM_OFFSET_X_L_M	0x16U	//Ư��������
#define LSM_OFFSET_X_H_M	0x17U
#define LSM_OFFSET_Y_L_M	0x18U
#define LSM_OFFSET_Y_H_M	0x19U
#define LSM_OFFSET_Z_L_M	0x1AU
#define LSM_OFFSET_Z_H_M	0x1BU
#define LSM_REFERENCE_X		0x1CU	//����ٶ����ݽ��и�ͨ�˲��Ĳο�ֵ
#define LSM_REFERENCE_Y		0x1DU
#define LSM_REFERENCE_Z		0x1EU
#define LSM_CTRL0			0x1FU	//0x00  ��ͨ�˲���������FIFO���� 0x00
#define LSM_CTRL1			0x20U	//0x07  ���ٶȼ�Ƶ�� ���ٶȸ���ʹ������·�ʽ 0x37�������У�Ƶ��Ϊ12.5Hz
#define LSM_CTRL2			0x21U	//0x00  ���ٶȼƴ����뷶Χ�Լ��Լ� 0xC8���Լ� 0xCA�Լ�   50Hz����+-4g����
#define LSM_CTRL3			0x22U	//0x00  �ж����� 0x00
#define LSM_CTRL4			0x23U	//0x00  �ж����� 0x00
#define LSM_CTRL5			0x24U	//0x18  �¶ȼ��ʹ�� �����ƾ�����Ƶ�ʣ�0xE8 �����¶ȼ�⣬Ƶ��Ϊ12.5Hz
#define LSM_CTRL6			0x25U	//0x20  �����Ʒ�Χ 0x20Ĭ��4gauss
#define LSM_CTRL7			0x26U	//0x01  ����ģʽ ����������ת�� 0x80
#define LSM_STATUS_A		0x27U
#define LSM_OUT_X_L_A		0x28U
#define LSM_OUT_X_H_A		0x29U
#define LSM_OUT_Y_L_A		0x2AU
#define LSM_OUT_Y_H_A		0x2BU
#define LSM_OUT_Z_L_A		0x2CU
#define LSM_OUT_Z_H_A		0x2DU
#define LSM_FIFO_CTRL		0x2EU	//FIFO����
#define LSM_FIFO_SRC		0x2FU	//FIFO״̬
#define LSM_IG_CFG1			0x30U	//IG��Ϊ�ж����
#define LSM_IG_SRC1			0x31U
#define LSM_IG_THS1			0x32U
#define LSM_IG_DUR1			0x33U
#define LSM_IG_CFG2			0x34U
#define LSM_IG_SRC2			0x35U
#define LSM_IG_THS2			0x36U
#define LSM_IG_DUR2			0x37U
#define LSM_CLICK_CFG		0x38U	//�ж����
#define LSM_CLICK_SRC		0x39U 
#define LSM_CLICK_THS		0x3AU
#define LSM_TIME_LIMIT		0x3BU
#define LSM_TIME_LATENCY	0x3CU
#define LSM_TIME_WINDOW		0x3DU
#define LSM_Act_THS			0x3EU
#define LSM_Act_DUR			0x3FU

#define LSM303D_ADDR		0x3C

#define LSM_ACC_FCT			0.00119708f		//��λ����Ϊ��m/s^2��������9.8065
#define LSM_MAG_FCT			0.160f			//��λ����Ϊ��mgauss���ų���500~600mguass

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

//ʵ��ֵ���¶ȵ�λΪ���϶ȣ�������Ϊ��/s,���ٶȼ�Ϊm/s^2����ѹΪV���߶�Ϊm��
typedef struct
{
	SPI_HandleTypeDef *hspi; //�˿�  --
	char *name;       
	sTIM  Tim;			//��ʱ��
	sCNT  Filt;			//��ֵ�˲���  --
	s8    Dir[6];		//����  --
	u8    AccCalFlag;
	s16   AccOff[3];	//Ư����  --
	s16   MagOff[3];	//ƫ����  --
	float MagMat[9];	//У׼�������ڰ���������Ϊ��
	s16   AccRaw[3];	//ԭʼֵ
	s16   MagRaw[3];	//ԭʼֵ
	s16   TmpRaw[1];	//ԭʼֵ
	s16   AccRot[3];	//��תֵ
	s16   MagRot[3];	//��תֵ
	float AccRel[3];	//ʵ��ֵ
	float MagRel[3];	//ʵ��ֵ
	float TmpRel[1];	//ʵ��ֵ
	//�û���������
	bool  Update;		//����  --
	eSTA  Sta;			//״̬  --
	eERR  Err;			//������Ϣ  --
	u32   Time;			//��ʱ
	bool  MagCal;		//У׼  --
	bool  AccCal;		//У׼  --
	float OneG;			//����  --
	float AccFil[3];	//�˲�ֵ
	float MagFil[3];	//�˲�ֵ
	float TmpFil[1];	//�˲�ֵ
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
