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
#define L3G_GYR_FCT 0.0175f     //����

//ʵ��ֵ���¶ȵ�λΪ���϶ȣ�������Ϊ��/s,���ٶȼ�Ϊm/s^2����ѹΪV���߶�Ϊm��
typedef struct
{
    SPI_HandleTypeDef *hspi;
	char *name;
    s16 FiltNum;
    char *Dir;       
    float GyrOffO[3]; //25��
	float GyrOffT[3]; 
}sL3G_CFG;

typedef struct
{
	SPI_HandleTypeDef *hspi;  //�˿�  --
	char *name;       
    sTIM  Tim;        //��ʱ��
    sCNT  Filt;       //��ֵ�˲���  --
    s8    Dir[6];     //����  --
	s16   GyrOff[3];  //Ư����  --
    float GyrOffO[3]; //��Ʈ  --
	float GyrOffT[3]; //��Ʈ  --
    s16   GyrRaw[3];  //ԭʼֵ
    s16   TmpRaw[1];  //�¶�ԭʼֵ
    s16   GyrRot[3];  //��תֵ
    float GyrRel[3];  //ʵ��ֵ
    float TmpRel[1];     //�¶�ʵ��ֵ
	//�û���������
	bool  Update;     //����  --
    eSTA  Sta;        //״̬  --
    eERR  Err;        //������Ϣ  --
	u32   Time;       //��ʱ
	bool  GyrCal;     //У׼  --
    float GyrFil[3];  //�˲�ֵ
    float TmpFil[1];  //�˲�ֵ
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
