#ifndef __MS5611_H
#define __MS5611_H

#include "userlib.h"
//#include "pid.h"
#include "bell.h"

#define MS5611_CMD_RESET			 0x1EU // ADC��λ
#define MS5611_CMD_D1_256      0x40U // D1ת����8bit��ADC��
#define MS5611_CMD_D1_512      0x42U // D1ת����9bit��ADC��
#define MS5611_CMD_D1_1024     0x44U // D1ת����10bit��ADC��
#define MS5611_CMD_D1_2048     0x46U // D1ת����11bit��ADC��
#define MS5611_CMD_D1_4096     0x48U // D1ת����12bit��ADC��
#define MS5611_CMD_D2_256      0x50U // D2ת����8bit��ADC��
#define MS5611_CMD_D2_512      0x52U // D2ת����9bit��ADC��
#define MS5611_CMD_D2_1024     0x54U // D2ת����10bit��ADC��
#define MS5611_CMD_D2_2048     0x56U // D2ת����11bit��ADC��
#define MS5611_CMD_D2_4096     0x58U // D2ת����12bit��ADC��
#define MS5611_CMD_ADC_READ	   0x00U // ��ȡADC��D1��ѹ��D2�¶�
#define MS5611_CMD_PROM_RD	   0xA0U // ��ȡPROM��Ԥ��
#define MS5611_CMD_PROM_C1     0xA2U // ��ȡPROM��C1��У׼��
#define MS5611_CMD_PROM_C2     0xA4U // ��ȡPROM��C2��У׼��
#define MS5611_CMD_PROM_C3     0xA6U // ��ȡPROM��C3��У׼��
#define MS5611_CMD_PROM_C4     0xA8U // ��ȡPROM��C4��У׼��
#define MS5611_CMD_PROM_C5     0xAAU // ��ȡPROM��C5��У׼��
#define MS5611_CMD_PROM_C6     0xACU // ��ȡPROM��C6��У׼��
#define MS5611_CMD_PROM_CRC    0xAEU // ��ȡPROM��CRC������

#define MSLP	 101325.0f			// ƽ����ƽ����ѹ = 1013.25 hPa (1hPa = 100Pa = 1mbar)
#define MS_SLOPE_NUM 20

typedef struct
{
    SPI_HandleTypeDef *hspi;
	char *name;
    s16 FiltNum; 
	u8  Delay;
}sMS5611_CFG;

//ʵ��ֵ���¶ȵ�λΪ���϶ȣ�������Ϊ��/s,���ٶȼ�Ϊm/s����ѹΪV���߶�Ϊm��
typedef struct
{
	SPI_HandleTypeDef *hspi;  //�˿�  --
	char *name;       
    sTIM  Tim;       //��ʱ��
    sCNT  Filt;      //��ֵ�˲���  --
    float AltOff;    //m
	u16   PROM[7];   //PROM���ݣ�����ʱ��
	u8 TmpRaw[3];
	u8 PrsRaw[3];
	bool  RxFlag;
	float TmpRel;    //��C
	float PrsRel;    //Pa
	float AltRel;    //m
	//�û���������
	bool  Update;     //����  --
    eSTA  Sta;        //״̬  --
    eERR  Err;        //������Ϣ  --
	u32   Time;
	bool  Cal;        //У׼  --
	u8    StaCnt;
	u8    Delay;
    float TmpFil;    //��C
	float PrsFil;    //Pa
	float AltFil;    //m
	float AltTmp[MS_SLOPE_NUM];  //��С���˷�ʹ��
	float AltSlope;
	
	u32   height_update_time;
	bool  gps_ins_update_flag;
}sMS5611;

#define MS5611_NUM   1  //2
extern sMS5611 ms5611[MS5611_NUM];

void Ms5611_Init_Para(sMS5611 *ele,sMS5611_CFG *elecfg);
bool Ms5611_Init(sMS5611 *ele);
bool Ms5611_Calc(sMS5611 *ele);
void Ms5611_Loop_1ms(sMS5611 *ele);	//��ѹ��״̬��
void Ms5611_Ctr(sMS5611 *ele);   //������ѹ���¶�
//void Tmp_Ctrl(void);   //add by wqz
//void Tmp_Calc(void);


#endif
