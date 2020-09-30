#ifndef __MS5611_H
#define __MS5611_H

#include "userlib.h"
//#include "pid.h"
#include "bell.h"

#define MS5611_CMD_RESET			 0x1EU // ADC复位
#define MS5611_CMD_D1_256      0x40U // D1转换，8bit（ADC）
#define MS5611_CMD_D1_512      0x42U // D1转换，9bit（ADC）
#define MS5611_CMD_D1_1024     0x44U // D1转换，10bit（ADC）
#define MS5611_CMD_D1_2048     0x46U // D1转换，11bit（ADC）
#define MS5611_CMD_D1_4096     0x48U // D1转换，12bit（ADC）
#define MS5611_CMD_D2_256      0x50U // D2转换，8bit（ADC）
#define MS5611_CMD_D2_512      0x52U // D2转换，9bit（ADC）
#define MS5611_CMD_D2_1024     0x54U // D2转换，10bit（ADC）
#define MS5611_CMD_D2_2048     0x56U // D2转换，11bit（ADC）
#define MS5611_CMD_D2_4096     0x58U // D2转换，12bit（ADC）
#define MS5611_CMD_ADC_READ	   0x00U // 读取ADC，D1气压，D2温度
#define MS5611_CMD_PROM_RD	   0xA0U // 读取PROM，预留
#define MS5611_CMD_PROM_C1     0xA2U // 读取PROM，C1，校准用
#define MS5611_CMD_PROM_C2     0xA4U // 读取PROM，C2，校准用
#define MS5611_CMD_PROM_C3     0xA6U // 读取PROM，C3，校准用
#define MS5611_CMD_PROM_C4     0xA8U // 读取PROM，C4，校准用
#define MS5611_CMD_PROM_C5     0xAAU // 读取PROM，C5，校准用
#define MS5611_CMD_PROM_C6     0xACU // 读取PROM，C6，校准用
#define MS5611_CMD_PROM_CRC    0xAEU // 读取PROM，CRC，检验

#define MSLP	 101325.0f			// 平均海平面气压 = 1013.25 hPa (1hPa = 100Pa = 1mbar)
#define MS_SLOPE_NUM 20

typedef struct
{
    SPI_HandleTypeDef *hspi;
	char *name;
    s16 FiltNum; 
	u8  Delay;
}sMS5611_CFG;

//实际值：温度单位为摄氏度，陀螺仪为°/s,加速度计为m/s，电压为V，高度为m。
typedef struct
{
	SPI_HandleTypeDef *hspi;  //端口  --
	char *name;       
    sTIM  Tim;       //计时器
    sCNT  Filt;      //均值滤波器  --
    float AltOff;    //m
	u16   PROM[7];   //PROM数据，解算时用
	u8 TmpRaw[3];
	u8 PrsRaw[3];
	bool  RxFlag;
	float TmpRel;    //°C
	float PrsRel;    //Pa
	float AltRel;    //m
	//用户访问数据
	bool  Update;     //更新  --
    eSTA  Sta;        //状态  --
    eERR  Err;        //错误信息  --
	u32   Time;
	bool  Cal;        //校准  --
	u8    StaCnt;
	u8    Delay;
    float TmpFil;    //°C
	float PrsFil;    //Pa
	float AltFil;    //m
	float AltTmp[MS_SLOPE_NUM];  //最小二乘法使用
	float AltSlope;
	
	u32   height_update_time;
	bool  gps_ins_update_flag;
}sMS5611;

#define MS5611_NUM   1  //2
extern sMS5611 ms5611[MS5611_NUM];

void Ms5611_Init_Para(sMS5611 *ele,sMS5611_CFG *elecfg);
bool Ms5611_Init(sMS5611 *ele);
bool Ms5611_Calc(sMS5611 *ele);
void Ms5611_Loop_1ms(sMS5611 *ele);	//气压计状态机
void Ms5611_Ctr(sMS5611 *ele);   //控制气压计温度
//void Tmp_Ctrl(void);   //add by wqz
//void Tmp_Calc(void);


#endif
