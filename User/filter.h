#ifndef __FILTER_H
#define __FILTER_H	
#include "userlib.h"
//#include "arm_math.h"

#ifdef __cplusplus
 extern "C" {
#endif

//滑动平均滤波器参数
typedef struct				//一维数据，指用于对单个变量进行滤波
{
	float    *dataBuf;		//滑动窗口数组（FIFO队列），一维数组的指针
	uint8_t  bufNum;		//队列的长度
	uint8_t  bufCnt;		//有效数据长度
	uint8_t  index;			//队列中最旧数据的索引（或说是环形队列的队尾）
	float    lastSum;		//上次的总和
}T_MA_Filt;

//滑动平均滤波器参数
typedef struct				//三维数据，指用于对三个变量进行滤波
{
	float    (*dataBuf)[3];	//滑动窗口数组（FIFO队列），二维数组的指针(?)
	uint8_t  bufNum;		//队列的长度
	uint8_t  bufCnt;		//有效数据长度
	uint8_t  index;			//队列中最旧数据的索引（或说是环形队列的队尾）
	float    lastSum[3];	//上次的总和
}T_MA3_Filt;

//适用：一维数据
void MovAve_Filter_Init(T_MA_Filt* Filt, float* Buf, uint8_t Num); //滑动平均滤波初始化
void MovAve_Filter(float data_in, float* data_out, T_MA_Filt* Filt); //滑动平均滤波
float MovAve_differential(T_MA_Filt* Filt); //用滑窗数据来求近似的变化率(微分)

//适用：三维数据
void MovAve3_Filter_Init(T_MA3_Filt* Filt, float (*Buf)[3], uint8_t Num); //滑动平均滤波初始化
void MovAve3_Filter(float* data_in, float* data_out, T_MA3_Filt* Filt); //滑动平均滤波
//float MovAve3_differential(T_MA3_Filt* Filt); //用滑窗数据来求近似的变化率(微分)

#ifdef __cplusplus
}
#endif
	
#endif
/*========================================END OF FILE========================================*/
