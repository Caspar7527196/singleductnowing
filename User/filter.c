#include "filter.h"

/**********************************************************************************************
 *功　　能：滑动平均滤波（也称：滑动窗口滤波、递推平均滤波）
 *版    本：V2.0 [2017-03-15]
 **********************************************************************************************/
//滑动平均滤波器初始化
void MovAve_Filter_Init(T_MA_Filt* Filt, float* Buf, uint8_t Num)
{
	Filt->dataBuf = Buf;
	Filt->bufNum = Num;
	Filt->bufCnt = 0;
	Filt->index = 0;
	Filt->lastSum = 0;
}
//滑动平均滤波(moving average filtering)
//适用：一维数据
void MovAve_Filter(float data_in, float* data_out, T_MA_Filt* Filt)
{
	float sum = 0;
	
	if(Filt->bufCnt < Filt->bufNum)	//在刚开始进行滤波时起作用
	{
		Filt->bufCnt++;
	}
	sum = Filt->lastSum + data_in - Filt->dataBuf[Filt->index]; //[V2.0]
	*data_out = sum / (float)Filt->bufCnt; //滑动平均值
	Filt->lastSum = sum;
	
	Filt->dataBuf[Filt->index] = data_in; //更新滑动窗口数组(用最新的数据替换最旧的数据)，即“先进先出”
	Filt->index++;
	if(Filt->index == Filt->bufNum)
		Filt->index = 0;
}
//求近似的变化率(微分)
//方法：滑窗数据按时间先后分为两半，作差。
float MovAve_differential(T_MA_Filt* Filt)
{
	float preVal = 0, nowVal = 0;
	uint8_t ind = Filt->index;
	uint8_t halfN = Filt->bufNum / 2;
	uint8_t i, bound = ind+halfN;
	if(ind <= halfN)
	{
		for(i=ind; i<bound; i++)
		{
			preVal += Filt->dataBuf[i];
		}
		nowVal = (Filt->lastSum - preVal) / halfN; //注：在下次更新前，Filt->lastSum为当前的Sum.
		preVal = preVal / halfN;
	}
	else
	{
		for(i=bound; i<ind; i++)
		{
			nowVal += Filt->dataBuf[i];
		}
		preVal = (Filt->lastSum - nowVal) / halfN;
		nowVal = nowVal / halfN;
	}
	return (nowVal - preVal);
}

/**********************************************************************************************
 *功　　能：滑动平均滤波（也称：滑动窗口滤波、递推平均滤波）
 *版    本：V2.2 [2017-11-05]
 *描    述：用于三维情况
 **********************************************************************************************/
//滑动平均滤波器初始化
//或：void MovAve3_Filter_Init(T_MA3_Filt* Filt, float Buf[][3], uint8_t Num)
void MovAve3_Filter_Init(T_MA3_Filt* Filt, float (*Buf)[3], uint8_t Num)
{
	Filt->dataBuf = Buf;
	Filt->bufNum = Num;
	Filt->bufCnt = 0;
	Filt->index = 0;
	Filt->lastSum[0] = 0;
	Filt->lastSum[1] = 0;
	Filt->lastSum[2] = 0;
}
//滑动平均滤波(moving average filtering)
//适用：三维数据
void MovAve3_Filter(float* data_in, float* data_out, T_MA3_Filt* Filt)
{
	float sum[3] = {0};
	
	if(Filt->bufCnt < Filt->bufNum)	//在刚开始进行滤波时起作用
		Filt->bufCnt++;
	
	sum[0] = Filt->lastSum[0] + data_in[0] - Filt->dataBuf[Filt->index][0];
	sum[1] = Filt->lastSum[1] + data_in[1] - Filt->dataBuf[Filt->index][1];
	sum[2] = Filt->lastSum[2] + data_in[2] - Filt->dataBuf[Filt->index][2];
	data_out[0] = sum[0] / (float)Filt->bufCnt; //滑动平均值
	data_out[1] = sum[1] / (float)Filt->bufCnt;
	data_out[2] = sum[2] / (float)Filt->bufCnt;
	Filt->lastSum[0] = sum[0];
	Filt->lastSum[1] = sum[1];
	Filt->lastSum[2] = sum[2];
	
	Filt->dataBuf[Filt->index][0] = data_in[0]; //更新滑动窗口数组(用最新的数据替换最旧的数据)，即“先进先出”
	Filt->dataBuf[Filt->index][1] = data_in[1];
	Filt->dataBuf[Filt->index][2] = data_in[2];
	
	Filt->index++;
	if(Filt->index == Filt->bufNum)
		Filt->index = 0;
}
//DSP版 （经测试，对于长度为3的数组并无优势，运行时间大约是不用DSP版本的2倍！）
void MovAve3_Filter0(float* data_in, float* data_out, T_MA3_Filt* Filt)
{
	float sum[3] = {0};
	
	if(Filt->bufCnt < Filt->bufNum)	//在刚开始进行滤波时起作用
		Filt->bufCnt++;
	
	arm_add_f32(Filt->lastSum, data_in, sum, 3);
	arm_sub_f32(sum, Filt->dataBuf[Filt->index], sum, 3);
	arm_scale_f32(sum, 1.0f/(float)Filt->bufCnt, data_out, 3); //滑动平均值
	arm_copy_f32(sum, Filt->lastSum, 3);
	
	arm_copy_f32(data_in, Filt->dataBuf[Filt->index], 3); //更新滑动窗口数组(用最新的数据替换最旧的数据)，即“先进先出”
	
	Filt->index++;
	if(Filt->index == Filt->bufNum)
		Filt->index = 0;
}
/*========================================END OF FILE========================================*/
