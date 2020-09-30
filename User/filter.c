#include "filter.h"

/**********************************************************************************************
 *�������ܣ�����ƽ���˲���Ҳ�ƣ����������˲�������ƽ���˲���
 *��    ����V2.0 [2017-03-15]
 **********************************************************************************************/
//����ƽ���˲�����ʼ��
void MovAve_Filter_Init(T_MA_Filt* Filt, float* Buf, uint8_t Num)
{
	Filt->dataBuf = Buf;
	Filt->bufNum = Num;
	Filt->bufCnt = 0;
	Filt->index = 0;
	Filt->lastSum = 0;
}
//����ƽ���˲�(moving average filtering)
//���ã�һά����
void MovAve_Filter(float data_in, float* data_out, T_MA_Filt* Filt)
{
	float sum = 0;
	
	if(Filt->bufCnt < Filt->bufNum)	//�ڸտ�ʼ�����˲�ʱ������
	{
		Filt->bufCnt++;
	}
	sum = Filt->lastSum + data_in - Filt->dataBuf[Filt->index]; //[V2.0]
	*data_out = sum / (float)Filt->bufCnt; //����ƽ��ֵ
	Filt->lastSum = sum;
	
	Filt->dataBuf[Filt->index] = data_in; //���»�����������(�����µ������滻��ɵ�����)�������Ƚ��ȳ���
	Filt->index++;
	if(Filt->index == Filt->bufNum)
		Filt->index = 0;
}
//����Ƶı仯��(΢��)
//�������������ݰ�ʱ���Ⱥ��Ϊ���룬���
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
		nowVal = (Filt->lastSum - preVal) / halfN; //ע�����´θ���ǰ��Filt->lastSumΪ��ǰ��Sum.
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
 *�������ܣ�����ƽ���˲���Ҳ�ƣ����������˲�������ƽ���˲���
 *��    ����V2.2 [2017-11-05]
 *��    ����������ά���
 **********************************************************************************************/
//����ƽ���˲�����ʼ��
//��void MovAve3_Filter_Init(T_MA3_Filt* Filt, float Buf[][3], uint8_t Num)
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
//����ƽ���˲�(moving average filtering)
//���ã���ά����
void MovAve3_Filter(float* data_in, float* data_out, T_MA3_Filt* Filt)
{
	float sum[3] = {0};
	
	if(Filt->bufCnt < Filt->bufNum)	//�ڸտ�ʼ�����˲�ʱ������
		Filt->bufCnt++;
	
	sum[0] = Filt->lastSum[0] + data_in[0] - Filt->dataBuf[Filt->index][0];
	sum[1] = Filt->lastSum[1] + data_in[1] - Filt->dataBuf[Filt->index][1];
	sum[2] = Filt->lastSum[2] + data_in[2] - Filt->dataBuf[Filt->index][2];
	data_out[0] = sum[0] / (float)Filt->bufCnt; //����ƽ��ֵ
	data_out[1] = sum[1] / (float)Filt->bufCnt;
	data_out[2] = sum[2] / (float)Filt->bufCnt;
	Filt->lastSum[0] = sum[0];
	Filt->lastSum[1] = sum[1];
	Filt->lastSum[2] = sum[2];
	
	Filt->dataBuf[Filt->index][0] = data_in[0]; //���»�����������(�����µ������滻��ɵ�����)�������Ƚ��ȳ���
	Filt->dataBuf[Filt->index][1] = data_in[1];
	Filt->dataBuf[Filt->index][2] = data_in[2];
	
	Filt->index++;
	if(Filt->index == Filt->bufNum)
		Filt->index = 0;
}
//DSP�� �������ԣ����ڳ���Ϊ3�����鲢�����ƣ�����ʱ���Լ�ǲ���DSP�汾��2������
void MovAve3_Filter0(float* data_in, float* data_out, T_MA3_Filt* Filt)
{
	float sum[3] = {0};
	
	if(Filt->bufCnt < Filt->bufNum)	//�ڸտ�ʼ�����˲�ʱ������
		Filt->bufCnt++;
	
	arm_add_f32(Filt->lastSum, data_in, sum, 3);
	arm_sub_f32(sum, Filt->dataBuf[Filt->index], sum, 3);
	arm_scale_f32(sum, 1.0f/(float)Filt->bufCnt, data_out, 3); //����ƽ��ֵ
	arm_copy_f32(sum, Filt->lastSum, 3);
	
	arm_copy_f32(data_in, Filt->dataBuf[Filt->index], 3); //���»�����������(�����µ������滻��ɵ�����)�������Ƚ��ȳ���
	
	Filt->index++;
	if(Filt->index == Filt->bufNum)
		Filt->index = 0;
}
/*========================================END OF FILE========================================*/
