#ifndef __FILTER_H
#define __FILTER_H	
#include "userlib.h"
//#include "arm_math.h"

#ifdef __cplusplus
 extern "C" {
#endif

//����ƽ���˲�������
typedef struct				//һά���ݣ�ָ���ڶԵ������������˲�
{
	float    *dataBuf;		//�����������飨FIFO���У���һά�����ָ��
	uint8_t  bufNum;		//���еĳ���
	uint8_t  bufCnt;		//��Ч���ݳ���
	uint8_t  index;			//������������ݵ���������˵�ǻ��ζ��еĶ�β��
	float    lastSum;		//�ϴε��ܺ�
}T_MA_Filt;

//����ƽ���˲�������
typedef struct				//��ά���ݣ�ָ���ڶ��������������˲�
{
	float    (*dataBuf)[3];	//�����������飨FIFO���У�����ά�����ָ��(?)
	uint8_t  bufNum;		//���еĳ���
	uint8_t  bufCnt;		//��Ч���ݳ���
	uint8_t  index;			//������������ݵ���������˵�ǻ��ζ��еĶ�β��
	float    lastSum[3];	//�ϴε��ܺ�
}T_MA3_Filt;

//���ã�һά����
void MovAve_Filter_Init(T_MA_Filt* Filt, float* Buf, uint8_t Num); //����ƽ���˲���ʼ��
void MovAve_Filter(float data_in, float* data_out, T_MA_Filt* Filt); //����ƽ���˲�
float MovAve_differential(T_MA_Filt* Filt); //�û�������������Ƶı仯��(΢��)

//���ã���ά����
void MovAve3_Filter_Init(T_MA3_Filt* Filt, float (*Buf)[3], uint8_t Num); //����ƽ���˲���ʼ��
void MovAve3_Filter(float* data_in, float* data_out, T_MA3_Filt* Filt); //����ƽ���˲�
//float MovAve3_differential(T_MA3_Filt* Filt); //�û�������������Ƶı仯��(΢��)

#ifdef __cplusplus
}
#endif
	
#endif
/*========================================END OF FILE========================================*/
