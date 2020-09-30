#ifndef __PID_H
#define __PID_H

#include "userlib.h"
#include "ahrs.h"
#include "control.h"

#define ERRDEF_SLOPE_NUM 20


typedef struct
{
	float Kp,Ki,Kd,Kb;
	float eLimit;
	float iLimit;
	float dLimit;
	float filter_para;
	float setpoint;		// �趨ֵ
}sPID_CFG;

typedef struct
{
	float setpoint;		// �趨ֵ
	float feedback;		// ����ֵ
	float lastfeedback; // ��һ������ֵ
	float error;		// ���
	float lastError;    // ��һ�����
	float integral;		// ��ǰ����ֵ
	float eLimit;    //����޷�
	float iLimit;		// �����޷�ֵ
	float dLimit;   //΢���޷�
	float Kp;			// ����ϵ�� proportional gain
	float Ki;			// ����ϵ�� integral gain
	float Kd;			// ΢��ϵ�� differential gain
	float Kb;           // ΢������ϵ��
	float pout;			// (debugging)
	float iout;			// (debugging)
	float dout;			// (debugging)
	float output;		// ��ǰPID�����
	float signal;		// ��Ծ�ź�
	float filter_para; //��ͨ�˲�����
	float ErrDefTmp[ERRDEF_SLOPE_NUM];
	float dout2;
	float dout3;
}sPID;

#define PID_NUM 18

extern sPID pid[PID_NUM];
extern sPID *pidRolRate,*pidPitRate,*pidYawRate;
extern sPID *pidRol,*pidPit,*pidYaw;
extern sPID *pidXRate,*pidYRate,*pidZRate;
extern sPID *pidX,*pidY,*pidZ;
extern sPID *pidTmp;
extern sPID *pidThrust;

void Pid_Init_Para(sPID *ele,sPID_CFG *elecfg);
void Pid_Reset(sPID *ele);
float Pid_Position(sPID* ele, float SetXYZ, float FeedBackXYZ);
float Pid_Speed(sPID* ele, float SetSpeed, float FeedBackSpeed);
float Pid_Angle(sPID* ele, float SetAngle, float FeedBackAngle);
float Pid_RateAngle(sPID* ele, float SetRateAngle, float FeedBackRateAngle);
float Pid_PositionSpeed(sPID* ele, float SetPosition, float FeedBackPosition, float FeedBackSpeed);
double Pid_Controller(sPID *ele, double err);
double Pid_Controller2(sPID *ele, double err, double rate_err);
float Pid_tmp(sPID *ele);
#endif
