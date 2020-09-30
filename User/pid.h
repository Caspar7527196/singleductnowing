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
	float setpoint;		// 设定值
}sPID_CFG;

typedef struct
{
	float setpoint;		// 设定值
	float feedback;		// 反馈值
	float lastfeedback; // 上一个反馈值
	float error;		// 误差
	float lastError;    // 上一个误差
	float integral;		// 当前积分值
	float eLimit;    //误差限幅
	float iLimit;		// 积分限幅值
	float dLimit;   //微分限幅
	float Kp;			// 比例系数 proportional gain
	float Ki;			// 积分系数 integral gain
	float Kd;			// 微分系数 differential gain
	float Kb;           // 微分先行系数
	float pout;			// (debugging)
	float iout;			// (debugging)
	float dout;			// (debugging)
	float output;		// 当前PID的输出
	float signal;		// 阶跃信号
	float filter_para; //低通滤波参数
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
