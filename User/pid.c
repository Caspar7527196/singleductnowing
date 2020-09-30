#include "pid.h"

/***************************************************\
功能：
  pid控制算法
说明：
  1、串级PID
\***************************************************/

sPID pid[PID_NUM];
sPID *pidRolRate=&pid[0],*pidPitRate=&pid[1],*pidYawRate=&pid[2];
sPID *pidRol=&pid[3],*pidPit=&pid[4],*pidYaw=&pid[5];
sPID *pidXRate=&pid[6],*pidYRate=&pid[7],*pidZRate=&pid[8];
sPID *pidX=&pid[9],*pidY=&pid[10],*pidZ=&pid[11];
sPID *pidTmp=&pid[12];
sPID *pidThrust=&pid[13];

void Pid_Init_Para(sPID *ele,sPID_CFG *elecfg)
{
	ele->Kp = elecfg->Kp;
	ele->Ki = elecfg->Ki;
	ele->Kd = elecfg->Kd;
	ele->Kb = elecfg->Kb;
	ele->eLimit=elecfg->eLimit;
	ele->iLimit = elecfg->iLimit;
	ele->dLimit=elecfg->dLimit;
	ele->integral=0.0f;
	ele->filter_para = elecfg->filter_para;
	ele->setpoint=elecfg->setpoint ;
}

double Pid_Controller(sPID *ele, double err)
{
	ele->lastError=ele->error;
//	ele->error=fConstrain(err,-ele->eLimit,ele->eLimit);
	ele->error=err;
	ele->pout = ele->error * ele->Kp; 
	ele->integral += ele->error*CtrlDt;
	ele->iout = fConstrain(ele->integral * ele->Ki,-ele->iLimit,ele->iLimit);
	ele->dout = (ele->error-ele->lastError)/CtrlDt;
	//--------------误差导数最小二乘滤波--------------
	for(u8 i=1;i<ERRDEF_SLOPE_NUM;i++)
		ele->ErrDefTmp[i-1]=ele->ErrDefTmp[i];
	ele->ErrDefTmp[ERRDEF_SLOPE_NUM-1]=ele->error;
	float ab[2];
	LineFit(ele->ErrDefTmp,ERRDEF_SLOPE_NUM,ab); 
	ele->dout2 = ab[0]*100.0f;  
	//------------------------------------------------
//	ele->dout = fConstrain(ele->dout,-ele->dLimit,ele->dLimit);
	ele->dout3 = ele->dout2*ele->Kd;
	ele->output = ele->pout + ele->iout + ele->dout3;
	return ele->output;
}

double Pid_Controller2(sPID *ele, double err, double rate_err)
{
	ele->lastError=ele->error;
	ele->error=fConstrain(err,-ele->eLimit,ele->eLimit);
	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error*CtrlDt;
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit);
	ele->iout = ele->integral * ele->Ki;
	ele->dout = rate_err*ele->Kd;
	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

float Pid_Position(sPID* ele, float SetXYZ, float FeedBackXYZ)
{
	ele->setpoint = SetXYZ + ele->signal*0.03f;
	ele->feedback = FeedBackXYZ; 
	ele->error = ele->setpoint - ele->feedback;
	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki;
	//不完全微分
	ele->dout = (ele->Kb*(ele->lastfeedback - ele->feedback) + (1.0f - ele->Kb)*ele->dout) * ele->Kd;
	ele->lastfeedback = ele->feedback;
	
	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

float Pid_Speed(sPID* ele, float SetSpeed, float FeedBackSpeed)
{
	ele->setpoint = SetSpeed + ele->signal;
	ele->feedback = FeedBackSpeed; 
	ele->error = ele->setpoint - ele->feedback;
	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki;
	//不完全微分
	ele->dout = (ele->Kb*(ele->lastfeedback - ele->feedback) + (1.0f - ele->Kb)*ele->dout) * ele->Kd;
	ele->lastfeedback = ele->feedback;
	
	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

//PI，纯P控制，D相当于内环的P
float Pid_Angle(sPID* ele, float SetAngle, float FeedBackAngle)
{
	ele->setpoint = SetAngle + ele->signal;
	ele->feedback = FeedBackAngle; 
	ele->error = LoopConstrain(ele->setpoint - ele->feedback,-180.0f,180.0f);
	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki;
	//不完全微分
	ele->dout = (ele->Kb*(ele->lastfeedback - ele->feedback) + (1.0f - ele->Kb)*ele->dout) * ele->Kd;
	ele->lastfeedback = ele->feedback;
	
	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

//PID+微分先行+不完全微分
float Pid_RateAngle(sPID* ele, float SetRateAngle, float FeedBackRateAngle)
{
	ele->setpoint = SetRateAngle + ele->signal;
	ele->feedback = FeedBackRateAngle;
	ele->error = ele->setpoint - ele->feedback;

	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;		
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki; 
	
	//不完全微分
	ele->dout = (ele->Kb*(ele->lastfeedback - ele->feedback) + (1.0f - ele->Kb)*ele->dout) * ele->Kd;
	ele->lastfeedback = ele->feedback;

	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

float Pid_PositionSpeed(sPID* ele, float SetPosition, float FeedBackPosition, float FeedBackSpeed)
{
	ele->setpoint = SetPosition + ele->signal;
	ele->feedback = FeedBackPosition;
	ele->error = ele->setpoint - ele->feedback;

	ele->pout = ele->error * ele->Kp;
	ele->integral += ele->error;		
	ele->integral = fConstrain(ele->integral,-ele->iLimit,ele->iLimit); 
	ele->iout = ele->integral * ele->Ki; 
	
	ele->dout = -FeedBackSpeed * ele->Kd;

	ele->output = ele->pout + ele->iout + ele->dout;
	return ele->output;
}

void Pid_Reset(sPID* ele)
{
	ele->integral = 0;
}

 float Pid_tmp(sPID *ele)
 {   
  ele->error=ele->setpoint - ele->feedback;   
	if(ele->iout <800.0f&&ele->iout>-800.0f)
	{
		if(ele->error> 5.0f||ele->error<- 5.0f) ele->iout=0;    //积分分离
		else  {
	      		  ele->integral += ele->error;            
		        	ele->iout=ele->Ki*ele->integral;
	         	}
	}
	else if  
		(((ele->iout>800||ele->iout==800)&&ele->error<0)||((ele->iout<-800||ele->iout==-800)&&ele->error>0))
    	{   ele->integral += ele->error;            
		    	ele->iout=ele->Ki*ele->integral;
			}
			
	ele->pout=ele->Kp*ele->error;
	//ele->dout=(ele->Kd*(ele->error-ele->lastError)+ele->filter_para*ele->lastdout)/(0.001+ele->filter_para);  //低通滤波微分公式 
  ele->dout=ele->Kd*(ele->error-ele->lastError);
	ele->output=ele->pout+ele->iout;//+ele->dout;
	//ele->output=ele->pout+ele->iout;
	if(ele->output < 0) ele->output=0;
	ele->lastError=ele->error;
	if(ele->output >999)ele->output=999;
	//ele->lastdout=ele->dout;  //此处为上一微分输出值
   return ele->output;
	
}
 
