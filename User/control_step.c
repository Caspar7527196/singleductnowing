#include "control.h"
#include "control_step.h"
#include "math.h" 

void Control_Step(sIO *ele)
{   
	Control_Feedback(CtrlFbck);       //控制测量反馈
//	ControlRC_Check(CtrlIO);          //检查遥控器信号
	Integral_Reset(CtrlKiTemp);       //起降重置积分
	//--------------保存上次模式信息---------//
	
	ele->last_control_mode = ele->control_mode;
	ele->last_yaw_mode     = ele->yaw_mode    ;
	ele->last_fly_mode     = ele->fly_mode    ;
	
	//----------------摇杆信号---------------//
	
	ele->input[0] = -rc->Val[0];
	ele->input[1] = -rc->Val[1];
	ele->input[2] =  rc->Val[2];
	ele->input[3] =  rc->Val[3];
	
	//----------------飞行模式---------------//
	
	if      (rc->Key[3]==0 )
	{
		ele->fly_mode=0;//直升机模式
	}
	else if (rc->Key[3]==2 )
	{
		ele->fly_mode=1;//固定翼模式
		if (ele->last_control_mode==0)
		{
		  Transition_Init(CtrlLpIO);	
		}
//Transition_Calc(CtrlLpIO);//倾转规划
	}	
	
	//----------------锁尾模式---------------//
	
	if      (rc->Key[0]==0)   ele->yaw_mode=0;//解除锁尾
	else if (rc->Key[0]==2)   ele->yaw_mode=1;//锁尾，仅限直升机模式
	
	//----------------控制模式---------------//
	
	if (CtrlIO->rc_status!=LOST)
	{
		switch(rc->Key[2])
		{
			case 0:
				ele->control_mode = 0;	//手控
				break;
			case 1:
				ele->control_mode=1;	//半自控
				break;
			case 2:
				if (ele->fly_mode==0)		
				{
					Out_Loop_XY_Pre(CtrlLpIO); //判断定点模式、刹车模式
				}
				if (CtrlLpIO->position_mode==false && ele->fly_mode==0) ele->control_mode = 2;			//速度操纵
				else                                                    ele->control_mode = 3;			//自控
				break;
			default:break;
		}
	}
	else
	{
		ele->control_mode = 4;	//失控保护
	}
	
	//----------------控制逻辑----------------//
	
	switch(ele->control_mode)
	{
		case 0://手控
			In_Loop_Step(CtrlLpIO);              //角度环(锁尾)
		  CtrlLpIO->   pqr_command[0] = CtrlIO->input[0]*0.225f*D2R;//滚转角速度输入0.225对应90°/s
		  CtrlLpIO->   pqr_command[1] = CtrlIO->input[1]*0.225f*D2R;//俯仰角速度输入0.225对应90°/s	
		  CtrlLpIO->thrust_command[0] = (CtrlIO->input[3]-1500)/(1500-1090)*ICM20602->OneG + ICM20602->OneG;//拉力输入，遥控器1500pwm对应1g拉力
		  Gyro_Control(CtrlIO,CtrlFbck);    //陀螺力矩控制
		  break;
		case 1://半自控
		  CtrlLpIO->  roll_command[0] = CtrlIO->input[0]*0.05f*D2R;//滚转角度输入0.05对应20°
			CtrlLpIO-> pitch_command[0] = CtrlIO->input[1]*0.05f*D2R;//俯仰角度输入0.05对应20°
		  CtrlLpIO->thrust_command[0] = (CtrlIO->input[3]-1500)/(1500-1090)*ICM20602->OneG + ICM20602->OneG;//拉力输入，遥控器1500pwm对应1g拉力
		  CtrlLpIO->AngleLimitR       = 20*D2R;//滚转角限幅
			CtrlLpIO->AngleLimitP       = 20*D2R;//俯仰角限幅
		  In_Loop_Step(CtrlLpIO);              //角度环
		  Gyro_Control(CtrlIO,CtrlFbck);       //陀螺力矩控制
		  break;
		case 2://速度操纵
		  CtrlLpIO->X_command[1] =  -CtrlIO->input[1]*0.0125f;//机头坐标横向速度输入0.0125对应5m/s
		  CtrlLpIO->Y_command[1] =  CtrlIO->input[0]*0.0125f;//机头坐标纵向速度输入0.0125对应5m/s
		  Out_Loop_Z_Pre(CtrlLpIO);            //高度控制
      Out_Loop_Step(CtrlLpIO);             //速度控制
			Underactuated_Loop_Step(CtrlLpIO);   //欠驱动环（欠驱动解算，拉力补偿）
		  CtrlLpIO->AngleLimitR       = 20*D2R;//滚转角限幅
			CtrlLpIO->AngleLimitP       = 20*D2R;//俯仰角限幅
	    In_Loop_Step(CtrlLpIO);              //角度环
		  Gyro_Control(CtrlIO,CtrlFbck);       //陀螺力矩控制		    
		  break;
		case 3://自控
			Out_Loop_Z_Pre(CtrlLpIO);            //高度控制
      Out_Loop_Step(CtrlLpIO);             //速度控制
			Underactuated_Loop_Step(CtrlLpIO);   //欠驱动环（欠驱动解算，拉力补偿）
	    In_Loop_Step(CtrlLpIO);              //角度环
		  Gyro_Control(CtrlIO,CtrlFbck);       //陀螺力矩控制		
			break;
		case 4://失控保护
		  CtrlLpIO-> roll_command[0] = 0;      //滚转角度输入0
		  CtrlLpIO->pitch_command[0] = 0;      //俯仰角度输入0
		  In_Loop_Step(CtrlLpIO);              //角度环
		  Gyro_Control(CtrlIO,CtrlFbck);       //陀螺力矩控制
		default:break;
	}
	Control_Output(CtrlIO);           //角速度环
	Output_To_Motor(CtrlIO);          //控制输出到执行器
	Integral_Restore(CtrlKiTemp);
	//---------------------------------------//
}

