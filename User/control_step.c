#include "control.h"
#include "control_step.h"
#include "math.h" 

void Control_Step(sIO *ele)
{   
	Control_Feedback(CtrlFbck);       //���Ʋ�������
//	ControlRC_Check(CtrlIO);          //���ң�����ź�
	Integral_Reset(CtrlKiTemp);       //�����û���
	//--------------�����ϴ�ģʽ��Ϣ---------//
	
	ele->last_control_mode = ele->control_mode;
	ele->last_yaw_mode     = ele->yaw_mode    ;
	ele->last_fly_mode     = ele->fly_mode    ;
	
	//----------------ҡ���ź�---------------//
	
	ele->input[0] = -rc->Val[0];
	ele->input[1] = -rc->Val[1];
	ele->input[2] =  rc->Val[2];
	ele->input[3] =  rc->Val[3];
	
	//----------------����ģʽ---------------//
	
	if      (rc->Key[3]==0 )
	{
		ele->fly_mode=0;//ֱ����ģʽ
	}
	else if (rc->Key[3]==2 )
	{
		ele->fly_mode=1;//�̶���ģʽ
		if (ele->last_control_mode==0)
		{
		  Transition_Init(CtrlLpIO);	
		}
//Transition_Calc(CtrlLpIO);//��ת�滮
	}	
	
	//----------------��βģʽ---------------//
	
	if      (rc->Key[0]==0)   ele->yaw_mode=0;//�����β
	else if (rc->Key[0]==2)   ele->yaw_mode=1;//��β������ֱ����ģʽ
	
	//----------------����ģʽ---------------//
	
	if (CtrlIO->rc_status!=LOST)
	{
		switch(rc->Key[2])
		{
			case 0:
				ele->control_mode = 0;	//�ֿ�
				break;
			case 1:
				ele->control_mode=1;	//���Կ�
				break;
			case 2:
				if (ele->fly_mode==0)		
				{
					Out_Loop_XY_Pre(CtrlLpIO); //�ж϶���ģʽ��ɲ��ģʽ
				}
				if (CtrlLpIO->position_mode==false && ele->fly_mode==0) ele->control_mode = 2;			//�ٶȲ���
				else                                                    ele->control_mode = 3;			//�Կ�
				break;
			default:break;
		}
	}
	else
	{
		ele->control_mode = 4;	//ʧ�ر���
	}
	
	//----------------�����߼�----------------//
	
	switch(ele->control_mode)
	{
		case 0://�ֿ�
			In_Loop_Step(CtrlLpIO);              //�ǶȻ�(��β)
		  CtrlLpIO->   pqr_command[0] = CtrlIO->input[0]*0.225f*D2R;//��ת���ٶ�����0.225��Ӧ90��/s
		  CtrlLpIO->   pqr_command[1] = CtrlIO->input[1]*0.225f*D2R;//�������ٶ�����0.225��Ӧ90��/s	
		  CtrlLpIO->thrust_command[0] = (CtrlIO->input[3]-1500)/(1500-1090)*ICM20602->OneG + ICM20602->OneG;//�������룬ң����1500pwm��Ӧ1g����
		  Gyro_Control(CtrlIO,CtrlFbck);    //�������ؿ���
		  break;
		case 1://���Կ�
		  CtrlLpIO->  roll_command[0] = CtrlIO->input[0]*0.05f*D2R;//��ת�Ƕ�����0.05��Ӧ20��
			CtrlLpIO-> pitch_command[0] = CtrlIO->input[1]*0.05f*D2R;//�����Ƕ�����0.05��Ӧ20��
		  CtrlLpIO->thrust_command[0] = (CtrlIO->input[3]-1500)/(1500-1090)*ICM20602->OneG + ICM20602->OneG;//�������룬ң����1500pwm��Ӧ1g����
		  CtrlLpIO->AngleLimitR       = 20*D2R;//��ת���޷�
			CtrlLpIO->AngleLimitP       = 20*D2R;//�������޷�
		  In_Loop_Step(CtrlLpIO);              //�ǶȻ�
		  Gyro_Control(CtrlIO,CtrlFbck);       //�������ؿ���
		  break;
		case 2://�ٶȲ���
		  CtrlLpIO->X_command[1] =  -CtrlIO->input[1]*0.0125f;//��ͷ��������ٶ�����0.0125��Ӧ5m/s
		  CtrlLpIO->Y_command[1] =  CtrlIO->input[0]*0.0125f;//��ͷ���������ٶ�����0.0125��Ӧ5m/s
		  Out_Loop_Z_Pre(CtrlLpIO);            //�߶ȿ���
      Out_Loop_Step(CtrlLpIO);             //�ٶȿ���
			Underactuated_Loop_Step(CtrlLpIO);   //Ƿ��������Ƿ�������㣬����������
		  CtrlLpIO->AngleLimitR       = 20*D2R;//��ת���޷�
			CtrlLpIO->AngleLimitP       = 20*D2R;//�������޷�
	    In_Loop_Step(CtrlLpIO);              //�ǶȻ�
		  Gyro_Control(CtrlIO,CtrlFbck);       //�������ؿ���		    
		  break;
		case 3://�Կ�
			Out_Loop_Z_Pre(CtrlLpIO);            //�߶ȿ���
      Out_Loop_Step(CtrlLpIO);             //�ٶȿ���
			Underactuated_Loop_Step(CtrlLpIO);   //Ƿ��������Ƿ�������㣬����������
	    In_Loop_Step(CtrlLpIO);              //�ǶȻ�
		  Gyro_Control(CtrlIO,CtrlFbck);       //�������ؿ���		
			break;
		case 4://ʧ�ر���
		  CtrlLpIO-> roll_command[0] = 0;      //��ת�Ƕ�����0
		  CtrlLpIO->pitch_command[0] = 0;      //�����Ƕ�����0
		  In_Loop_Step(CtrlLpIO);              //�ǶȻ�
		  Gyro_Control(CtrlIO,CtrlFbck);       //�������ؿ���
		default:break;
	}
	Control_Output(CtrlIO);           //���ٶȻ�
	Output_To_Motor(CtrlIO);          //���������ִ����
	Integral_Restore(CtrlKiTemp);
	//---------------------------------------//
}

