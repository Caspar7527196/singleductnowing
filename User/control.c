#include "control.h"
#include "math.h" 
//---------------------------------------------by mch
#include "wls_alloc_mch.h"
#include "dir_alloc_mch.h"
#include "gps_ins_EKF.h"
//----------------------------------------------------
/***************************************************\
功能：
  主控制器

  
\***************************************************/

sFEEDBACK CtrlFbck[CTRLFB_NUM];
sIO CtrlIO[CTRLIO_NUM];
sLOOPIO CtrlLpIO[CTRLIL_NUM];
sKITEMP CtrlKiTemp[CTRLKT_NUM];
sMRAC2 CtrlMRAC2[CTRLKT_NUM];

//static double sin_num = 0;
/***************************************************\
  包含各控制算法所用函数定义
\***************************************************/
void Control_Init(void)
{
	CtrlIO->control_mode=0;
	CtrlIO->fly_mode=0;
	CtrlIO->yaw_mode=0;
	CtrlIO->gyro_gain=50.88f;
	CtrlIO->cs_output[4]=1200;
	CtrlIO->mid_trim[0]=0.0f;
	CtrlIO->mid_trim[1]=0.0f;
	CtrlIO->mid_trim[2]=-20.0f;
	CtrlKiTemp->int_reset=true;
	CtrlFbck->engine_speed=0.0f;
	CtrlLpIO->position_mode=false;
	CtrlLpIO->brake_mode=false;
	CtrlIO->InterfereFlag=0;
	CtrlLpIO->AngleLimitR=20*D2R;
	CtrlLpIO->AngleLimitP=20*D2R;

	//-----------------------------------------------by mch
	dir_alloc_mch_initialize();
	wls_alloc_mch_initialize();
	//----------------------------------------------------------
	CtrlIO->rc_cnt = 0;
	CtrlIO->rc_check = 0;
	CtrlIO->ever_rc = 0;
	CtrlIO->rc_status=NORMAL;
	
	CtrlLpIO->VelSet=0.0;
	
	CtrlLpIO->w1=0.0f;
	CtrlLpIO->wp=0.0f;
	CtrlLpIO->wr=0.0f;
	CtrlLpIO->k1=100.0f;
	CtrlLpIO->ko1=20.0f;
	CtrlLpIO->ko2=0.0f;
	CtrlLpIO->ko3=0.0f;
	CtrlLpIO->Vn_Pred[0]=0.0f;
	CtrlLpIO->Vn_Pred[1]=0.0f;
	CtrlLpIO->Vn_Pred[2]=0.0f;
	
	for(int i=0; i<RC_CHECK_NUM; i++)
	{
		CtrlIO->rc_buffer[i] = 0;
	}
	
	CtrlFbck->Z_offset = 0;
}

void Control_Feedback(sFEEDBACK *ele)
{
	ele->AirSpeed_Last   = ele->AirSpeed;
//	int modenum=0;
//	if(rc->Key[3] == 0)
//	{
//		modenum=0;//激光
//		ele->Z[0]=ahrs[0].Xyz[2];
//		ele->Z[1]=ahrs[0].Uvw[2];
//	  ele->W[0]=ele->Z[1];
//	}
//	else 
//	{
//		modenum=1;//卡尔曼
//		ele->Z[0]=ahrs[0].Xyz[2];
//		ele->Z[1]=ahrs[0].Uvw[2];
		ele->Z[0]=gps->NED[2] - ele->Z_offset;
		ele->Z[1]=output_data_new.Vel[2];
//	}
	for (u8 i=0;i<3;i++)
	{
		ele->pqr[i]=ICM20602->GyrFil_2nd[i];
		ele->Ang[i]=ahrs[1].Ang[i];
	}
	//----------北东地位置----------------------
	ele->X[0]=gps->NED[0];
	ele->Y[0]=gps->NED[1];
//	ele->Z[0]=ahrs[modenum].Xyz[2];
//	ele->X[0]=ahrs[0].Xyz[0];
//	ele->Y[0]=ahrs[0].Xyz[1];
	
	//----------北东地速度---------------------
	ele->X[1]=output_data_new.Vel[0];
	ele->Y[1]=output_data_new.Vel[1];
//	ele->Z[1]=ahrs[modenum].Uvw[2];
//	ele->X[1]=ahrs[0].Uvw[0];
//	ele->Y[1]=ahrs[0].Uvw[1];
	
	//----------机头坐标系速度---------------------
//	ele->XH[1]=ahrs[1].UvwH[0];
//	ele->YH[1]=ahrs[1].UvwH[1];
	
	
	float sinY,cosY;
	arm_sin_cos_f32(ahrs[1].Ang[2]*R2D,&sinY,&cosY);
	ele->XH[1] =  ele->X[1]*cosY + ele->Y[1]*sinY;
	ele->YH[1] = -ele->X[1]*sinY + ele->Y[1]*cosY;
//	ele->X[0]=ahrs[modenum].Acc[0];ele->Y[0]=ahrs[modenum].Acc[1];ele->Z[0]=ahrs[modenum].Acc[2];
//	ele->X[1]=ahrs[modenum].Uvw[0];ele->Y[1]=ahrs[modenum].Uvw[1];ele->Z[1]=ahrs[modenum].Uvw[2];
//	ele->XH[0]=ahrs[modenum].AccH[0];ele->YH[0]=ahrs[modenum].AccH[1];
//	ele->XH[1]=ahrs[modenum].UvwH[0];ele->YH[1]=ahrs[modenum].UvwH[1];
//	ele->U[0]=ahrs[modenum].AccB[0];ele->V[0]=ahrs[modenum].AccB[1];ele->W[0]=ahrs[modenum].AccB[2];
	ele->U[0] = ele->X[1]*ahrs[1].Cb2n[0][0]+ele->Y[1]*ahrs[1].Cb2n[1][0]+ele->Z[1]*ahrs[1].Cb2n[2][0];
	ele->V[0] = ele->X[1]*ahrs[1].Cb2n[0][1]+ele->Y[1]*ahrs[1].Cb2n[1][1]+ele->Z[1]*ahrs[1].Cb2n[2][1];
	ele->W[0] = ele->X[1]*ahrs[1].Cb2n[0][2]+ele->Y[1]*ahrs[1].Cb2n[1][2]+ele->Z[1]*ahrs[1].Cb2n[2][2];
	
	ele->AirSpeed = sqrt(CtrlFbck->X[1]*CtrlFbck->X[1]+CtrlFbck->Y[1]*CtrlFbck->Y[1]+CtrlFbck->Z[1]*CtrlFbck->Z[1]);

	ele->engine_speed=exitx->DatFil[0]*2.0f*PI/7.0f;;//原始数据rps（电气角），转换为rad/s（机械角），5为电机极对数
	if (CtrlIO->control_mode==2 || CtrlIO->control_mode==3)
	{
		ele->engine_speed=fConstrain(ele->engine_speed,800,2000);
	}
}

void ControlRC_Check(sIO *ele)
{
	static u16 new_index=0,old_index=1;
	
	ele->rc_buffer[new_index] = rc->RxFlag;
	ele->rc_check = ele->rc_check + ele->rc_buffer[new_index] - ele->rc_buffer[old_index];
	new_index = (new_index + 1)%RC_CHECK_NUM;
	old_index = (old_index + 1)%RC_CHECK_NUM;
	
	if(ele->rc_check != 0)
	{
		ele->rc_status=NORMAL;
		ele->ever_rc = 1;
	}
	else if(ele->ever_rc == 0 || ele->input[3] <= 1200)
	{
		ele->rc_status=CLOSE;
	}
	else
	{
		ele->rc_status=LOST;
	}
}

void Integral_Reset(sKITEMP *ele)
{
	for (int i=0;i<CTRLPID_NUM;i++)
	{
		ele->pid_Ki_temp[i]=pid[i].Ki;//暂存积分系数
	}
	/********************起降阶段内环暂停积分***************/
	if (ele->int_reset==true)
	{
		for (int i=0;i<CTRLPID_NUM;i++)
		{
			pid[i].Ki=0.0f;
			pid[i].integral=0.0f;
		}
		if (CtrlFbck->engine_speed>(HOVER_SPEED-100.0f))//转速高于悬停转速-50，重置并开始积分
		{
			ele->int_reset=false;
			CtrlFbck->Z_offset = gps->NED[2];
		}
	}
	else if (ele->int_reset==false)
	{
		
		for (int i=0;i<CTRLPID_NUM;i++)
		{
			pid[i].Ki=ele->pid_Ki_temp[i];
		}
		if (CtrlFbck->engine_speed<100.0f)//转速低于100，停止积分
		{
			ele->int_reset=true;
		}
		switch(CtrlIO->control_mode)
		{
			case 0://手控，姿态环积分系数置零
        for (u8 i=3;i<12;i++)
			  {
					pid[i].Ki=0.0f;
					pid[i].integral=0.0f;
			  }
			break;
			case 1://半自控
				for (u8 i=6;i<12;i++)
			  {
					pid[i].Ki=0.0f;
					pid[i].integral=0.0f;
				}
			break;
			case 2://自控
			break;
			case 3://自控
			break;
			default:break;	
		}
	}
}

void Integral_Restore(sKITEMP *ele)
{
	for (int i=0;i<CTRLPID_NUM;i++)
	{
	  pid[i].Ki=ele->pid_Ki_temp[i];
	}
}

void Out_Loop_XY_Pre(sLOOPIO *ele)
{
	/***************************************************\
	功能：
		自控模式下外环预处理
		1.遥控器回中对应定点模式
	  2.遥控器有动作对应输入模式（速度输入）
		3.遥控器由输入模式转为定点模式，采用刹车模式衔接，先速度降为0，以当前点为位置输入做定点控制
	\***************************************************/
	double X_err,Y_err;
  bool   X_command_flag,Y_command_flag;
	float  Vel_tol=0.2;
	int    brake_end=20;
	//----------------------检测打杆与否--------------------------------------
	if (CtrlIO->input[1]<10 && CtrlIO->input[1]>-10) X_command_flag = true;
	else  	                                         X_command_flag = false;
	if (CtrlIO->input[0]<10 && CtrlIO->input[0]>-10) Y_command_flag = true;
	else                                             Y_command_flag = false;
	//-----------------------水平速度-----------------------------------------
	if (ele->position_mode==false)
	{
		if (X_command_flag==true && Y_command_flag==true)//小杆量，不打杆
		{
			ele->position_mode = true;
			ele->brake_mode    = true;
			ele->brake_cnt     = 0;
		}
	}
	if (ele->position_mode==true)
	{
		if (X_command_flag==false || Y_command_flag==false)//打杆
		{
			ele->position_mode = false;
		}
		if (ele->brake_mode == true)
		{
			ele->X_command[1] = 0;
			ele->Y_command[1] = 0;
			if (Abs(ele->X_command[1]-CtrlFbck->XH[1])<Vel_tol && Abs(ele->Y_command[1]-CtrlFbck->YH[1])<Vel_tol)
			{
				ele->brake_cnt++;
			}
			if (ele->brake_cnt==brake_end)
			{
				ele->brake_mode    = false;
				ele->X_command[0]  = CtrlFbck->X[0];//当前位置作为位置给定
				ele->Y_command[0]  = CtrlFbck->Y[0];
				pid[9].integral    = 0.0;
				pid[10].integral   = 0.0;
			}						
		}
		else
		{
			//位置误差转到机头坐标误差
			X_err = (ele->X_command[0]-CtrlFbck->X[0])* cos(CtrlFbck->Ang[2]) + (ele->Y_command[0]-CtrlFbck->Y[0])*sin(CtrlFbck->Ang[2]);
			Y_err = (ele->X_command[0]-CtrlFbck->X[0])*-sin(CtrlFbck->Ang[2]) + (ele->Y_command[0]-CtrlFbck->Y[0])*cos(CtrlFbck->Ang[2]);
			ele->X_command[1] = Pid_Controller(pidX,X_err);
			ele->Y_command[1] = Pid_Controller(pidY,Y_err);
		}
	}
}
void Out_Loop_Z_Pre(sLOOPIO *ele)
{
	double Z_err;
	if (CtrlIO->last_control_mode!=2 && CtrlIO->last_control_mode!=3)  //模式切换瞬态初始化
	{
		ele->Z_command[0] = CtrlFbck->Z[0];
		ele->Z_mid        = CtrlFbck->Z[0];
//			ele->Z_mid = 0.0f;
		ele->Z_mid_pwm    = CtrlIO->input[3];
		ele->w1=0.0f;
		ele->wr=0.0f;
		ele->wp=0.0f;
		ele->Vn_Pred[0]=CtrlFbck->XH[1];
		ele->Vn_Pred[1]=CtrlFbck->YH[1];
		ele->Vn_Pred[2]=CtrlFbck->Z[1];
	}
	ele->Z_command[0] = ele->Z_mid - (CtrlIO->input[3]-ele->Z_mid_pwm)*0.005f;
	Z_err             = ele->Z_command[0]-CtrlFbck->Z[0];		
	ele->Z_command[1] = Pid_Controller(pidZ,Z_err);	
	//		ele->Z_command[1]= ele->Z_mid - (CtrlIO->input[3]-ele->Z_mid_pwm)*0.005f;
	//----------------------------------------
	if (CtrlIO->fly_mode==0)//过渡完毕重新设置定高中点
	{
		if (CtrlIO->last_fly_mode==1)
		{
			ele->Z_mid        = CtrlFbck->Z[0];
		}
	}
}
void Out_Loop_Step(sLOOPIO *ele)
{
	double Vel_err[3];
	//---------------------速度输入限幅----------------------//
	
	ele->X_command[1] = fConstrain(ele->X_command[1],-5,5);
	ele->Y_command[1] = fConstrain(ele->Y_command[1],-5,5);
	ele->Z_command[1] = fConstrain(ele->Z_command[1],-5,5);
	
	//-----------------------速度误差------------------------//
	
	Vel_err[0] = ele->X_command[1]-CtrlFbck->XH[1];  //X_command[1]机头速度
	Vel_err[1] = ele->Y_command[1]-CtrlFbck->YH[1];
	Vel_err[2] = ele->Z_command[1]-CtrlFbck->Z[1];
	
	//---------------------加速度指令，NED-------------------//
	
	ele->acc_command[0] = Pid_Controller(pidXRate,Vel_err[0]) ;//加速度指令,NED
	ele->acc_command[1] = Pid_Controller(pidYRate,Vel_err[1]) ;
	ele->acc_command[2] = Pid_Controller(pidZRate,Vel_err[2]) ;
	
  //-------------------------------------------------------//
	ele->VelNEDH_Last[0] = CtrlFbck->XH[1];
	ele->VelNEDH_Last[1] = CtrlFbck->YH[1];
	ele->VelNEDH_Last[2] = CtrlFbck->Z[1];
}
void Underactuated_Loop_Step(sLOOPIO *ele)
{
	//-----------------------加速度限幅----------------------//
	ele->acc_command[0] = fConstrain(ele->acc_command[0],-3.5,3.5);
	ele->acc_command[1] = fConstrain(ele->acc_command[1],-3.5,3.5);
	ele->acc_command[2] = fConstrain(ele->acc_command[2],-ICM20602->OneG,ICM20602->OneG);
	
  float ad = ele->acc_command[2] - ICM20602->OneG;
	float bd = ele->acc_command[1];
	float cd = ele->acc_command[0];
//	float ax = -ICM20602->AccFil_3nd[0];
//	float ay = -ICM20602->AccFil_3nd[1];
//	float az = -ICM20602->AccFil_3nd[2];
//	float ax = ahrs[0].Acc_Fil_buffer[0][ahrs[0].acc_buffer_cnt];
//	float ay = ahrs[0].Acc_Fil_buffer[1][ahrs[0].acc_buffer_cnt];
//	float az = ahrs[0].Acc_Fil_buffer[2][ahrs[0].acc_buffer_cnt];	
	float ax = -ICM20602->AccFil[0];
	float ay = -ICM20602->AccFil[1];
	float az = -ICM20602->AccFil[2];
	float eta1,eta2;
	float cosPd,sinPd;
	float cosRd,sinRd;
	float g13_bar,g23_bar,g33_bar;
	float l1r,l2r,l3r,l1p,l2p,l3p;
	float phi1,phir,phip;
	float Rc,Pc;
	float g11_tilt,g12_tilt,g13_tilt,g21_tilt,g22_tilt,g23_tilt,g31_tilt,g32_tilt,g33_tilt;
	float e1,e2,e3;
	float Vx_est_D,Vy_est_D,Vz_est_D;
	float W1_D,Wr_D,Wp_D;
	//nominal system
	eta1           = atan2(cd,-ad);
  ele->Pd_Bar    = asin(fConstrain(ax/(sqrt(ad*ad+cd*cd)),-1,1))-eta1;
  //ele->Pd_Bar=-eta1;
  cosPd          = cos(ele->Pd_Bar);
	sinPd          = sin(ele->Pd_Bar);
  eta2           = atan2(-bd,-(ad*cosPd+cd*sinPd));
  ele->Rd_Bar    = asin(fConstrain(-ay/(sqrt((ad*cosPd+cd*sinPd)*(ad*cosPd+cd*sinPd)+bd*bd)),-1,1))-eta2;
  //ele->Rd_Bar=-eta2;
	cosRd          = cos(ele->Rd_Bar);
	sinRd          = sin(ele->Rd_Bar);
  ele->u1_Bar[0] = cd*sinPd*cosRd-bd*sinRd+ad*cosPd*cosRd;
	//varibles update
	g13_bar = sin(ele->Pd_Bar)*cos(ele->Rd_Bar);
	g23_bar =-sin(ele->Rd_Bar);
	g33_bar = cos(ele->Pd_Bar)*cos(ele->Rd_Bar);
	l1r     = sin(ele->Pd_Bar)*cos(ele->Rd_Bar)*ay-sin(ele->Pd_Bar)*sin(ele->Rd_Bar)*az;
	l2r     =-sin(ele->Rd_Bar)*ay-cos(ele->Rd_Bar)*az;
	l3r     = cos(ele->Pd_Bar)*cos(ele->Rd_Bar)*ay-cos(ele->Pd_Bar)*sin(ele->Rd_Bar)*az;
	l1p     =-sin(ele->Pd_Bar)*ax+cos(ele->Pd_Bar)*sin(ele->Rd_Bar)*ay+cos(ele->Pd_Bar)*cos(ele->Rd_Bar)*az;
	l2p     = 0;
	l3p     =-cos(ele->Pd_Bar)*ax-sin(ele->Pd_Bar)*sin(ele->Rd_Bar)*ay-sin(ele->Pd_Bar)*cos(ele->Rd_Bar)*az;
	phi1    = 1.0f;
	phir    = 1.0f;
	phip    = 1.0f;
	Rc       = ele->Rd_tilt+ele->wr*phir;
	Pc       = ele->Pd_Tilt+ele->wp*phip;
	g11_tilt =-sin(ele->Pd_Bar)*Pc;  
	g12_tilt = sin(ele->Pd_Bar)*cos(ele->Rd_Bar)*Rc+cos(ele->Pd_Bar)*sin(ele->Rd_Bar)*Pc; 
	g13_tilt =-sin(ele->Pd_Bar)*sin(ele->Rd_Bar)*Rc+cos(ele->Pd_Bar)*cos(ele->Rd_Bar)*Pc;
	g21_tilt = 0.0f;                
	g22_tilt =-sin(ele->Rd_Bar)*Rc;                                       
	g23_tilt =-cos(ele->Rd_Bar)*Rc;
	g31_tilt =-cos(ele->Pd_Bar)*Pc;  
	g32_tilt = cos(ele->Pd_Bar)*cos(ele->Rd_Bar)*Rc-sin(ele->Pd_Bar)*sin(ele->Rd_Bar)*Pc; 
	g33_tilt =-cos(ele->Pd_Bar)*sin(ele->Rd_Bar)*Rc-sin(ele->Pd_Bar)*cos(ele->Rd_Bar)*Pc;
	e1       = g11_tilt*ax+g12_tilt*ay+g13_tilt*az+g13_bar*(ele->u1_Tilt[0]+ele->w1*phi1);
	e2       = g21_tilt*ax+g22_tilt*ay+g23_tilt*az+g23_bar*(ele->u1_Tilt[0]+ele->w1*phi1);
	e3       = g31_tilt*ax+g32_tilt*ay+g33_tilt*az+g33_bar*(ele->u1_Tilt[0]+ele->w1*phi1);
	//auxilary system
	Vx_est_D        = ele->acc_command[0]+e1-ele->k1*(ele->Vn_Pred[0]-CtrlFbck->XH[1]);
	Vy_est_D        = ele->acc_command[1]+e2-ele->k1*(ele->Vn_Pred[1]-CtrlFbck->YH[1]);
	Vz_est_D        = ele->acc_command[2]+e3-ele->k1*(ele->Vn_Pred[2]-CtrlFbck->Z[1]);
	W1_D            =-ele->ko1*phi1*(g13_bar*(ele->Vn_Pred[0]-CtrlFbck->XH[1])+g23_bar*(ele->Vn_Pred[1]-CtrlFbck->YH[1])+g33_bar*(ele->Vn_Pred[2]-CtrlFbck->Z[1]));
	Wr_D            =-ele->ko2*phir*(l1r*(ele->Vn_Pred[0]-CtrlFbck->XH[1])+l2r*(ele->Vn_Pred[1]-CtrlFbck->YH[1])+l3r*(ele->Vn_Pred[2]-CtrlFbck->Z[1]));
	Wp_D            =-ele->ko3*phip*(l1p*(ele->Vn_Pred[0]-CtrlFbck->XH[1])+l2p*(ele->Vn_Pred[1]-CtrlFbck->YH[1])+l3p*(ele->Vn_Pred[2]-CtrlFbck->Z[1]));
	ele->w1         = ele->w1+W1_D*CtrlDt;
	ele->wr         = ele->wr+Wr_D*CtrlDt;
	ele->wp         = ele->wp+Wp_D*CtrlDt;
	ele->Vn_Pred[0] = ele->Vn_Pred[0]+Vx_est_D*CtrlDt;
	ele->Vn_Pred[1] = ele->Vn_Pred[1]+Vy_est_D*CtrlDt;
	ele->Vn_Pred[2] = ele->Vn_Pred[2]+Vz_est_D*CtrlDt;
	ele->u1_Tilt[0] =-ele->w1*phi1;
  ele->Rd_tilt    =-ele->wr*phir;
  ele->Pd_Tilt    =-ele->wp*phip;
	//-----------------------------------------------------------
	ele->thrust_command[0] = -(ele->u1_Bar[0]+ele->u1_Tilt[0]);
	ele-> roll_command[0] = ele->Rd_Bar+ele->Rd_tilt;
	ele->pitch_command[0] = ele->Pd_Bar+ele->Pd_Tilt;
	ele->thrust_command[0] = fConstrain(ele->thrust_command[0],6,20);
}
void In_Loop_Step(sLOOPIO *ele)
{
	//----------------滚转俯仰角输入限幅----------------//
	ele-> roll_command[0] = fConstrain(ele-> roll_command[0],-ele->AngleLimitR,ele->AngleLimitR);
	ele->pitch_command[0] = fConstrain(ele->pitch_command[0],-ele->AngleLimitP,ele->AngleLimitP);
	if (CtrlIO->last_yaw_mode==0 && CtrlIO->yaw_mode==1)
	{
		ele->yaw_mid = CtrlFbck->Ang[2];
	}
	//----------------滚转俯仰角变化率输入----------------
	ele->roll_err         = ele->roll_command[0] -CtrlFbck->Ang[0];
	ele->pitch_err        = ele->pitch_command[0]-CtrlFbck->Ang[1];
	ele->roll_command[1]  = Pid_Controller(pidRol,ele->roll_err );
	ele->pitch_command[1] = Pid_Controller(pidPit,ele->pitch_err);
	//--------------------偏航控制--------------------
	if (CtrlIO->yaw_mode==1 && gps->MagUpdate==true)//磁力计正常更新才允许锁尾
	{
//		if(rc->Key[3] == 0)
//    {
			ele->yaw_command[0] = ele->yaw_mid + CtrlIO->input[2]*0.225f*D2R;
//		}
//		else
//		{
//			ele->yaw_command[0]=ele->yaw_mid-0.2;
//		}
		ele->yaw_err = (ele->yaw_command[0]-CtrlFbck->Ang[2]);
		//----------------消除-180°到+180°的跳变----------------
		if      (ele->yaw_err<-PI)  ele->yaw_err+=2*PI;
		else if (ele->yaw_err>PI)   ele->yaw_err-=2*PI;
		//------------------------------------------------------
		ele->yaw_command[1] = Pid_Controller(pidYaw,ele->yaw_err);
	}
	
	//----------------欧拉角变化率到pqr----------------
	ele->pqr_command[0]=ele->roll_command[1]-ele->yaw_command[1]*sin(CtrlFbck->Ang[1]);
	ele->pqr_command[1]=ele->pitch_command[1]*cos(CtrlFbck->Ang[0])+ele->yaw_command[1]*cos(CtrlFbck->Ang[1])*sin(CtrlFbck->Ang[0]);
	ele->pqr_command[2]=-ele->pitch_command[1]*sin(CtrlFbck->Ang[0])+ele->yaw_command[1]*cos(CtrlFbck->Ang[1])*cos(CtrlFbck->Ang[0]);
	if (CtrlIO->yaw_mode==0)
	{
		ele->pqr_command[2] = CtrlIO->input[2]*0.225f*D2R;//不锁尾，遥控器输入为航向角速度指令
	}
}


void Gyro_Control(sIO *ele, sFEEDBACK *fb)
{
		ele->gyro_output[0] = -ele->gyro_gain*fb->pqr[1]/fConstrain(CtrlFbck->engine_speed,800.0f,2000.0f);
	  ele->gyro_output[1] = +ele->gyro_gain*fb->pqr[0]/fConstrain(CtrlFbck->engine_speed,800.0f,2000.0f);
		ele->gyro_output[2] = 0.0f;	
}

void Control_Output(sIO *ele)
{
	for (u8 i=0;i<3;i++)
	{
		ele->pqr_err[i]=CtrlLpIO->pqr_command[i]-CtrlFbck->pqr[i];
	}
	if (ele->control_mode!=2)
	{
		if (ele->last_control_mode==2)
		{
			for (u8 i=0;i<3;i++)
			{
				pid[i].integral=0;//由自控切回半自控重置内环积分
			}
		}
		ele->output_pid[0]=Pid_Controller(pidRolRate,ele->pqr_err[0]);
		ele->output_pid[1]=Pid_Controller(pidPitRate,ele->pqr_err[1]);
		ele->output_pid[2]=Pid_Controller(pidYawRate,ele->pqr_err[2]);
		ele->output1[0]=pid[0].iout;
		ele->output1[1]=pid[1].iout;
		ele->output1[2]=pid[2].iout;
	}
	else//自控模式下开启内环MRAC
	{
	  if (ele->last_control_mode!=2)
		{
			MRAC_Init2(CtrlMRAC2);//模式切换瞬态初始化
		}
		MRAC_Calc2(CtrlMRAC2,CtrlIO->pqr_err);
		ele->output_pid[0]=CtrlMRAC2->uc2[0];
		ele->output_pid[1]=CtrlMRAC2->uc2[1];
		ele->output_pid[2]=CtrlMRAC2->uc2[2];
		ele->output1[0]=CtrlMRAC2->uc2_Tilt[0];
		ele->output1[1]=CtrlMRAC2->uc2_Tilt[1];
		ele->output1[2]=CtrlMRAC2->uc2_Tilt[2];
//	  ele->output_pid[0]=Pid_Controller(pidRolRate,ele->pqr_err[0]);
//		ele->output_pid[1]=Pid_Controller(pidPitRate,ele->pqr_err[1]);
//		ele->output_pid[2]=Pid_Controller(pidYawRate,ele->pqr_err[2]);
//		ele->output1[0]=pid[0].iout;
//		ele->output1[1]=pid[1].iout;
//		ele->output1[2]=pid[2].iout;
	}
	//------------------------三轴力距输出-----------------------------
		ele->output[0]=(ele->mid_trim[0]/((double)RAD_TO_PWM))+ele->output_pid[0]+ele->gyro_output[0];
	  ele->output[1]=(ele->mid_trim[1]/((double)RAD_TO_PWM))+ele->output_pid[1]+ele->gyro_output[1];
	  ele->output[2]=(ele->mid_trim[2]/((double)RAD_TO_PWM))+ele->output_pid[2]+ele->gyro_output[2];
	  ele->output2[0]=ele->output[0]-ele->output1[0];
	  ele->output2[1]=ele->output[1]-ele->output1[1];
	  ele->output2[2]=ele->output[2]-ele->output1[2];
	//------------------------拉力输出-----------------------------------------------

		ele->output[3] = CtrlLpIO->thrust_command[0];

}


void Output_To_Motor(sIO *ele)  //by MCH2019-6
{	
	double V[3];
	for(int i=0;i<3;++i)
	{
		V[i]=ele->output[i]+eso->y[i];
	}
	//================================舵面分配======================================
	//---------------------直接分配--------------------------------------------
	//wls_alloc_mch(ele->output, wls.p_limits, 0, wls.u);//加权最小二乘法求控制分配，0.8ms
	//dir_alloc_mch(ele->output, dir.umin, dir.umax, dir.u);//保方向，线性规划
	//---------------------两次分配，先保证扰动补偿，再分配动态------------------------------
	two_dir_alloc_mch(ele->output1, ele->output2, dir.p_limits, dir.u);

			ele->cs_output[0] = (int) ((dir.u[0]) * RAD_TO_PWM);//务必添加括号，否则强制转换数据类型后输出错误
			ele->cs_output[1] = (int) ((dir.u[1]) * RAD_TO_PWM);//记得修改dir和wls
			ele->cs_output[2] = (int) ((dir.u[2]) * RAD_TO_PWM);
			ele->cs_output[3] = (int) ((dir.u[3]) * RAD_TO_PWM);


  //--------------------电机输出----------------------------------	
	double K_pwm = ICM20602->OneG/((HOVER_PWM-1090)*(HOVER_PWM-1090));
	if(ele->rc_status == NORMAL)
	{
		ele->mt_output[0] = (int)(sqrt((ele->output[3])/K_pwm))+1090;
//		ele->mt_output[0] =1090;
	}
	else if(ele->rc_status==CLOSE)
	{
		ele->mt_output[0] =1090;
	}
	else
	{
//		ele->mt_output[0] =1090;
		ele->mt_output[0] = DRIFT_PWM;
	}
}

void MRAC_Init2(sMRAC2 *ele)
{
	ele->k1          = 100.0f;
	ele->k2[0][0]    = 400.0f;
	ele->k2[1][0]    = 10.0f;
	ele->k2[2][0]    = 10.0f;
	ele->k2[0][1]    = 400.0f;
	ele->k2[1][1]    = 10.0f;
	ele->k2[2][1]    = 10.0f;
	ele->k2[0][2]    = 400.0f;
	ele->k2[1][2]    = 0.0001f;
	ele->k2[2][2]    = 0.0f;
	ele->k5[0][0]    = 0.0f;
	ele->k5[0][1]    = 0.0001;
	ele->k5[0][2]    = 0.0f;
	ele->k5[0][3]    = 0.0f;
	ele->k5[1][0]    = 0.0f;
	ele->k5[1][1]    = 0.0001;
	ele->k5[1][2]    = 0.0f;
	ele->k5[1][3]    = 0.0f;
	ele->k5[2][0]    = 0.0f;
	ele->k5[2][1]    = 0.000056;
	ele->k5[2][2]    = 0.0f;
	ele->k5[2][3]    = 0.0f;
	ele->kuc2[0]     = 0.1f;
	ele->kuc2[1]     = 0.1f;
	ele->kuc2[2]     = 0.1f;
	ele->w3[0][0]    = 0.0f;
	ele->w3[1][0]    = 0.0f;
	ele->w3[2][0]    = 0.0f;
	ele->w3[0][1]    = 0.0f;
	ele->w3[1][1]    = 0.0f;
	ele->w3[2][1]    = 0.0f;
	ele->w3[0][2]    = 0.0f;
	ele->w3[1][2]    = 0.0f;
	ele->w3[2][2]    = 0.0f;
	ele->w4[0]       = -1.0f;
	ele->w4[1]       = 0.01006;
	ele->w4[2]       = -0.53;
	ele->w4[3]       = 0.0001406731;
	ele->uc2_Tilt[0] = 0.0f;
	ele->uc2_Tilt[1] = 0.0f;
	ele->uc2_Tilt[2] = CtrlIO->mid_trim[2]/((double)RAD_TO_PWM);
	ele->uc2[0]      = 0.0f;
	ele->uc2[1]      = 0.0f;
	ele->uc2[2]      = ele->uc2_Tilt[2];
	ele->Ctrl_Flag[0]= 1;
	ele->Ctrl_Flag[1]= 1;
	ele->Ctrl_Flag[2]= 1;
}

void MRAC_Calc2(sMRAC2 *ele, double pqr_input[3])
{
	float DM[3];
	float g4_Bar;
	float c_m;
	double pqr_Pred_D[3];
	double w3_D[3][3];
	double w4_D[4];
	double k5tw4_D[3][4];
	double uc2_Tilt_D[3];
//	float pm[3];
	//reference model
  DM[0]  = DM1_MRAC;
	DM[1]  = DM2_MRAC;
	DM[2]  = DM3_MRAC;	
	g4_Bar = G4_BAR;
	c_m    = 20*D2R;
	//phi3
	ele->phi3[0][0] = 1.0f;
	ele->phi3[1][0] = CtrlFbck->YH[1];
	ele->phi3[2][0] = CtrlFbck->Z[1];
	ele->phi3[0][1] = 1.0f;
	ele->phi3[1][1] = CtrlFbck->XH[1];
	ele->phi3[2][1] = CtrlFbck->Z[1];
	ele->phi3[0][2] = 1.0f;
	ele->phi3[1][2] = CtrlFbck->engine_speed;
	ele->phi3[2][2] = 0;
	//phi4
	ele->phi4[0] = g4_Bar;
	ele->phi4[1] = -CtrlFbck->engine_speed*CtrlFbck->W[0];
	ele->phi4[2] = CtrlFbck->W[0]*CtrlFbck->W[0];
	ele->phi4[3] = CtrlFbck->engine_speed*CtrlFbck->engine_speed;
	//auxilary system
  for (u8 i=0;i<3;i++)
	{
	  ele->uc2_Bar[i] = fConstrain(pqr_input[i]*pid[i].Kp,-c_m,c_m);
		pqr_Pred_D[i]   =   (ele->w3[0][i]*ele->phi3[0][i] + ele->w3[1][i]*ele->phi3[1][i] + ele->w3[2][i]*ele->phi3[2][i])
		                  + g4_Bar*DM[i]*ele->uc2[i]
		                  + (ele->w4[0]*ele->phi4[0] + ele->w4[1]*ele->phi4[1] + ele->w4[2]*ele->phi4[2] +ele->w4[3]*ele->phi4[3])*DM[i]*ele->uc2[i]
		                  - ele->k1*(ele->pqr_Pred[i]-CtrlFbck->pqr[i]);
		for (u8 j=0;j<3;j++)
		{
			w3_D[j][i]    = -ele->k2[j][i]*ele->phi3[j][i]*(ele->pqr_Pred[i]-CtrlFbck->pqr[i]);
			ele->w3[j][i] =  ele->w3[j][i] + w3_D[j][i]*CtrlDt;
		}
		for (u8 k=0;k<4;k++)
		{
			k5tw4_D[i][k] = -ele->k5[i][k]*ele->phi4[k]*DM[i]*ele->uc2[i]*(ele->pqr_Pred[i]-CtrlFbck->pqr[i]);
		}
		ele->pqr_Pred[i] = ele->pqr_Pred[i] + pqr_Pred_D[i]*CtrlDt;
	}
	for (u8 i=0;i<4;i++)
	{
		w4_D[i]    = k5tw4_D[0][i] + k5tw4_D[1][i] + k5tw4_D[2][i];
		ele->w4[i] = ele->w4[i] + w4_D[i]*CtrlDt;
	}
	//control output
	for (u8 i=0;i<3;i++)
	{
		uc2_Tilt_D[i]    = -ele->kuc2[i]*
										   ((ele->w3[0][i]*ele->phi3[0][i] + ele->w3[1][i]*ele->phi3[1][i] + ele->w3[2][i]*ele->phi3[2][i])
										   +((g4_Bar + ele->w4[0]*ele->phi4[0] + ele->w4[1]*ele->phi4[1] + ele->w4[2]*ele->phi4[2] +ele->w4[3]*ele->phi4[3])*DM[i]*ele->uc2_Tilt[i])
										   +((ele->w4[0]*ele->phi4[0] + ele->w4[1]*ele->phi4[1] + ele->w4[2]*ele->phi4[2] +ele->w4[3]*ele->phi4[3])*DM[i]*ele->uc2_Bar[i]));
		ele->uc2_Tilt[i] = ele->uc2_Tilt[i] + uc2_Tilt_D[i]*CtrlDt;
		ele->uc2[i]      = ele->uc2_Bar[i] + ele->uc2_Tilt[i];
		ele->uc2[i]      = fConstrain(ele->uc2[i],-c_m,c_m);
		ele->g3_uc2[0]   = (ele->w3[0][i]*ele->phi3[0][i] + ele->w3[1][i]*ele->phi3[1][i] + ele->w3[2][i]*ele->phi3[2][i])/((g4_Bar + ele->w4[0]*ele->phi4[0] + ele->w4[1]*ele->phi4[1] + ele->w4[2]*ele->phi4[2] +ele->w4[3]*ele->phi4[3])*DM[i]);
		if (Abs(ele->uc2_Tilt[i])>=(0.5f*c_m))
		{
			ele->Ctrl_Flag[i]=0;
		}
		else
		{
			ele->Ctrl_Flag[i]=1;
		}
	}

}
void Transition_Init(sLOOPIO *ele)
{
			ele->Phase        = 0;
			ele->Vydd         = 0.0;
			ele->X_command[0] = CtrlFbck->X[0];
			ele->Y_command[0] = CtrlFbck->Y[0];
			ele->Y_command[1] = 0.0;
			ele->Tran_cnt = 0;
//		if (ele->Phase!=5)//非定点阶段
//		{
////			  X_err             = (ele->X_command[0]-CtrlFbck->X[0])* cos(CtrlFbck->Ang[2])+(ele->Y_command[0]-CtrlFbck->Y[0])*sin(CtrlFbck->Ang[2]);
////				ele->X_command[1] = 0.0f;
//			ele->Z_command[1] = 2.0;
//		}

}
void Transition_Calc(sLOOPIO *ele)
{
	double evy;
	double X_err,Y_err;
	evy    = ele->Y_command[1]-CtrlFbck->YH[1];
	float VRate = 2.0f;
	float VCase2 = 4.0f;
	switch (ele->Phase)
	{
		case 0://加速阶段
			//if (CtrlFbck->Ang[0] >= (10*D2R))//倾转至设定角度，进入下一阶段
		  if (Abs(CtrlFbck->YH[1])>=(ele->VelSet-0.5f) || CtrlMRAC2->uc2_Tilt[0]>=(15*D2R))
			{
				ele->Phase = 1;
			}
			ele->AngleLimitR = 80*D2R;
			ele->AngleLimitP = 20*D2R;
			ele->Vydd         = VRate;
			ele->Y_command[1] = ele->Y_command[1]+ele->Vydd*CtrlDt;
			ele->Y_command[1] = fConstrain(ele->Y_command[1],-ele->VelSet,ele->VelSet);
//		  ele->AngleLimit   = 20*D2R;
//		  ele->Y_command[1] = 2.0f;
			break;
		case 1://减速
			if (CtrlMRAC2->uc2_Tilt[0]<=(-15*D2R))
			{
				ele->Phase = 2;
			}
			if (Abs(CtrlFbck->YH[1])<=(VCase2+0.5f))//速度稳定，进入下一阶段
			{
				ele->Phase = 4;
				ele->Tran_cnt=0;
				ele->Z_mid = CtrlFbck->Z[0];
				pid[11].integral=0.0f;
			}
			ele->AngleLimitR = 80*D2R;
			ele->AngleLimitP = 20*D2R;
			ele->Vydd         = -VRate;
			ele->Y_command[1] = ele->Y_command[1]+ele->Vydd*CtrlDt;
			ele->Y_command[1] = fConstrain(ele->Y_command[1],-ele->VelSet,ele->VelSet);
//			ele->Y_command[1]=0.0f;
			break;
		case 2://爬高减速
			if (Abs(CtrlFbck->YH[1])<=(VCase2+0.5f))//速度稳定，进入下一阶段
			{
				ele->Phase = 3;
				ele->Tran_cnt=0;
			}
			ele->AngleLimitR = 80*D2R;
			ele->AngleLimitP = 20*D2R;
			ele->Vydd         = -VRate;
			ele->Y_command[1] = ele->Y_command[1]+ele->Vydd*CtrlDt;
			ele->Y_command[1] = fConstrain(ele->Y_command[1],-ele->VelSet,ele->VelSet);
//			ele->Y_command[1]=0.0f;
			ele->Z_command[1] = -0.4f-0.2f*Abs(ele->roll_command[0]-CtrlFbck->Ang[0])*R2D;
			break;
		case 3://进入悬停（减加速度）
			if (Abs(ele->Z_command[1]-CtrlFbck->Z[1])<0.4f)//加速度减至0，进入下一阶段
			{
        ele->Tran_cnt++;
			}
			if (ele->Tran_cnt>=100 )//速度减至0，进入下一阶段
			{
				ele->Phase = 4;
				ele->Tran_cnt=0;
        ele->Z_mid = CtrlFbck->Z[0];
				pid[11].integral=0.0f;
			}
			ele->AngleLimitR = 20*D2R;
			ele->AngleLimitP = 20*D2R;
			ele->Vydd         = -VRate;
			ele->Y_command[1] = ele->Y_command[1]+ele->Vydd*CtrlDt;
//			ele->Y_command[1]=0.0f;
			ele->Z_command[1]= 0.0f;
			if (Sign(ele->Y_command[1]) != Sign(VRate))
			{
				ele->Y_command[1]=0.0f;
			}
			break;
		case 4://进入悬停（减速度）
			if (Abs(evy)<=0.2f && ele->Y_command[1]==0.0f)
			{
				ele->Tran_cnt++;
			}
			if (ele->Tran_cnt>=100 )//速度减至0，进入下一阶段
			{
				ele->Phase = 5;
				ele->X_command[0]  = CtrlFbck->X[0];//当前位置作为位置给定
				ele->Y_command[0]  = CtrlFbck->Y[0];
				pid[9].integral    = 0.0f;
				pid[10].integral   = 0.0f;
			}
			ele->AngleLimitR = 20*D2R;
			ele->AngleLimitP = 20*D2R;
			ele->Vydd         = -VRate;
			ele->Y_command[1] = ele->Y_command[1]+ele->Vydd*CtrlDt;
			if (Sign(ele->Y_command[1]) != Sign(VRate))
			{
				ele->Y_command[1]=0.0f;
			}
			break;
		case 5://定点悬停
			ele->AngleLimitR = 20*D2R;
			ele->AngleLimitP = 20*D2R;
			X_err             = (ele->X_command[0]-CtrlFbck->X[0])* cos(CtrlFbck->Ang[2])+(ele->Y_command[0]-CtrlFbck->Y[0])*sin(CtrlFbck->Ang[2]);
			Y_err             = (ele->X_command[0]-CtrlFbck->X[0])*-sin(CtrlFbck->Ang[2])+(ele->Y_command[0]-CtrlFbck->Y[0])*cos(CtrlFbck->Ang[2]);
			ele->X_command[1] = Pid_Controller(pidX,X_err);  //机头坐标速度给定
			ele->Y_command[1] = Pid_Controller(pidY,Y_err);
		  break;			
		default:break;
	}
}
/*========================================END OF FILE========================================*/
