#ifndef __CONTROL_H
#define __CONTROL_H

#include "motor.h"
#include "ahrs.h"
#include "pid.h"
#include "rc.h"
#include "icm20602.h"
#include "ms5611.h"
//#include "command.h"
#include "exit.h"
#include "userlib.h"
#include "eso.h"
//#include "gps_ins_EKF.h"

#define CTRLFB_NUM 1
#define CTRLIO_NUM 1
#define CTRLIL_NUM 1
#define CTRLKT_NUM 1
#define CtrlDt 0.01f
#define CTRLFB_SLOPE_NUM 15
#define GROSSMASS 1.6f
#define HOVER_SPEED 1225.0f//
#define HOVER_PWM 1658//
#define DRIFT_PWM 1630
#define ACC_FF_CO 0.3456008821f//���ٶ�ǰ������
#define CTRLPID_NUM 12
#define FOLD_WING_PWM_MIN 0.0f
#define FOLD_Wing_PWM_MAX 900.0f
//#define DM1_MRAC 42.32014f  //MRAC 1
//#define DM2_MRAC 42.32014f  //MRAC 2
//#define DM3_MRAC 16.47319f  //MRAC 3
#define DM1_MRAC   0.100235f  //MRAC 1
#define DM2_MRAC   0.100155f  //MRAC 2
#define DM3_MRAC   0.17692f  //MRAC 3
#define G4_BAR   211.09765f  //g4_bar


#define RC_CHECK_NUM 500
typedef enum {NORMAL=0,CLOSE=1,LOST=2}eRC;

typedef enum
{
	MODE_MANUAL = 0x00U,          //�ֿ�
	MODE_ALTITUDE  = 0x01U,       //��̬
	MODE_HIGH_HOLD = 0x02U,       //����
	MODE_POSITION_HOLD = 0x03U,   //����
	MODE_TARGET = 0x04U,          //����
}eMODE;

typedef struct
{
	float pqr[3];//0 for p, 1 for q, 2for r
	double Ang[3];//0 for phi, 1 for theta, 2 for psi
	//-----------------------------------------------------------------------------------
	//NED XYZ, 0 for positition, 1 for velocity, 2 for acceleration
	float X[3];
	float Y[3];
	float Z[3];
	float Z_offset;		//��ң������ʱ�� ��ȡ��ǰ�߶�
	//Headframe XY //0 for position, 1 for velocity
	float XH[2];
	float YH[2];
	//-----------------------------------------------------------------------------------
	//Bodyframe UVW,0 for UVW, 1 for UVW derivative
	float U[2];
	float V[2];
	float W[2];
	float  AirSpeed;
	float  AirSpeed_Last;
	//������ת��
	float engine_speed;
	//����ģ̬
	int flymode;
	float Z1_temp[CTRLFB_SLOPE_NUM];
}sFEEDBACK;

typedef struct
{
	float input[4];//��������
	double output[5];//�������,T,Mx,My,Mz,��
	int cs_output[7];//�������
	int mt_output[2];//������
	float gyro_output[3];//�������ز���
	float gyro_gain;//��������ϵ��
	int fly_mode;//����ģʽ��1ֱ������2�̶���3����
	int yaw_mode;//����������0����β��1��β
	int control_mode;		//0�ֿأ�1���Կأ�2�Կأ�3�ٶȿ��ƣ�4ʧ�ر���
	int last_yaw_mode;
	int last_control_mode;
	bool wing_flag;//flase�۵���trueչ��
	int last_fly_mode;
	double mid_trim[3];//ƽ��������roll,pitch yaw
	int pwm_temp;//������
	int pwm_temp1;
	double roll_sin;
	//-----------------------------------------
	double pqr_err[3];
	float output_pid[3];
	double output1[3];
	double output2[3];
	int output_pwm_adjust[2];
	int output_yaw_pwm_adjust[2];
	double output_sat[3];
	double U_d[3];
	int InterfereFlag;
	//-----------------------------------------
	u8 rc_buffer[RC_CHECK_NUM];
	u16 rc_cnt;
	u16 rc_check;
	bool ever_rc;
	eRC rc_status;
}sIO;

typedef struct
{
	float X_command[2];//X[0]ΪNED�±���λ�ã�X[1]ΪNED��X�᷽���ٶ�(�Ǳ����ٶȣ�
	float Y_command[2];//Y[0]ΪNED�±���λ�ã�Y[1]ΪNED��Y�᷽���ٶ�(�Ǳ����ٶ�)
	float Z_command[2];
	float Z_mid;
	int   Z_mid_pwm;
	
	float acc_command[3];//NED���ٶ�
	float roll_command[2];
	float pitch_command[2];
	float yaw_command[2];
	float pqr_command[3];
	float thrust_command[2];
	bool  position_mode;
  bool  brake_mode;
	int   brake_cnt;
	float yaw_mid;
	float AngleLimitR;
	float AngleLimitP;
	//-----------------------------------------
	double roll_err;
	double pitch_err;
	double yaw_err;
	//-----------------------------------------

  int    Tran_cnt;
	float  VelNEDH_Last[3];
	int    Phase;
	float  Vydd;//���ٶȹ滮
	float  Vyc; //�ٶȹ滮
	float  VelSet;//�ٶ���ֵ
	//
	float Z_test;
	float Z_test_mid;
	float Z_test_mid2;
	//
	float Vn_Pred[3];//��ͷϵ
	float u1_Bar[2];
	float u1_Tilt[2];
	float Rd_Bar;
	float Rd_tilt;
	float Pd_Bar;
	float Pd_Tilt;
  double w1;
	double wr;
	double wp;
	float ko1;
	float ko2;
	float ko3;
	float k1;

}sLOOPIO;


typedef struct
{
	float  pqr_Pred[3];
	double uc2_Bar[3];
	double uc2_Tilt[3];
	double uc2[3];
	double phi3[3][3];
	double phi4[4];
	double w3[3][3];
	double w4[4];
	float  k1;
	float  k2[3][3];
	float  kuc2[3];
	float  k5[3][4];
	double g3_Est[3];
	double g3_uc2[3];
	bool   Ctrl_Flag[3];
}sMRAC2;



typedef struct
{
	float pid_Ki_temp[12];
	bool int_reset;
	bool test;
}sKITEMP;



extern sFEEDBACK CtrlFbck[CTRLFB_NUM];
extern sIO CtrlIO[CTRLIO_NUM];
extern sLOOPIO CtrlLpIO[CTRLIL_NUM];
extern sKITEMP CtrlKiTemp[CTRLKT_NUM];
extern sMRAC2 CtrlMRAC2[CTRLKT_NUM];


void Control_Init(void);
void Control_Feedback(sFEEDBACK *ele);
void Out_Loop_XY_Pre(sLOOPIO *ele);
void Out_Loop_Z_Pre(sLOOPIO *ele);
void Out_Loop_Step(sLOOPIO *ele);
void Underactuated_Loop_Step(sLOOPIO *ele);
void In_Loop_Step(sLOOPIO *ele);
void Control_Output(sIO *ele);
void Output_To_Motor(sIO *ele);
void MRAC_Calc2(sMRAC2 *ele, double pqr_input[3]);
void MRAC_Init2(sMRAC2 *ele);
void Transition_Init(sLOOPIO *ele);
void Transition_Calc(sLOOPIO *ele);
void ControlRC_Check(sIO *ele);          
void Integral_Reset(sKITEMP *ele);
void Integral_Restore(sKITEMP *ele);
void Gyro_Control(sIO *ele, sFEEDBACK *fb);


#endif
