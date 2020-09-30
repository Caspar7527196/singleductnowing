#ifndef __AHRS_H
#define __AHRS_H


//#include "mpu6050.h"
//#include "adis16365.h"
//#include "adis16480.h"
//#include "lsm303d.h"
//#include "hmc5883.h"
#include "gps.h"
#include "ms5611.h"
//#include "command.h"
#include "exit.h"
#include "pid.h"	//??,PI??
//#include "control.h"
#include "icm20602.h"
#include "userlib.h"

#define USE_UWB 
#define HARD_FILT(x,y) (0.9f*(x)+0.1f*(y))
typedef enum {FLY_HEL=0,FLY_FIX=1}eFLY;

typedef struct
{
	char *name;
	float halfDt;
	float AngOff[3];
	eFLY  mode; 
}sAHRS_CFG;
//---------------------------------2018.10 MCH
typedef struct
{
	bool Update;
	float dt;
	arm_matrix_instance_f32 matA,mat33,matB;
	float LLS[3][3];
	char *name;
	sTIM Tim;        //计时器
	u32  Time;
	float halfDt;
	float Cb2n[3][3];
	float AngOff[3]; //偏移量
	float DatAcc[3];
	float DatMag[3];
	float DatGyr[3];
	float pqr[3];    //陀螺仪
	float Ang[3];    //融合后的角度
	float AccB[3];   //减偏置后机体坐标系加速度
	float Accm[3];   //机体坐标系加速度
	float Acc[3];    //NED加速度
	float Uvw[3];    //NED速度
	float Xyz[3];    //NED位置
	float AccH[3];
	float UvwH[3];
	float XyzH[3];
	float Fac;       //倾斜角因子
	float w,x,y,z;   //四元数
	float exInt,eyInt,ezInt;
	//-----------------by MCH--2018-10-10
	double H_element[4][6];
	double H1_element[4][7];
	double H2_element[7][6];
	double P_p_element[6][6];
	double HPHT_middle_element[4][4];
	double HPHT_add_R_element[4][4];
	double HPHT_add_R_inv_element[4][4]; //根据情况调整类型
	double K_k_element[6][4];
	double h_X_p_element[4];
	double Z_k_element[4];
	double Z_sub_h_element[4];
	double X_k_error_element[6];
	double d_theta[3],d_phi,d_u[3];	
	double d_q_element[4];
	double X_p_q[4];
	double X_p_w[3];
	double X_k_q[4];
	double X_k_w[3];

	double wm_sub_wb[3], dw_phi, dw_u[3];
	double q_w[4];
	double X_n_q[4];
	double X_n_w[3];
	double Rot_element[3][3];
	double F_element[6][6];
	double KH_element[6][6];
	double I_sub_KH_element[6][6];
	
	double P_k_element[6][6];
	double FT_element[6][6];
	double PFT_element[6][6];
	double FPFT_element[6][6];
	
	double P_n_element[6][6];
  double X_n_element[7];

	double P_n1T_element[6][6];
	double P_n1_element[6][6];
	double P_n2_element[6][6];
	double I_element[6][6];
	double Q_element[6][6];
	double R_element[4][4];
	double G_element[6][6];
	double GP_element[6][6];
	double GT_element[6][6];
	double HT_middle_element[6][4];					 
	double PHT_middle_element[6][4];
	
	double Q1;
	double Q2;
	double R1;
	double R2;
	//--------------------------------------------------------------------------------------------------

/*
//	arm_matrix_instance_f32    X_k, K_k, P_k, P_p, X_n, P_n, X_p, Q, R;
//	//?????????
//	float X_p_element1[7];
//	float P_p_element1[7][7];
//	float X_k_element1[7];
//	float X_n_element1[7];
//	float P_n_element1[7][7];
//	float K_k_element1[7][4];
//	float P_k_element1[7][7];
//	float Q_element1[7][7];
//	float R_element1[4][4];
//	//??????????
//	arm_matrix_instance_f32  H, h_X_p, Z_k;
//	float H_element1[4][7];
//	float h_X_p_element1[4];
//	float Z_k_element1[4];
//	//???????
//	arm_matrix_instance_f32  A, Df_k, f_k;
//	float A_element1[7][7];
//	float Df_k_element1[7][7];
//	float f_k_element1[7];
//	//??????????
//	arm_matrix_instance_f32  HT_middle, Z_sub_h, X_k_error, KH, I_sub_KH, 
//													 AT, PAT, APAT, P_n1T, P_n2, P_n1, I, PHT_middle;
//	//------------??double????-----------------------
//	//arm_matrix_instance_f32   HPHT_middle, HPHT_add_R;
//	//-----------------------------------------------------
//	//-------------------------------------------------------------------------------------------------
//	double HPHT_middle_element1[4][4];
//	double HPHT_add_R_element1[4][4];		
//	//------------??matrix_inverse_LU-----------------------------------------------------------------
//	arm_matrix_instance_f32 HPHT_add_R_inv;//??arm_mat_init_f32???
//	float HPHT_add_R_inv_element1[4][4]; //????????
//	//--------------------------------------------------------------------------------------------------
//	float HT_middle_element1[7][4];					 
//	float PHT_middle_element1[7][4];
//	float Z_sub_h_element1[4];
//	float X_k_error_element1[7];
//	float KH_element1[7][7];
//	float I_sub_KH_element1[7][7];
//	float AT_element1[7][7];
//	float PAT_element1[7][7];
//	float APAT_element1[7][7];
//	float P_n1T_element1[7][7];
//	float P_n1_element1[7][7];
//	float P_n2_element1[7][7];
//	float I_element1[7][7];
*/
	
	//------------------
	float Ang_Init[3]; //加速计和磁力计初始化得出来的角度
	float q_init[4]; //初始化四元数
	int Cnt;
	
	float gyr_buffer[3][200];
	int gyr_cnt;
	
float PosOffsetBody[3];
float PosOffsetNed[3];
float VelOffsetBody[3];
float VelOffsetNed[3];
float PosNedCor[3];
float VelNedCor[3];
float yaw_mag;

float acc_buffer[3][200];
float acc_buffer_data[3];
float Acc_Fil_buffer[3][200];
u8 acc_buffer_cnt;
u8 Acc_Fil_cnt;

float acc_sum;
float acc_R_x;
float acc_R_a;
float acc_R_b;
float R;


int circle_loop;
u32   ahrs_update_time;
bool  gps_ins_update_flag;
}sAHRS;
//-----------------------------------
#define AHRS_NUM 2



extern sAHRS ahrs[AHRS_NUM];


void Ahrs_Init_Para(sAHRS *ele,sAHRS_CFG *elecfg);
bool Ahrs_Init(sAHRS *ele);
bool Ahrs_Calc(sAHRS *ele);
bool ESKF_Init_MCH(sAHRS *ele);
bool ESKF_Attitude_Calc_MCH(sAHRS *ele);
bool ESKF_Attitude_Calc_CHR(sAHRS *ele);
bool EKF_Init_MCH(sAHRS *ele);
bool EKF_Attitude_Calc_MCH(sAHRS *ele);
bool EKF_Attitude_Calc_CHR(sAHRS *ele);

	
#endif
