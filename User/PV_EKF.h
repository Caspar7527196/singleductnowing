#ifndef __PV_EKF_H
#define __PV_EKF_H

#include "ahrs.h"
#include "userlib.h"
typedef struct
{
	char *name;
	float dt;
}sPV_EKF_CFG;

typedef struct
{
	bool Update;
	float dt;

	char *name;
	sTIM Tim;        //计时器
	u32  Time;
	
	bool newgps;
	float AccB[3];   //去掉偏置后机体坐标系加速度
	float Acc[3];    //NED加速度
	//float Uvw[3];    //NED速度
	//float Xyz[3];    //NED位置
	//float AccH[3];
	//float UvwH[3];
	//float XyzH[3];
	float P_EKF[3];
	float V_EKF[3];
	//-----------------by MCH--2018-10-10
	double H[6][9];
	//double H2[1][9];
	//double H3[5][9];
	double P_p[9][9];
	double HT[9][6];
	//double H2T[9][1];
	//double H3T[9][5];
	double PHT[9][6];
	//double PH2T[9][1];
	//double PH3T[9][5];
	double HPHT[6][6];
	//double H2PH2T;
	//double H3PH3T[5][5];
	double R[6][6];
	//double R2;
	//double R3[5][5];
	double HPHT_add_R[6][6];
	//double H2PH2T_add_R2;
	//double H3PH3T_add_R3[5][5];
	double HPHT_add_R_inv[6][6];
	double test[6][6];
	//double H2PH2T_add_R2_inv;
	//double H3PH3T_add_R3_inv[5][5];
	double K_k[9][6];
	//double K_k2[9][1];
	//double K_k3[9][5];
	double h_X_p[6];
	//double h_X_p2;
	//double h_X_p3[5];
	double Z_k[6];
	//double Z_k2;
	//double Z_k3[5];
	double Z_sub_h[6];
	//double Z_sub_h2;
	//double Z_sub_h3[5];
	double X_k_error[9];
	//double R_N;
	//double R_E;
	//double d_R_N;
	//double d_R_E;
	double X_p[9];
	double X_k[9];
	double X_n[9];
	//double W[3], W_x[3][3], a_c_n[3], d_V[3], W_ie[3], W_en[3];
	double F[9][9];
	double KH[9][9];
	double I_sub_KH[9][9];
	double P_k[9][9];
	double FT[9][9];
	double PFT[9][9];
	double FPFT[9][9];
	double P_n[9][9];
	double P_n1T[9][9];
	double P_n1[9][9];
	double I[9][9];
	double Q[6][6];
	double G[9][6];
	//double GP[9][9];
	double GT[6][9];
	double QGT[6][9];
	double GQGT[9][9];
	bool GPS_INS_EKF_flag;
	double Q_test;
	double Q_bias_test;
}sPV_EKF;
//-----------------------------------
#define PV_EKF_NUM 2

extern sPV_EKF pv_ekf[PV_EKF_NUM];
void EKF_PV_Init_Para(sPV_EKF *ele,sPV_EKF_CFG *elecfg);
bool EKF_PV_Calc(sGPS* ele);
#endif
