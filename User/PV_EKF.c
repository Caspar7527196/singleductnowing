#include "PV_EKF.h"

/***************************************************\
功能说明：
	1.
	2.
	3.
	4.用起飞点的北东地坐标系进行局部导航
\***************************************************/

sPV_EKF pv_ekf[PV_EKF_NUM];
//--------------------------EKF------------------------
static int cnt=0;
double g_n[3]={0, 0, 9.788f};
//------------------功能函数------------------/
void EKF_PV_Init_Para(sPV_EKF *ele,sPV_EKF_CFG *elecfg)
{
	ele->name = elecfg->name;
	ele->Update = false;
	ele->dt = elecfg->dt;
	ele->Time = 0;
	ele->GPS_INS_EKF_flag = false;
	pv_ekf[0].Q_test = 5e-8f;
	pv_ekf[0].Q_bias_test = 5e-5f;
}
void EKF_PV(double X_p[9], float P_EKF[3], float V_EKF[3], float Accm[3], double P_p[][9], float dt,
			    float Cb2n[][3], double X_k[9], double X_n[9], double P_n[][9])
{	

	if(pv_ekf[0].newgps)//&&(gps->GPSUpdate == true))
	{
		pv_ekf[0].H[0][0]=1;//Hx =6X9,X=I9X9,H =Hx * X=[I6X6 06X3]
		pv_ekf[0].H[1][1]=1;
		pv_ekf[0].H[2][2]=1;
		pv_ekf[0].H[3][3]=1;
		pv_ekf[0].H[4][4]=1;
		pv_ekf[0].H[5][5]=1;
		//gps->GPSUpdate = false;
		//ms5611[0].Update = false;
		//HT
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<6;j++)
			{	
				pv_ekf[0].HT[i][j]= pv_ekf[0].H[j][i];
			}
		}
		//PHT
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<6;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 9 ; k++)
				{
						temp += pv_ekf[0].P_p[i][k] * pv_ekf[0].HT[k][j];
				}
				pv_ekf[0].PHT[i][j] = temp;
			}
		}		
		//HPHT
		for(int i=0;i<6;i++)
		{
			for(int j=0;j<6;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 9 ; k++)
				{
						temp += pv_ekf[0].H[i][k] * pv_ekf[0].PHT[k][j];
				}
				pv_ekf[0].HPHT[i][j] = temp;
			}
		}
		//HPHT_add_R
		for(int i=0;i<6;i++)
		{
			for(int j=0;j<6;j++)
			{
					pv_ekf[0].HPHT_add_R[i][j] = pv_ekf[0].HPHT[i][j] + pv_ekf[0].R[i][j];
			}
		}
		//=====================LU分解求逆=======================================================
		//---------若H1PH1T_add_R_inv_element为float，函数matrix_inverse_LU里需要做强制转换-------
		matrix_inverse_LU(6, pv_ekf[0].HPHT_add_R,pv_ekf[0].HPHT_add_R_inv);//
		//======================================================================================
//		for(int i=0;i<6;i++)
//		{
//			for(int j=0;j<6;j++)
//			{
//				double  temp = 0.0f;
//				for(int k = 0 ; k < 6 ; k++)
//				{
//						temp += pv_ekf[0].HPHT_add_R[i][k] * pv_ekf[0].HPHT_add_R_inv[k][j];
//				}
//				pv_ekf[0].test[i][j] = temp;
//			}
//		}
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<6;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 6 ; k++)
				{
						temp += pv_ekf[0].PHT[i][k] * pv_ekf[0].HPHT_add_R_inv[k][j];
				}
				pv_ekf[0].K_k[i][j] = temp;
			}
		}
		pv_ekf[0].h_X_p[0]=X_p[0];
		pv_ekf[0].h_X_p[1]=X_p[1];
		pv_ekf[0].h_X_p[2]=X_p[2];
		pv_ekf[0].h_X_p[3]=X_p[3];
		pv_ekf[0].h_X_p[4]=X_p[4];
		pv_ekf[0].h_X_p[5]=X_p[5];
		
		pv_ekf[0].Z_k[0]= P_EKF[0];//经度
		pv_ekf[0].Z_k[1]= P_EKF[1];//纬度
		pv_ekf[0].Z_k[2]= P_EKF[2];//气压计高度
		pv_ekf[0].Z_k[3]= V_EKF[0];//北速度
		pv_ekf[0].Z_k[4]= V_EKF[1];//东速度
		pv_ekf[0].Z_k[5]= V_EKF[2];//地速度
		
	//	Z_sub_h
		for(int i=0;i<6;i++)
		{
			pv_ekf[0].Z_sub_h[i]=(pv_ekf[0].Z_k[i]-pv_ekf[0].h_X_p[i]);	
		}
	//X_k_error
		for(int i=0;i<9;i++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
				temp += pv_ekf[0].K_k[i][k] * pv_ekf[0].Z_sub_h[k];
			}
			pv_ekf[0].X_k_error[i] = temp;
		}
	//KH
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<9;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 6 ; k++)
				{
						temp += pv_ekf[0].K_k[i][k] * pv_ekf[0].H[k][j];
				}
				pv_ekf[0].KH[i][j] = temp;
			}
		}
	}
	
	/*	
	else if((!gps->GPSUpdate)&&(ms5611->Update))//
	{	
		pv_ekf[0].H2[0][2]=1;//一行9列
		ms5611[0].Update = false;
		//H2T
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<1;j++)
			{	
				pv_ekf[0].H2T[i][j]= pv_ekf[0].H2[j][i];
			}
		}
		//PH2T
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<1;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 9 ; k++)
				{
						temp += pv_ekf[0].P_p[i][k] * pv_ekf[0].H2T[k][j];
				}
				pv_ekf[0].PH2T[i][j] = temp;
			}
		}
		
		//H2PH2T
		for(int i=0;i<1;i++)
		{
			for(int j=0;j<1;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 9 ; k++)
				{
						temp += pv_ekf[0].H2[i][k] * pv_ekf[0].PH2T[k][j];
				}
				pv_ekf[0].H2PH2T = temp;
			}
		}
		pv_ekf[0].H2PH2T_add_R2 = pv_ekf[0].H2PH2T + pv_ekf[0].R2;

		pv_ekf[0].H2PH2T_add_R2_inv=1/(pv_ekf[0].H2PH2T_add_R2);
		
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<1;j++)
			{
				pv_ekf[0].K_k2[i][j] = pv_ekf[0].PH2T[i][j] * pv_ekf[0].H2PH2T_add_R2_inv;
			}
		}
		pv_ekf[0].h_X_p2=X_p[2];
		
		pv_ekf[0].Z_k2= P_EKF[2];//气压计高度
		

		pv_ekf[0].Z_sub_h2=pv_ekf[0].Z_k2-pv_ekf[0].h_X_p2;	

	  //X_k_error
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<1;j++)
			{
				pv_ekf[0].X_k_error[i] = (pv_ekf[0].K_k2[i][j]) * (pv_ekf[0].Z_sub_h2);
			}
		}
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<9;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 1 ; k++)
				{
						temp += pv_ekf[0].K_k2[i][k] * pv_ekf[0].H2[k][j];
				}
				pv_ekf[0].KH[i][j] = temp;
			}
		}
	}
		
	else if((gps->GPSUpdate)&&(!ms5611->Update))//
	{
		pv_ekf[0].H3[0][0]=a;
		pv_ekf[0].H3[1][1]=a;

		pv_ekf[0].H3[2][3]=1;
		pv_ekf[0].H3[3][4]=1;
		pv_ekf[0].H3[4][5]=1;
		gps->GPSUpdate = false;
		//H3T
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<5;j++)
			{	
				pv_ekf[0].H3T[i][j]= pv_ekf[0].H3[j][i];
			}
		}
		//PH3T
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<5;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 9 ; k++)
				{
						temp += pv_ekf[0].P_p[i][k] * pv_ekf[0].H3T[k][j];
				}
				pv_ekf[0].PH3T[i][j] = temp;
			}
		}
		
		//H3PH3T
		for(int i=0;i<5;i++)
		{
			for(int j=0;j<5;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 9 ; k++)
				{
						temp += pv_ekf[0].H3[i][k] * pv_ekf[0].PH3T[k][j];
				}
				pv_ekf[0].H3PH3T[i][j] = temp;
			}
		}
		//H3PH3T_add_R
		for(int i=0;i<5;i++)
		{
			for(int j=0;j<5;j++)
			{
					pv_ekf[0].H3PH3T_add_R3[i][j] = pv_ekf[0].H3PH3T[i][j] + pv_ekf[0].R3[i][j];
			}
		}
		//=====================LU分解求逆=======================================================
		//---------若H3PH3T_add_R_inv_element为float，函数matrix_inverse_LU里需要做强制转换-------
		matrix_inverse_LU(5, pv_ekf[0].H3PH3T_add_R3,pv_ekf[0].H3PH3T_add_R3_inv);//
		//======================================================================================
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<5;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 5 ; k++)
				{
						temp += pv_ekf[0].PH3T[i][k] * pv_ekf[0].H3PH3T_add_R3_inv[k][j];
				}
				pv_ekf[0].K_k3[i][j] = temp;
			}
		}
		pv_ekf[0].h_X_p3[0]=X_p[0]*a;
		pv_ekf[0].h_X_p3[1]=X_p[1]*a;
		pv_ekf[0].h_X_p3[2]=X_p[3];
		pv_ekf[0].h_X_p3[3]=X_p[4];
		pv_ekf[0].h_X_p3[4]=X_p[5];
		
		pv_ekf[0].Z_k3[0]= P_EKF[0]*a;//经度
		pv_ekf[0].Z_k3[1]= P_EKF[1]*a;//纬度
		pv_ekf[0].Z_k3[2]= V_EKF[0];//北速度
		pv_ekf[0].Z_k3[3]= V_EKF[1];//东速度
		pv_ekf[0].Z_k3[4]= V_EKF[2];//地速度
		
	//Z_sub_h
		for(int i=0;i<5;i++)
		{
			pv_ekf[0].Z_sub_h3[i]=(pv_ekf[0].Z_k3[i]-pv_ekf[0].h_X_p3[i]);	
		}
	//X_k_error
		for(int i=0;i<9;i++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 5 ; k++)
			{
				temp += pv_ekf[0].K_k3[i][k] * pv_ekf[0].Z_sub_h3[k];
			}
			pv_ekf[0].X_k_error[i] = temp;
		}
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<9;j++)
			{
				double  temp = 0.0f;
				for(int k = 0 ; k < 5 ; k++)
				{
						temp += pv_ekf[0].K_k3[i][k] * pv_ekf[0].H3[k][j];
				}
				pv_ekf[0].KH[i][j] = temp;
			}
		}	
	}
	
*/
	
	else //if((!gps->GPSUpdate)&&(!ms5611->Update))
	{
		for(int i=0;i<9;i++)
		{
			pv_ekf[0].X_k_error[i] = 0.0f;
			for(int j=0;j<9;j++)
			{
				pv_ekf[0].KH[i][j] = 0.0f;
			}
		}
	}
	//X_p->X_k
	for(int i=0;i<9;i++)
	{
		X_k[i]=X_p[i]+pv_ekf[0].X_k_error[i];
	}
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{
				pv_ekf[0].I_sub_KH[i][j] = pv_ekf[0].I[i][j] - pv_ekf[0].KH[i][j];
		}
	}
	//P_p->P_k
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 9 ; k++)
			{
					temp += pv_ekf[0].I_sub_KH[i][k] * pv_ekf[0].P_p[k][j];
			}
			pv_ekf[0].P_k[i][j] = temp;
		}
	}
	
/*	
	pv_ekf[0].R_N= a*(1-2*e+3*e*sin(X_k[0])*sin(X_k[0]));
	pv_ekf[0].R_E= a*(1+e*sin(X_k[0])*sin(X_k[0]));
	pv_ekf[0].d_R_N= 6*a*e*sin(X_k[0])*cos(X_k[0]);
	pv_ekf[0].d_R_E= 2*a*e*sin(X_k[0])*cos(X_k[0]);
	
	pv_ekf[0].W_ie[0]=we*cos(X_k[0]);
	pv_ekf[0].W_ie[2]=-we*sin(X_k[0]);
	pv_ekf[0].W_en[0]=X_k[4]/(pv_ekf[0].R_E+X_k[3]);
	pv_ekf[0].W_en[1]=-X_k[3]/(pv_ekf[0].R_N+X_k[3]);
	pv_ekf[0].W_en[2]=-X_k[4]*tan(X_k[0])/(pv_ekf[0].R_E+X_k[3]);
	
	double g_n[3]={0, 0, g};
	for(int i=0;i<3;i++)
	{
		pv_ekf[0].W[i] = pv_ekf[0].W_ie[i] + pv_ekf[0].W_en[i];
	}
	cross_product_matrix(pv_ekf[0].W, pv_ekf[0].W_x);
	for(int i=0;i<3;i++)
	{
		double  temp = 0.0f;
		for(int j = 0 ; j < 3 ; j++)
		{
			temp +=pv_ekf[0].W_x[i][j] * X_k[j+3];
		}
		pv_ekf[0].a_c_n[i] = temp;
	}
	for(int i=0;i<3;i++)
	{
		double  temp = 0.0f;
		for(int j = 0 ; j < 3 ; j++)
		{
			temp +=Cb2n[i][j] * (AccB[j] - X_k[j+6]);
		}
		pv_ekf[0].Acc[i] = temp;
	}
	//比力方程
	for(int i=0;i<3;i++)
	{
		pv_ekf[0].d_V[i] = pv_ekf[0].Acc[i] - pv_ekf[0].a_c_n[i] + g_n[i];
	}
	
	//一阶泰勒近似做一步预测
	X_n[0] = X_k[0] + (X_k[3]/(pv_ekf[0].R_N+X_k[2]))*dt;
	X_n[1] = X_k[1] + (X_k[4]/((pv_ekf[0].R_E+X_k[2])*cos(X_k[0])))*dt;
	X_n[2] = X_k[2] + (-X_k[5])*dt;
	
	X_n[3] = X_k[3] + pv_ekf[0].d_V[0]*dt;
	X_n[4] = X_k[4] + pv_ekf[0].d_V[1]*dt;
	X_n[5] = X_k[5] + pv_ekf[0].d_V[2]*dt;
	//Acc要改
//	X_n[3] = X_k[3] + (-(X_k[4]/((pv_ekf[0].R_E+X_k[2])*cos(X_k[0]))+2*we)*X_k[4]*sin(X_k[0]) + X_k[3]*X_k[5]/(pv_ekf[0].R_N+X_k[2]) + AccB[0])*dt;
//	X_n[4] = X_k[4] + ((X_k[4]/((pv_ekf[0].R_E+X_k[2])*cos(X_k[0]))+2*we)*X_k[3]*sin(X_k[0]) + X_k[3]*X_k[5]/(pv_ekf[0].R_E+X_k[2]) + 2*we*X_k[5]*cos(X_k[0]) + AccB[1])*dt;
//	X_n[5] = X_k[5] + (-X_k[4]*X_k[4]/(pv_ekf[0].R_E+X_k[2]) - X_k[5]*X_k[5]/(pv_ekf[0].R_N+X_k[2]) - 2*we*cos(X_k[0]) + g + AccB[2])*dt;
	//
	X_n[6] = X_k[6];
	X_n[7] = X_k[7];
	X_n[8] = X_k[8];
	//雅可比矩阵
	pv_ekf[0].F[0][0]= 1;//+(-X_k[3]*pv_ekf[0].d_R_N/((pv_ekf[0].R_N+X_k[2])*(pv_ekf[0].R_N+X_k[2])))*dt;
	pv_ekf[0].F[0][2]= (-X_k[3]/((pv_ekf[0].R_N+X_k[2])*(pv_ekf[0].R_N+X_k[2])))*dt;
	pv_ekf[0].F[0][3]= (1/(pv_ekf[0].R_N+X_k[2]))*dt;
	
	pv_ekf[0].F[1][0]= (X_k[4]*tan(X_k[0])/( (pv_ekf[0].R_E+X_k[2]) * cos(X_k[0])) )*dt;//-X_k[4]*pv_ekf[0].d_R_E/( (pv_ekf[0].R_E+X_k[2]) * (pv_ekf[0].R_E+X_k[2]) * cos(X_k[0]) )+
	pv_ekf[0].F[1][1]= 1;
	pv_ekf[0].F[1][2]= (-X_k[4]/( (pv_ekf[0].R_E+X_k[2]) * (pv_ekf[0].R_E+X_k[2]) * cos(X_k[0]) ))*dt;
	pv_ekf[0].F[1][4]= (1/( (pv_ekf[0].R_E+X_k[2]) * cos(X_k[0]) ))*dt;
	
	pv_ekf[0].F[2][2]= 1;
	pv_ekf[0].F[2][5]= -1*dt;
	
	pv_ekf[0].F[3][0]= (-X_k[4]*X_k[4]/((pv_ekf[0].R_E+X_k[2])*cos(X_k[0])*cos(X_k[0]))  - 2*we*X_k[4]*cos(X_k[0]) )*dt;//+ X_k[4]*X_k[4]*tan(X_k[0])*pv_ekf[0].d_R_E/((pv_ekf[0].R_E+X_k[2])*(pv_ekf[0].R_E+X_k[2])) - X_k[3]*X_k[5]*pv_ekf[0].d_R_N/((pv_ekf[0].R_N+X_k[2])*(pv_ekf[0].R_N+X_k[2]))
	pv_ekf[0].F[3][2]= (X_k[4]*X_k[4]*tan(X_k[0])/((pv_ekf[0].R_E+X_k[2])*(pv_ekf[0].R_E+X_k[2])) - X_k[3]*X_k[5]/((pv_ekf[0].R_N+X_k[2])*(pv_ekf[0].R_N+X_k[2])))*dt;
	pv_ekf[0].F[3][3]= 1+(X_k[5]/(pv_ekf[0].R_N+X_k[2]))*dt;
	pv_ekf[0].F[3][4]= (-2*X_k[4]*tan(X_k[0])/(pv_ekf[0].R_E+X_k[2]) - 2*we*sin(X_k[0]))*dt;
	pv_ekf[0].F[3][5]= (X_k[3]/(pv_ekf[0].R_N+X_k[2]))*dt;
	pv_ekf[0].F[3][6]= -Cb2n[0][0]*dt;
	pv_ekf[0].F[3][7]= -Cb2n[0][1]*dt;
	pv_ekf[0].F[3][8]= -Cb2n[0][2]*dt;
	
	pv_ekf[0].F[4][0]= (X_k[4]*X_k[3]/((pv_ekf[0].R_E+X_k[2])*cos(X_k[0])*cos(X_k[0]))  + 2*we*X_k[3]*cos(X_k[0])  - 2*we*X_k[5]*sin(X_k[0]))*dt;//- X_k[4]*X_k[3]*tan(X_k[0])*pv_ekf[0].d_R_E/((pv_ekf[0].R_E+X_k[2])*(pv_ekf[0].R_E+X_k[2])) - X_k[4]*X_k[5]*pv_ekf[0].d_R_E/((pv_ekf[0].R_E+X_k[2])*(pv_ekf[0].R_N+X_k[2]))
	pv_ekf[0].F[4][2]= (-X_k[4]*( (X_k[3]*tan(X_k[0])+X_k[5])/((pv_ekf[0].R_E+X_k[2])*(pv_ekf[0].R_E+X_k[2])) ))*dt;
	pv_ekf[0].F[4][3]= (X_k[4]*tan(X_k[0])/(pv_ekf[0].R_E+X_k[2]) + 2*we*sin(X_k[0]))*dt;
	pv_ekf[0].F[4][4]= 1+((X_k[5]+X_k[3]*tan(X_k[0]))/(pv_ekf[0].R_E+X_k[2]))*dt;
	pv_ekf[0].F[4][5]= (X_k[4]/(pv_ekf[0].R_E+X_k[2])+2*we*cos(X_k[0]))*dt;
	pv_ekf[0].F[4][6]= -Cb2n[1][0]*dt;
	pv_ekf[0].F[4][7]= -Cb2n[1][1]*dt;
	pv_ekf[0].F[4][8]= -Cb2n[1][2]*dt;
	
	pv_ekf[0].F[5][0]= ( 2*we*X_k[4]*sin(X_k[0]) )*dt;//X_k[4]*X_k[4]*pv_ekf[0].d_R_E/((pv_ekf[0].R_E+X_k[2])*(pv_ekf[0].R_E+X_k[2])) + X_k[3]*X_k[3]*pv_ekf[0].d_R_N/((pv_ekf[0].R_N+X_k[2])*(pv_ekf[0].R_N+X_k[2])) + + 9.780327f*(0.0106048f*sin(X_k[0])*cos(X_k[0])-0.0000464*(sin(X_k[0])*cos(X_k[0])*cos(X_k[0])*cos(X_k[0])-sin(X_k[0])*sin(X_k[0])*sin(X_k[0])*cos(X_k[0])))+0.0000000088*X_k[2]*sin(X_k[0])*cos(X_k[0])
	pv_ekf[0].F[5][2]= (X_k[4]*X_k[4]/((pv_ekf[0].R_E+X_k[2])*(pv_ekf[0].R_E+X_k[2])) + X_k[3]*X_k[3]/((pv_ekf[0].R_N+X_k[2])*(pv_ekf[0].R_N+X_k[2])) )*dt;//- 0.0000030877+0.0000000044*sin(X_k[0])*sin(X_k[0])+0.000000000000144*X_k[2]
	pv_ekf[0].F[5][3]= (-2*X_k[3]/(pv_ekf[0].R_N+X_k[2]))*dt;
	pv_ekf[0].F[5][4]= (-2*X_k[4]/(pv_ekf[0].R_E+X_k[2]) - 2*we*cos(X_k[0]))*dt;
	pv_ekf[0].F[5][5]= 1;
	pv_ekf[0].F[5][6]= -Cb2n[2][0]*dt;
	pv_ekf[0].F[5][7]= -Cb2n[2][1]*dt;
	pv_ekf[0].F[5][8]= -Cb2n[2][2]*dt;
	pv_ekf[0].F[6][6]= 1;
	pv_ekf[0].F[7][7]= 1;
	pv_ekf[0].F[8][8]= 1;
	//把I+F*T记为F，即状态转移矩阵
	*/
	//X_k->X_n
	for(int i=0;i<3;i++)
	{
		pv_ekf[0].AccB[i]=ahrs[1].Accm[i]-X_k[i+6];//BODY
	}
	for(int i=0;i<3;i++)
	{
		double  temp = 0.0f;
		for(int k = 0 ; k < 3 ; k++)
		{
			temp += ahrs[1].Cb2n[i][k] * pv_ekf[0].AccB[k];
		}
		pv_ekf[0].Acc[i] = temp;////NED坐标系下比力
	}
	for(int i=0;i<3;i++)
	{
		X_n[i]=X_k[i]+X_k[i+3]*dt+0.5f*(pv_ekf[0].Acc[i]+g_n[i])*dt*dt;//012
		X_n[i+3]=X_k[i+3]+(pv_ekf[0].Acc[i]+g_n[i])*dt;//345
		X_n[i+6]= X_k[i+6];//678
	}
	//P_k->P_n

	pv_ekf[0].F[0][0]= 1.0f;
	pv_ekf[0].F[1][1]= 1.0f;
	pv_ekf[0].F[2][2]= 1.0f;
	pv_ekf[0].F[0][3]= dt;
	pv_ekf[0].F[1][4]= dt;
	pv_ekf[0].F[2][5]= dt;
	pv_ekf[0].F[3][3]= 1.0f;
	pv_ekf[0].F[4][4]= 1.0f;
	pv_ekf[0].F[5][5]= 1.0f;
	pv_ekf[0].F[6][6]= 1.0f;
	pv_ekf[0].F[7][7]= 1.0f;
	pv_ekf[0].F[8][8]= 1.0f;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			pv_ekf[0].F[i+3][j+6] = -ahrs[1].Cb2n[i][j]*dt;
		}
	}
	//FT
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{	
			pv_ekf[0].FT[i][j]= pv_ekf[0].F[j][i];
		}
	}
//	PFT
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 9 ; k++)
			{
					temp += pv_ekf[0].P_k[i][k] * pv_ekf[0].FT[k][j];
			}
			pv_ekf[0].PFT[i][j] = temp;
		}
	}
//	FPFT
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 9 ; k++)
			{
					temp += pv_ekf[0].F[i][k] * pv_ekf[0].PFT[k][j];
			}
			pv_ekf[0].FPFT[i][j] = temp;
		}
	}
	//GT
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<9;j++)
		{	
			pv_ekf[0].GT[i][j]= pv_ekf[0].G[j][i];
		}
	}
	//QGT
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<9;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
					temp += pv_ekf[0].Q[i][k] * pv_ekf[0].GT[k][j];
			}
			pv_ekf[0].QGT[i][j] = temp;
		}
	}
	//GQGT
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{
			double  temp = 0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
					temp += pv_ekf[0].G[i][k] * pv_ekf[0].QGT[k][j];
			}
			pv_ekf[0].GQGT[i][j] = temp;
		}
	}
	
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{	
			pv_ekf[0].P_n1[i][j] = pv_ekf[0].FPFT[i][j] + pv_ekf[0].GQGT[i][j];
		}
	}
	//P_n1T
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{	
			pv_ekf[0].P_n1T[i][j]= pv_ekf[0].P_n1[j][i];
		}
	}
	//P_n
	for(int i=0;i<9;i++)
	{
		for(int j=0;j<9;j++)
		{	
			pv_ekf[0].P_n[i][j] =( pv_ekf[0].P_n1[i][j] + pv_ekf[0].P_n1T[i][j])*0.5f;
		}
	}
	memset(pv_ekf[0].X_k_error,0,9U*sizeof(double));
}

bool EKF_PV_Calc(sGPS* ele)
{
	Tim_Calc(&(pv_ekf[0].Tim));   //计时
	if((gps->ECEF_Init_Flag)&&(!(pv_ekf[0].GPS_INS_EKF_flag))&&(ahrs[1].Update))//
	{
		pv_ekf[0].GPS_INS_EKF_flag = true;
		
		pv_ekf[0].X_p[0] = ele->NED[0];
		pv_ekf[0].X_p[1] = ele->NED[1];
		//ele->P_EKF[2] = exitx->DatRel[2];
		//pv_ekf[0].X_p[2] = (-exitx->DatFil[2]*ahrs[1].Cb2n[2][2]);//jiguang
		pv_ekf[0].X_p[2] = ele->NED[2];
		
		pv_ekf[0].X_p[3] = ele->NED_spd[0];
		pv_ekf[0].X_p[4] = ele->NED_spd[1];
		//pv_ekf[0].X_p[5] = -(exitx->DatSlopeFil[2]*ahrs[1].Cb2n[2][2]);//jiguang
		pv_ekf[0].X_p[5] = ele->NED_spd[2];
		
		pv_ekf[0].X_p[6] = 0.0f;
		pv_ekf[0].X_p[7] = 0.0f;
		pv_ekf[0].X_p[8] = 0.0f;
		for(int i =0;i<9;i++)
		{
			pv_ekf[0].P_p[i][i] = 0.00001f;
			pv_ekf[0].I[i][i]=1;
		}
		
//		pv_ekf[0].R[0][0] = 0.001f*(gps->gpsPosAccuracy)*(gps->gpsPosAccuracy);//0.0005f
//		pv_ekf[0].R[1][1] = 0.001f*(gps->gpsPosAccuracy)*(gps->gpsPosAccuracy);
//		pv_ekf[0].R[2][2] = 0.000004f;
//		pv_ekf[0].R[3][3] = 1.0f*(gps->gpsSpdAccuracy)*(gps->gpsSpdAccuracy);
//		pv_ekf[0].R[4][4] = 1.0f*(gps->gpsSpdAccuracy)*(gps->gpsSpdAccuracy);
//		pv_ekf[0].R[5][5] = 1.0f*(gps->gpsSpdAccuracy)*(gps->gpsSpdAccuracy);
		pv_ekf[0].R[0][0] = (gps->gpsPosAccuracy)*(gps->gpsPosAccuracy);//0.0005f
		pv_ekf[0].R[1][1] = (gps->gpsPosAccuracy)*(gps->gpsPosAccuracy);
//		pv_ekf[0].R[2][2] = 0.0001f;//jiguang
		pv_ekf[0].R[2][2] = (gps->gpsHeiAccuracy)*(gps->gpsHeiAccuracy);
		pv_ekf[0].R[3][3] = (gps->gpsSpdAccuracy)*(gps->gpsSpdAccuracy);
		pv_ekf[0].R[4][4] = (gps->gpsSpdAccuracy)*(gps->gpsSpdAccuracy);
//		pv_ekf[0].R[5][5] = 0.0001f;//jiguang
		pv_ekf[0].R[5][5] = (gps->gpsSpdAccuracy)*(gps->gpsSpdAccuracy);//gps
//		pv_ekf[0].Q[0][0] =  0.00000000003f;//gps
//		pv_ekf[0].Q[1][1] =  0.00000000003f;//gps
//		pv_ekf[0].Q[2][2] =  0.0000000000008f;//gps

		pv_ekf[0].Q[0][0] = pv_ekf[0].Q_test;//gps
		pv_ekf[0].Q[1][1] = pv_ekf[0].Q_test;//gps
		pv_ekf[0].Q[2][2] = pv_ekf[0].Q_test;//gps		
		
//		pv_ekf[0].Q[2][2] = 0.000000008f;//jiguang
		for(int i =0;i<3;i++)
		{
			
			pv_ekf[0].Q[i+3][i+3] = pv_ekf[0].Q_bias_test;//0.00000000003f
			
			pv_ekf[0].G[i+3][i] = 1.0f;
			pv_ekf[0].G[i+6][i+3] = 1.0f;
		}
	}
	cnt++;
	if(cnt==5)
	{
		cnt=0;
		pv_ekf[0].newgps=true;
	}
	if(pv_ekf[0].GPS_INS_EKF_flag)
	{
		//传感器值获取
		if(pv_ekf[0].newgps)//&&(gps->GPSUpdate == true))
		{
			pv_ekf[0].P_EKF[0] = ele->NED[0];
			pv_ekf[0].P_EKF[1] = ele->NED[1];
			//pv_ekf[0].P_EKF[2] = (-exitx->DatFil[2]*ahrs[1].Cb2n[2][2]);
			pv_ekf[0].P_EKF[2] = ele->NED[2];
			pv_ekf[0].V_EKF[0] = ele->NED_spd[0];
			pv_ekf[0].V_EKF[1] = ele->NED_spd[1];
			//pv_ekf[0].V_EKF[2] = -(exitx->DatSlopeFil[2]*ahrs[1].Cb2n[2][2]);
			pv_ekf[0].V_EKF[2] = ele->NED_spd[2];		
		}
		/*
		if(ms5611->Update)
		{
			pv_ekf[0].P_EKF[2] = ms5611[0].AltRel;	
		}
		if(gps->GPSUpdate)
		{
			pv_ekf[0].P_EKF[0] = ele->lat*D2R;
			pv_ekf[0].P_EKF[1] = ele->lng*D2R;
			pv_ekf[0].V_EKF[0] = ele->NED_spd[0];
			pv_ekf[0].V_EKF[1] = ele->NED_spd[1];
			pv_ekf[0].V_EKF[2] = ele->NED_spd[2];
		}
		*/

//-----------------------------------------------------------
		EKF_PV(pv_ekf[0].X_p, pv_ekf[0].P_EKF, pv_ekf[0].V_EKF, ahrs[1].Accm, pv_ekf[0].P_p, 0.01f,
						ahrs[1].Cb2n, pv_ekf[0].X_k, pv_ekf[0].X_n, pv_ekf[0].P_n);
		for(int j=0;j<9;j++)
		{
				pv_ekf[0].X_p[j] = pv_ekf[0].X_n[j];
		}
		for(int i=0;i<9;i++)
		{
			for(int j=0;j<9;j++)
			{	
				pv_ekf[0].P_p[i][j] = pv_ekf[0].P_n[i][j] ;
			}
		}
		//---------------NED位置,NED速度--------------------------
		for(int j=0;j<3;j++)
		{
				ahrs[1].Xyz[j] = pv_ekf[0].X_k[j];
				ahrs[1].Uvw[j] = pv_ekf[0].X_k[j+3];
		}
		float sinY,cosY;
		arm_sin_cos_f32(ahrs[1].Ang[2]*R2D,&sinY,&cosY);
		ahrs[1].UvwH[0] = pv_ekf[0].X_k[3]*cosY + pv_ekf[0].X_k[4]*sinY;
		ahrs[1].UvwH[1] = -pv_ekf[0].X_k[3]*sinY + pv_ekf[0].X_k[4]*cosY;
		ahrs[1].UvwH[2] = pv_ekf[0].X_k[5];
		
		ele->Update=true;	
//--------------------------------------------------------------		
	}
	//------------------------------------------------------------------------------------------
	Tim_Calc(&(pv_ekf[0].Tim));   //计时
	pv_ekf[0].Time = pv_ekf[0].Tim.OUT;	
	return ele->Update;
}


