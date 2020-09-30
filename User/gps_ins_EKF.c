#include "gps_ins_EKF.h"

//**************EKF + complementary filter*****************//
STORE_IMU_BUFFER storeIMU;
STORE_GPS_BUFFER storeGPS;
STORE_BAR_BUFFER storeBAR; 
STORE_OUTPUT_BUFFER storeOUTPUT;
IMU_RING_ELEMENT imu_data_new,imu_data_delay;
GPS_RING_ELEMENT gps_data_new,gps_data_delay;
BAR_RING_ELEMENT bar_data_new,bar_data_delay;
OUTPUT_RING_ELEMENT output_data_new,output_data_delay;
double Pos_error_inter[3],Vel_error_inter[3];
bool horizon_success_flag = false;
bool bar_success_flag = false;
bool ekf_update_flag = false;
double Pos_err[3],Vel_err[3],dv1[3],dv2[3] = {0,0,9.788};
double Pos_cor[3],Vel_cor[3];
double dV_inter[3] = {0};
//**************EKF + complementary filter*****************//

/*********EKF GPS/INS 变量**********/
//double pp_init[9] = {0.4,0.4,0.4,0.1,0.1,0.1,0.0097803,0.0097803,0.0097803};
double pp_init[9] = {1.8e-18,1.9e-18,5.3e-6,
					0.0004,0.00035,8e-5,
					0.00035,0.00035,6.5e-5};
double PP_GI[81]={0};


/*********EKF GPS/INS 变量**********/

static const double R[6] = {1e-4,1e-4,1e-4,
														1e-4,1e-4,1e-4};
static const double Q[6] = {(1e-4)*g0,(1e-4)*g0,(1e-5)*g0,
														(1e-4)*g0,(1e-4)*g0,(1e-6)*g0};

double R_M[36] = {0};
double Q_M[36] = {0};

//注：程序详情请看matlab代码
/*****************GPS_INS_EKF Function*******************/

void diag(const double v[6], double d[36])
{
  int j;
  memset(&d[0], 0, 36U * sizeof(double));
  for (j = 0; j < 6; j++) {
    d[j + 6 * j] = v[j];
  }
}

void cross(const double a[3], const double b[3], double c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}




void mrdivide(double A[54], const double B[36])
{
  double b_A[36];
  int k;
  int j;
  signed char ipiv[6];
  int c;
  int jAcol;
  int jy;
  int ix;
  double smax;
  int iy;
  int i;
  double s;
  memcpy(&b_A[0], &B[0], 36U * sizeof(double));
  for (k = 0; k < 6; k++) {
    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    jAcol = 0;
    ix = c;
    smax = fabs(b_A[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > smax) {
        jAcol = k - 1;
        smax = s;
      }
    }

    if (b_A[c + jAcol] != 0.0) {
      if (jAcol != 0) {
        ipiv[j] = (signed char)((j + jAcol) + 1);
        ix = j;
        iy = j + jAcol;
        for (k = 0; k < 6; k++) {
          smax = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      k = (c - j) + 6;
      for (i = c + 1; i < k; i++) {
        b_A[i] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (jAcol = 1; jAcol <= 5 - j; jAcol++) {
      smax = b_A[jy];
      if (b_A[jy] != 0.0) {
        ix = c + 1;
        k = (iy - j) + 12;
        for (i = 7 + iy; i < k; i++) {
          b_A[i] += b_A[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  for (j = 0; j < 6; j++) {
    jy = 9 * j;
    jAcol = 6 * j;
    for (k = 1; k <= j; k++) {
      iy = 9 * (k - 1);
      if (b_A[(k + jAcol) - 1] != 0.0) {
        for (i = 0; i < 9; i++) {
          A[i + jy] -= b_A[(k + jAcol) - 1] * A[i + iy];
        }
      }
    }

    smax = 1.0 / b_A[j + jAcol];
    for (i = 0; i < 9; i++) {
      A[i + jy] *= smax;
    }
  }

  for (j = 5; j >= 0; j--) {
    jy = 9 * j;
    jAcol = 6 * j - 1;
    for (k = j + 2; k < 7; k++) {
      iy = 9 * (k - 1);
      if (b_A[k + jAcol] != 0.0) {
        for (i = 0; i < 9; i++) {
          A[i + jy] -= b_A[k + jAcol] * A[i + iy];
        }
      }
    }
  }

  for (jAcol = 4; jAcol >= 0; jAcol--) {
    if (ipiv[jAcol] != jAcol + 1) {
      iy = ipiv[jAcol] - 1;
      for (jy = 0; jy < 9; jy++) {
        smax = A[jy + 9 * jAcol];
        A[jy + 9 * jAcol] = A[jy + 9 * iy];
        A[jy + 9 * iy] = smax;
      }
    }
  }
}

void kalman_GPS_INS_pv_only_delay_level_arm(const double V_k[3], const double
  P_k[3], const double acc_bias_k[3], const double gps_data[6], const double Fb
  [3], const double T[9], double tao, const double PP0[81], bool Gps_update,
  const double Q[36], const double R[36], double V_n[3], double P_n[3], double
  P[81], double acc_bias_n[3], double dV_o[3])
{
  double x;
  double Rm;
  double Rn;
  double b_Fb[3];
  double V_p[3];
  int i0;
  int i;
  double PV_p[6];
  static const double dv0[3] = { 0.0, 0.0, 9.7883 };

  double F[81];
  double a;
  double G[54];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double b_T[9];
  double A[81];
  int i1;
  double b_A[81];
  double b_G[81];
  double c_G[54];
  double b_a[36];
  static const int b_b[54] = { 6378245, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6378245, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };

  static const int c_a[54] = { 6378245, 0, 0, 0, 0, 0, 0, 6378245, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double z[6];
  double XX[9];

  /* GPS/INS位置速度组合 卡尔曼滤波 */
  /* 连续状态系统方程 */
  /* dx = F*x + G*w             %z = H*x + v */
  /* 离散状态系统方程 */
  /* x(k+1) = A*x(k) + B*w(k)   %z(k+1) = C*x(k+1) + v(k+1) */
  /* Dpv     量测位置速度误差， 作为滤波器输入 */
  /* Q      系统噪声方差 */
  /* R      测量噪声方差 */
  /* Ta     加表误差漂移相关时间 */
  /* Tg     陀螺仪误差漂移相关时间 */
  /* tao    迭代步长 */
  /* 地球长半径 */
  /* 地球扁率 */
  /* 地球自转角速度         */
  /*     %% 上一时刻修正完位置，速度，比力信息 */
  /*    %% 计算当前时刻的北东地的比力 */
  /*   */
  /*     %% 当前时刻的子午曲率半径和卯酉曲率半径 */
  x = sin(P_k[0]);
  Rm = 6.378245E+6 * (0.99329437364420614 + 0.010058439533690743 * (x * x));
//  x = sin(P_k[0]);
  Rn = 6.378245E+6 * (1.0 - 0.0033528131778969143 * (x * x));

  /*     %% 当前时刻的北东地速度产生的角速度 */
  /*     %% 当前时刻地球自转角速度 */
  /*     %% 加速度比力方程 */
  b_Fb[0] = 2.0 * (7.292E-5 * cos(P_k[0])) + V_k[1] / (Rn + P_k[2]);
  b_Fb[1] = -V_k[0] / (Rm + P_k[2]);
  b_Fb[2] = 2.0 * (-7.292E-5 * sin(P_k[0])) + -V_k[1] * tan(P_k[0]) / (Rn + P_k[2]);
  cross(b_Fb, V_k, V_p);
  for (i0 = 0; i0 < 3; i0++) {
    b_Fb[i0] = Fb[i0] - acc_bias_k[i0];
  }

  /*     %% 一步预测 速度 */
  for (i = 0; i < 3; i++) {
    x = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      x += T[i + 3 * i0] * b_Fb[i0];
    }

    x = (x - V_p[i]) + dv0[i];
    V_p[i] = V_k[i] + x * tao;
    dV_o[i] = x;
  }

  /*     %% 一步预测 位置 */
  PV_p[0] = P_k[0] + V_k[0] / (Rm + P_k[2]) * tao;
  PV_p[1] = P_k[1] + V_k[1] / ((Rn + P_k[2]) * cos(P_k[0])) * tao;
  PV_p[2] = P_k[2] - V_k[2] * tao;
  PV_p[3] = V_p[0];
  PV_p[4] = V_p[1];
  PV_p[5] = V_p[2];

  /*     %% 指北方位系统误差模型 */
  memset(&F[0], 0, 81U * sizeof(double));
  x = Rm + P_k[2];
  F[18] = -V_k[0] / (x * x);
  F[27] = 1.0 / (Rm + P_k[2]);
  F[1] = V_k[1] * (1.0 / cos(P_k[0])) * tan(P_k[0]) / (Rn + P_k[2]);
  x = Rn + P_k[2];
  F[19] = -V_k[1] * (1.0 / cos(P_k[0])) / (x * x);
  F[37] = 1.0 / cos(P_k[0]) / (Rn + P_k[2]);
  F[47] = -1.0;
  x = 1.0 / cos(P_k[0]);
  F[3] = -0.00014584 * V_k[1] * cos(P_k[0]) - V_k[1] * V_k[1] * (x * x) / (Rn +
    P_k[2]);
  x = Rn + P_k[2];
  a = Rm + P_k[2];
  F[21] = V_k[1] * V_k[1] * tan(P_k[0]) / (x * x) - V_k[0] * V_k[2] / (a * a);
  F[30] = V_k[2] / (Rm + P_k[2]);
  F[39] = -0.00014584 * sin(P_k[0]) - 2.0 * V_k[1] * tan(P_k[0]) / (Rn + P_k[2]);
  F[48] = V_k[0] / (Rm + P_k[2]);
  x = 1.0 / cos(P_k[0]);
  F[4] = (0.00014584 * V_k[0] * cos(P_k[0]) + V_k[1] * V_k[0] * (x * x) / (Rn +
           P_k[2])) - 0.00014584 * V_k[2] * sin(P_k[0]);
  x = Rn + P_k[2];
  a = Rn + P_k[2];
  F[22] = -V_k[0] * V_k[1] * tan(P_k[0]) / (x * x) - V_k[1] * V_k[2] / (a * a);
  F[31] = 0.00014584 * sin(P_k[0]) + V_k[1] * tan(P_k[0]) / (Rn + P_k[2]);
  F[40] = V_k[2] / (Rn + P_k[2]) + V_k[0] * tan(P_k[0]) / (Rn + P_k[2]);
  F[49] = 0.00014584 * cos(P_k[0]) + V_k[1] / (Rn + P_k[2]);
  F[5] = 0.00014584 * V_k[1] * sin(P_k[0]);
  x = Rm + P_k[2];
  a = Rn + P_k[2];
  F[23] = V_k[0] * V_k[0] / (x * x) + V_k[1] * V_k[1] / (a * a);
  F[32] = -2.0 * V_k[0] / (Rm + P_k[2]);
  F[41] = -0.00014584 * cos(P_k[0]) - 2.0 * V_k[1] / (Rn + P_k[2]);
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      F[(i + 9 * (6 + i0)) + 3] = -T[i + 3 * i0];
    }
  }

  /*     %% G阵 */
  memset(&G[0], 0, 54U * sizeof(double));
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      G[(i + 9 * i0) + 3] = b[i + 3 * i0];
      b_T[i + 3 * i0] = -T[i + 3 * i0];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      G[(i0 + 9 * (3 + i)) + 3] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        G[(i0 + 9 * (3 + i)) + 3] += b_T[i0 + 3 * i1] * (double)b[i1 + 3 * i];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      G[(i + 9 * (3 + i0)) + 6] = b[i + 3 * i0];
    }
  }

  /*     %% 离散化 */
  memset(&A[0], 0, 81U * sizeof(double));
  for (i = 0; i < 9; i++) {
    A[i + 9 * i] = 1.0;
  }

  for (i0 = 0; i0 < 81; i0++) {
    A[i0] += F[i0] * tao;
  }

  for (i0 = 0; i0 < 54; i0++) {
    G[i0] *= tao;
  }

  /*     %% 协方差矩阵传播 */
  for (i0 = 0; i0 < 9; i0++) {
    for (i = 0; i < 9; i++) {
      b_A[i0 + 9 * i] = 0.0;
      for (i1 = 0; i1 < 9; i1++) {
        b_A[i0 + 9 * i] += A[i0 + 9 * i1] * PP0[i1 + 9 * i];
      }
    }

    for (i = 0; i < 6; i++) {
      c_G[i0 + 9 * i] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        c_G[i0 + 9 * i] += G[i0 + 9 * i1] * Q[i1 + 6 * i];
      }
    }

    for (i = 0; i < 9; i++) {
      F[i0 + 9 * i] = 0.0;
      for (i1 = 0; i1 < 9; i1++) {
        F[i0 + 9 * i] += b_A[i0 + 9 * i1] * A[i + 9 * i1];
      }

      b_G[i0 + 9 * i] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_G[i0 + 9 * i] += c_G[i0 + 9 * i1] * G[i + 9 * i1];
      }
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (i = 0; i < 9; i++) {
      P[i + 9 * i0] = F[i + 9 * i0] + b_G[i + 9 * i0];
    }
  }

  if (Gps_update == 1) {
    /*        %% 观测值矩阵 */

    /*         %% Kalman增益 */
    for (i0 = 0; i0 < 9; i0++) {
      for (i = 0; i < 6; i++) {
        G[i0 + 9 * i] = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          G[i0 + 9 * i] += P[i0 + 9 * i1] * (double)b_b[i1 + 9 * i];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i = 0; i < 9; i++) {
        c_G[i0 + 6 * i] = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          c_G[i0 + 6 * i] += (double)c_a[i0 + 6 * i1] * P[i1 + 9 * i];
        }
      }

      for (i = 0; i < 6; i++) {
        x = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          x += c_G[i0 + 6 * i1] * (double)b_b[i1 + 9 * i];
        }

        b_a[i0 + 6 * i] = x + R[i0 + 6 * i];
      }
    }

    mrdivide(G, b_a);
    memset(&F[0], 0, 81U * sizeof(double));
    for (i = 0; i < 9; i++) {
      F[i + 9 * i] = 1.0;
    }

    for (i0 = 0; i0 < 9; i0++) {
      for (i = 0; i < 9; i++) {
        x = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          x += G[i0 + 9 * i1] * (double)c_a[i1 + 6 * i];
        }

        b_A[i0 + 9 * i] = F[i0 + 9 * i] - x;
      }

      for (i = 0; i < 9; i++) {
        A[i0 + 9 * i] = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          A[i0 + 9 * i] += b_A[i0 + 9 * i1] * P[i1 + 9 * i];
        }
      }
    }

    for (i0 = 0; i0 < 9; i0++) {
      memcpy(&P[i0 * 9], &A[i0 * 9], 9U * sizeof(double));
    }

    for (i = 0; i < 6; i++) {
      z[i] = PV_p[i] - gps_data[i];
    }

    z[0] *= 6.378245E+6;
    z[1] *= 6.378245E+6;

    /*         %% 修正速度，位置 */
    for (i0 = 0; i0 < 9; i0++) {
      XX[i0] = 0.0;
      for (i = 0; i < 6; i++) {
        XX[i0] += G[i0 + 9 * i] * z[i];
      }
    }

    for (i = 0; i < 3; i++) {
      V_n[i] = PV_p[i + 3] - XX[i + 3];
      P_n[i] = PV_p[i] - XX[i];
      acc_bias_n[i] = acc_bias_k[i] - XX[i + 6];
    }

    /*     */
  } else {
    for (i = 0; i < 3; i++) {
      V_n[i] = PV_p[i + 3];
      P_n[i] = PV_p[i];
      acc_bias_n[i] = acc_bias_k[i];
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (i = 0; i < 9; i++) {
      b_A[i + 9 * i0] = (P[i + 9 * i0] + P[i0 + 9 * i]) / 2.0;
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    memcpy(&P[i0 * 9], &b_A[i0 * 9], 9U * sizeof(double));
  }
}




void Imu_Data_Push(IMU_RING_ELEMENT data)
{
	storeIMU.youngest = (storeIMU.youngest + 1)%IMU_BUFFER_SIZE;
	storeIMU.imu_data[storeIMU.youngest] = data;
	storeIMU.oldest = (storeIMU.youngest + 1)%IMU_BUFFER_SIZE;
	if(storeIMU.oldest ==0) storeIMU.is_filled = true;
}

void Imu_Data_Pop(IMU_RING_ELEMENT* data)
{
	*data = storeIMU.imu_data[storeIMU.oldest];
}

void Read_Imu_Data(sAHRS *ele)
{
	u8 index = 0;
	if(ahrs[1].gps_ins_update_flag)
	{
		imu_data_new.update_time = ahrs[1].ahrs_update_time;
		for(u8 i=0;i<3;i++)
		{
			imu_data_new.acc[i] = ele->AccB[i]; 
		}
	
		for(u8 i=0;i<3;i++)
		{
			for(u8 j=0;j<3;j++)
			{
				imu_data_new.T[index] = (double)ahrs[1].Cb2n[j][i];
				index++;
			}
		}
		Imu_Data_Push(imu_data_new);
		Imu_Data_Pop(&imu_data_delay);
		ahrs[1].gps_ins_update_flag = false;
	}
}


void Gps_Data_Push(GPS_RING_ELEMENT data)
{
	storeGPS.youngest = (storeGPS.youngest + 1)%GPS_BUFFER_SIZE; //越大的youngest所对应的值是越新的数据
	storeGPS.gps_data[storeGPS.youngest] = data;
}

void Read_Gps_Data(sGPS* ele)
{
	if(ele->gps_update_time - storeGPS.last_update_time > 70000)
	{
		gps_data_new.Pos_obs[0] = ele->lat*D2R_D; 
		gps_data_new.Pos_obs[1] = ele->lng*D2R_D;
		gps_data_new.Pos_obs[2] = ele->alti;
		
		gps_data_new.Vel_obs[0] = ele->NED_spd[0];
		gps_data_new.Vel_obs[1] = ele->NED_spd[1];
		gps_data_new.Vel_obs[2] = ele->NED_spd[2];
		
		storeGPS.last_update_time = ele->gps_update_time;
//		gps_data_new.update_time = ele->gps_update_time - GPS_DELAYED_TIME - 10000;
		gps_data_new.update_time = ele->gps_update_time - GPS_DELAYED_TIME;
		storeGPS.new_data_flag = true;
		Gps_Data_Push(gps_data_new);
	}
}

void Bar_Data_Push(BAR_RING_ELEMENT data)
{
	storeBAR.youngest = (storeBAR.youngest + 1)%IMU_BUFFER_SIZE;
	storeBAR.bar_data[storeBAR.youngest] = data;
	//storeBAR.oldest = (storeBAR.youngest + 1)%IMU_BUFFER_SIZE;
	//storeBAR.oldest = (storeBAR.oldest + 1)%IMU_BUFFER_SIZE;
}

void Bar_Data_Pop(BAR_RING_ELEMENT *data)
{
   *data = storeBAR.bar_data[storeBAR.oldest];
}

//void Read_Baro_Data(sEXIT* ele)
//{
//	if(ele->gps_ins_update_flag)
//	{
//		bar_data_new.update_time = ele->height_update_time - 25000;
//		bar_data_new.Height = exitx->DatRel[2];
//		storeBAR.new_data_flag = true;
//		Bar_Data_Push(bar_data_new);
//		//Bar_Data_Pop(&bar_data_delay);
//		ele->gps_ins_update_flag = false;
//	}
//}
void Read_Baro_Data(sMS5611* ele)
{
	if(ele->gps_ins_update_flag)
	{
		bar_data_new.update_time = ele->height_update_time - 25000;
		bar_data_new.Height = ele->AltFil;
		storeBAR.new_data_flag = true;
		Bar_Data_Push(bar_data_new);
		//Bar_Data_Pop(&bar_data_delay);
		ele->gps_ins_update_flag = false;
	}
}


void Store_Buffer_Init(sGPS *ele)
{
	u8 index = 0;															//指示值刚开始为0
	storeIMU.oldest = storeIMU.youngest = 0;
	storeIMU.is_filled = false;								//缓存器溢满标志位
	storeGPS.oldest = storeGPS.youngest = 0;	
	storeBAR.oldest = storeBAR.youngest = 0;	//气压计的结构体指针
	storeOUTPUT.oldest = storeOUTPUT.youngest = 0;
	for(u8 i=0;i<3;i++)
	{
		Pos_error_inter[i] = 0;
		Vel_error_inter[i] = 0;
	}
	if(ahrs[1].gps_ins_update_flag)
	{
		imu_data_new.update_time = ahrs[1].ahrs_update_time;;
		for(u8 i=0;i<3;i++)
		{
			imu_data_new.acc[i] = ahrs[1].AccB[i];
//			imu_data_new.w_qpr[i] = ahrs[1].pqr[i];
		}
		for(u8 i=0;i<3;i++)
		{
			for(u8 j=0;j<3;j++)
			{
				imu_data_new.T[index] = (double)ahrs[1].Cb2n[j][i];
				index++;
			}
		}
		storeIMU.imu_data[0] = imu_data_new;   //将测得的acc和pqr信息的首地址赋予storeIMU.imu_data
		ahrs[1].gps_ins_update_flag = false;   //若是没有更新信息，相当于下一次不进入这个if函数进行数据更新
	}
	if(ele->GPSUpdate)
	{
		gps_data_new.Pos_obs[0] = ele->lat*D2R_D; 
		gps_data_new.Pos_obs[1] = ele->lng*D2R_D;
		gps_data_new.Pos_obs[2] = ele->alti; 
		
		gps_data_new.Vel_obs[0] = ele->NED_spd[0];		//obs的意义？？？
		gps_data_new.Vel_obs[1] = ele->NED_spd[1];
		gps_data_new.Vel_obs[2] = ele->NED_spd[2];
		
		gps_data_new.update_time = ele->gps_update_time - GPS_DELAYED_TIME - 5000;
		storeGPS.gps_data[0] = gps_data_new;
		ele->GPSUpdate = 0;
		
		output_data_new.Pos[0] = ele->lat*D2R_D;
		output_data_new.Pos[1] = ele->lng*D2R_D;
		output_data_new.Pos[2] = ele->alti; 
		
		output_data_new.Vel[0] = ele->NED_spd[0];
		output_data_new.Vel[1] = ele->NED_spd[1];
		output_data_new.Vel[2] = ele->NED_spd[2];
		
		for(int i = 0;i<IMU_BUFFER_SIZE;i++)
		{
			storeOUTPUT.output_data[i].Pos[0] = ele->lat*D2R_D;
			storeOUTPUT.output_data[i].Pos[1] = ele->lng*D2R_D;
			storeOUTPUT.output_data[i].Pos[2] = ele->alti;
			
			storeOUTPUT.output_data[i].Vel[0] = ele->NED_spd[0];
			storeOUTPUT.output_data[i].Vel[1] = ele->NED_spd[1];
			storeOUTPUT.output_data[i].Vel[2] = ele->NED_spd[2];
		}
	}
	//if(exitx->gps_ins_update_flag)
//	if(ms5611->gps_ins_update_flag)
//	{
//		//bar_data_new.Height = exitx->DatRel[2];
//		//		bar_data_new.update_time = exitx->height_update_time - 15000;
//		//	exitx->gps_ins_update_flag = false;
//		bar_data_new.Height = ms5611[0].AltFil;                 //???
//		bar_data_new.update_time = ms5611->height_update_time - 15000;
//		ms5611->gps_ins_update_flag = false;
//		storeBAR.bar_data[0] = bar_data_new;
//	}
}


bool Recall_Gps_data(STORE_GPS_BUFFER* data,u32 sampleing_time)
{
	u8 oldest = data->oldest;
	u8 bestindex;
	bool success = false;
	if(!storeGPS.new_data_flag)
		return false;
	if(data->youngest == oldest)			//
	{
		if((data->gps_data[oldest].update_time!=0) && (data->gps_data[oldest].update_time <= sampleing_time)){
			if(sampleing_time - data->gps_data[oldest].update_time < 100000)
//			if(sampleing_time - data->gps_data[oldest].update_time < 50000)
			{
				bestindex = oldest;
				success = true;
				storeGPS.new_data_flag = false;
			}
		}
	}
	else{
		while(data->youngest != oldest)   
		{
			if((data->gps_data[oldest].update_time!=0) && (data->gps_data[oldest].update_time <= sampleing_time)){
				if(sampleing_time - data->gps_data[oldest].update_time < 100000)
//				if(sampleing_time - data->gps_data[oldest].update_time < 50000)
				{
					bestindex = oldest;
					success = true;
				}
			}
			else if (data->gps_data[oldest].update_time > sampleing_time)
			{
				break;
			}
			oldest = (oldest+1)%GPS_BUFFER_SIZE;
		}
	}
	
	if(success)
	{ 
		gps_data_delay = data->gps_data[bestindex];
		data->oldest = (bestindex+1)%GPS_BUFFER_SIZE;
		data->gps_data[bestindex].update_time = 0;  //准备用来做卡尔曼融合的GPS数据的的更新时间都强制为0
		return true;
	}
	else
	{
		return false;
	}
}

bool Recall_Bar_data(STORE_BAR_BUFFER* data,u32 sampleing_time)
{
	u8 oldest = data->oldest;
	u8 bestindex;
	bool success = false;
	if(!storeBAR.new_data_flag)
		return false;
	if(data->youngest == oldest)
	{
		if((data->bar_data[oldest].update_time!=0) && (data->bar_data[oldest].update_time <= sampleing_time)){
			if(sampleing_time - data->bar_data[oldest].update_time < 50000)
			{
				bestindex = oldest;
				success = true;
				storeBAR.new_data_flag = false;
			}
		}
	}
	else{
		while(data->youngest != oldest)
		{
			if((data->bar_data[oldest].update_time!=0) && (data->bar_data[oldest].update_time <= sampleing_time)){
				if(sampleing_time - data->bar_data[oldest].update_time < 50000)
				{
					bestindex = oldest;
					success = true;
				}
			}
			else if (data->bar_data[oldest].update_time > sampleing_time)
			{
				break;
			}
			oldest = (oldest+1)%IMU_BUFFER_SIZE;
		}
	}
	
	if(success)
	{
		bar_data_delay = data->bar_data[bestindex];
		data->oldest = (bestindex+1)%IMU_BUFFER_SIZE;
		data->bar_data[bestindex].update_time = 0;
		return true;
	}
	else
	{
		return false;
	}
}

void Output_Data_Push(OUTPUT_RING_ELEMENT data)
{
	storeOUTPUT.youngest = (storeOUTPUT.youngest + 1)%IMU_BUFFER_SIZE;
	storeOUTPUT.output_data[storeOUTPUT.youngest] = data;
	storeOUTPUT.oldest = (storeOUTPUT.youngest + 1)%IMU_BUFFER_SIZE;
}

void Output_Data_Pop(OUTPUT_RING_ELEMENT* data)
{
	*data = storeOUTPUT.output_data[storeOUTPUT.oldest];
}

void Calculate_Output(sGPS* ele)
{
	double Fb[3],b_T[3],P_temp[3],Vel_last[3];
	double x,Rm,Rn;
	double tao=0.01;
	
  x = sin(output_data_new.Pos[0]);          //纬度
  Rm = 6.378245E+6 * (0.99329437364420614 + 0.010058439533690743 * (x * x));       //Re*(1-2*e+3*e*sin(L)^2)
//  x = sin(output_data_new.Pos[0]);
  Rn = 6.378245E+6 * (1.0 - 0.0033528131778969143 * (x * x));       //Re*(1-e*sin(L)^2)

  Fb[0] = 2.0 * (7.292E-5 * cos(output_data_new.Pos[0])) + output_data_new.Vel[1] / (Rn + output_data_new.Pos[2]);		//机体的运动相对于惯性系的角速度
  Fb[1] = -output_data_new.Vel[0] / (Rm + output_data_new.Pos[2]);
  Fb[2] = 2.0 * (-7.292E-5 * sin(output_data_new.Pos[0])) - output_data_new.Vel[1] * tan(output_data_new.Pos[0]) / (Rn + output_data_new.Pos[2]);
  
	cross(Fb, output_data_new.Vel, dv1);			//哥氏加速度
  
	for (u8 i = 0; i < 3; i++) {
    Fb[i] = imu_data_new.acc[i] - ele->acc_bias_EKF[i];
		Vel_last[i] = output_data_new.Vel[i];
  }

  for (u8 i = 0; i < 3; i++) {
    x = 0.0;
    for (u8 j = 0; j < 3; j++) {
      x += imu_data_new.T[i + 3 * j] * Fb[j];
    }

    b_T[i] = (x - dv1[i]) + dv2[i];
    output_data_new.Vel[i] = output_data_new.Vel[i] + b_T[i] * tao;   //得到新的速度
  }

  P_temp[0] = output_data_new.Pos[0] + 0.5 * (output_data_new.Vel[0] + Vel_last[0])/ (Rm + output_data_new.Pos[2]) * tao;     //经度？
  P_temp[1] = output_data_new.Pos[1] + 0.5 * (output_data_new.Vel[1] + Vel_last[1])/ ((Rn + output_data_new.Pos[2]) * cos(output_data_new.Pos[0])) * tao;   //纬度？
  P_temp[2] = output_data_new.Pos[2] - 0.5 * (output_data_new.Vel[2] + Vel_last[2])* tao;
	
	output_data_new.Pos[0] = P_temp[0];
	output_data_new.Pos[1] = P_temp[1];
	output_data_new.Pos[2] = P_temp[2];
	
	Output_Data_Push(output_data_new);
	Output_Data_Pop(&output_data_delay);			//output_data_delay代表IMU的延时值，与EKF的值（延时）一一对应
	
	for(u8 i = 0;i<3;i++)
	{
		Pos_err[i] = ele->P_EKF[i] - output_data_delay.Pos[i];
		Pos_error_inter[i] += Pos_err[i];
		Vel_err[i] = ele->V_EKF[i] - output_data_delay.Vel[i];
		Vel_error_inter[i] += Vel_err[i];
	}
//	double tau = 0.25;
	double PosVelGain = 0.1;
	for(u8 i = 0;i<3;i++)
	{
		Pos_cor[i] = Pos_err[i] * PosVelGain + Pos_error_inter[i] * (PosVelGain * PosVelGain) * 0.1;    //????
	}
	//Pos_cor[2] = Pos_err[2] * 2 * PosVelGain + Pos_error_inter[2] * (2 * PosVelGain * 2 * PosVelGain) * 0.1;
	for(u8 i = 0;i<3;i++)
	{
		Vel_cor[i] = Vel_err[i] * PosVelGain + Vel_error_inter[i] * (PosVelGain * PosVelGain) * 0.1;
	} 
	OUTPUT_RING_ELEMENT output_state;
	for(u8 i = 0;i<IMU_BUFFER_SIZE;i++)
	{
		output_state = storeOUTPUT.output_data[i];

		for(u8 j=0;j<3;j++)
		{
			output_state.Pos[j] = output_state.Pos[j] + Pos_cor[j];
			output_state.Vel[j] = output_state.Vel[j] + Vel_cor[j];
		}
		storeOUTPUT.output_data[i] = output_state;
	}
	output_data_new = storeOUTPUT.output_data[storeOUTPUT.youngest];
}

void GPS_INS_EKF(sGPS* ele)
{
	
	if((ele->star>5)&&(!ele->GPS_INS_EKF_flag)&&(ahrs[1].circle_loop>=400)&&(!ele->GPS_INS_EKF_start_flag)&&(ele->ECEF_Init_Flag))//星数要求5   高度初始化完毕
	{
		ele->GPS_INS_EKF_flag = true;
		
		ele->V_EKF[0] = ele->NED_spd[0];   //初始化速度信息
		ele->V_EKF[1] = ele->NED_spd[1];
		ele->V_EKF[2] = ele->NED_spd[2];
		
		ele->P_EKF[0] = ele->lat*D2R_D;
		ele->P_EKF[1] = ele->lng*D2R_D;
		ele->P_EKF[2] = ele->alti;
		//ele->P_EKF[2] = exitx->DatRel[2];
		
		ele->acc_bias_EKF[0] = 0.0;
		ele->acc_bias_EKF[1] = 0.0;
		ele->acc_bias_EKF[2] = 0.0;
		
		ele->dV_o[0] = 0.0;
		ele->dV_o[1] = 0.0;
		ele->dV_o[2] = 0.0;
		
		ele->acc_bias_p[0] = 0.0;
		ele->acc_bias_p[1] = 0.0;
		ele->acc_bias_p[2] = 0.0;
		
		for(int i =1;i<=9;i++)
		{
			ele->P_GI[(i-1)*9 + i-1] = pp_init[i-1];
		}
		diag(Q,Q_M);	//初始化Q阵
		diag(R,R_M);	//初始化R阵
		
		//FIFO初始化函数
		Store_Buffer_Init(ele);			//疑问：每次进入EKF函数就会初始化一次吗？
	}
	//IMUFIFO 数据满了才开始进行EKF
	if(ele->GPS_INS_EKF_flag)
	{
		Read_Imu_Data(&ahrs[1]);
		Read_Gps_Data(ele);
		//Read_Baro_Data(exitx);
//		Read_Baro_Data(ms5611);
		if(storeIMU.is_filled)
		{
			ele->GPS_INS_EKF_start_flag = true;
			ele->GPS_INS_EKF_flag = false;
		}
	}
	//初始化完成后进行9阶EKF
	if(ele->GPS_INS_EKF_start_flag)
	{   
		double gps_data[6];
		Read_Imu_Data(&ahrs[1]);
		Read_Gps_Data(ele);
		//Read_Baro_Data(exitx);
//		Read_Baro_Data(ms5611);
		
		horizon_success_flag = Recall_Gps_data(&storeGPS,imu_data_delay.update_time);  //相当于看此时GPS的缓存里有没有最新的、之前没被IMU融合过的数据
		
//		bar_success_flag = Recall_Bar_data(&storeBAR,imu_data_delay.update_time);
//		
		ekf_update_flag = horizon_success_flag & 1;   ///bug
//		ekf_update_flag = 1;
		if(ekf_update_flag)
		{
			for(int i=0;i<3;i++)
			{
				gps_data[i] = gps_data_delay.Pos_obs[i];
			}
//			gps_data[2] = bar_data_delay.Height;
			for(int i=0;i<3;i++)
			{
				gps_data[i+3] = gps_data_delay.Vel_obs[i];
			}                                                                                                                            
		}

//		kalman_GPS_INS_pv_only_delay_level_arm(const double V_k[3], const double
//		P_k[3], const double acc_bias_k[3], const double gps_data[6], const double Fb
//		[3], const double T[9], double tao, const double PP0[81], bool Gps_update,
//		const double Q[36], const double R[36], double V_n[3], double P_n[3], double
//		P[81], double acc_bias_n[3], double dV_o[3])
//		
		kalman_GPS_INS_pv_only_delay_level_arm(ele->V_EKF, ele->P_EKF,ele->acc_bias_EKF,gps_data,
		imu_data_delay.acc,imu_data_delay.T, 0.01, ele->P_GI, ekf_update_flag,Q_M, R_M, ele->V_P, 
		ele->P_p, PP_GI, ele->acc_bias_p, ele->dV_o);

		//下一时刻预测的速度、位置、加速度偏置
		for(int i=0;i<3;i++)
		{
			ele->V_EKF[i] = ele->V_P[i];
			ele->P_EKF[i] = ele->P_p[i];
			ele->acc_bias_EKF[i] = ele->acc_bias_p[i];
			dV_inter[i] += (imu_data_new.acc[i] - ele->acc_bias_EKF[i])*0.01;
		}
		for(int i=0;i<81;i++)
		{
			ele->P_GI[i] = PP_GI[i];
		}
		Calculate_Output(ele);
	}
}
/*****************GPS_INS_EKF Function*******************/

/*******************经纬度转NED坐标系********************/
void LLH2NED(sGPS* ele)
{
	//零点ECEF初始完毕才开始计算NED坐标系相对位置
	if(ele->ECEF_Init_Flag && ele->GPS_INS_EKF_start_flag)
	{
		ele->LLH[0] = output_data_new.Pos[0];//ele->P_EKF[0];         //纬度
		ele->LLH[1] = output_data_new.Pos[1];         //经度
		ele->LLH[2] = output_data_new.Pos[2];//高度，注意是该减还是加
		//LLH换算成ECFF（地球中心坐标系）
		ele->N = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(ele->LLH[0])));
		ele->ECFF[0]= (ele->N + ele->LLH[2]) * cos(ele->LLH[0]) * cos(ele->LLH[1]);
		ele->ECFF[1]= (ele->N + ele->LLH[2]) * cos(ele->LLH[0]) * sin(ele->LLH[1]);
		ele->ECFF[2]= (ele->N*(1-SQR(C_WGS84_e)) +ele->LLH[2]) * sin(ele->LLH[0]);
		//减去零点ECEF
		ele->ECFF[0] -= ele->Zero_ECFF[0];
		ele->ECFF[1] -= ele->Zero_ECFF[1];
		ele->ECFF[2] -= ele->Zero_ECFF[2];
		//北东地坐标系坐标
		ele->NED[0] = ele->Re2t[0][0]*ele->ECFF[0]+ele->Re2t[0][1]*ele->ECFF[1]+ele->Re2t[0][2]*ele->ECFF[2];
		ele->NED[1] = ele->Re2t[1][0]*ele->ECFF[0]+ele->Re2t[1][1]*ele->ECFF[1]+ele->Re2t[1][2]*ele->ECFF[2];
		ele->NED[2] = ele->Re2t[2][0]*ele->ECFF[0]+ele->Re2t[2][1]*ele->ECFF[1]+ele->Re2t[2][2]*ele->ECFF[2];
//		ele->NED[2] = output_data_new.Pos[2];
		
//		ele->LLH_Init[0] = ele->lat*D2R_D;         //纬度
//		ele->LLH_Init[1] = ele->lng*D2R_D;         //经度
//		ele->LLH_Init[2] = ele->P_EKF[2];//高度，注意是该减还是加
//		
//		//LLH换算成ECFF（地球中心坐标系）
//		ele->N_Init = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(ele->LLH_Init[0])));
//		ele->ECFF_Init[0]= (ele->N_Init + ele->LLH_Init[2]) * cos(ele->LLH_Init[0]) * cos(ele->LLH_Init[1]);
//		ele->ECFF_Init[1]= (ele->N_Init + ele->LLH_Init[2]) * cos(ele->LLH_Init[0]) * sin(ele->LLH_Init[1]);
//		ele->ECFF_Init[2]= (ele->N_Init*(1-SQR(C_WGS84_e)) +ele->LLH_Init[2]) * sin(ele->LLH_Init[0]);
//	
//		ele->ECFF_Init[0] -= ele->Zero_ECFF[0];
//		ele->ECFF_Init[1] -= ele->Zero_ECFF[1];
//		ele->ECFF_Init[2] -= ele->Zero_ECFF[2];
//	
//		ele->NED_Init[0] = ele->Re2t[0][0]*ele->ECFF_Init[0]+ele->Re2t[0][1]*ele->ECFF_Init[1]+ele->Re2t[0][2]*ele->ECFF_Init[2];
//		ele->NED_Init[1] = ele->Re2t[1][0]*ele->ECFF_Init[0]+ele->Re2t[1][1]*ele->ECFF_Init[1]+ele->Re2t[1][2]*ele->ECFF_Init[2];
//		ele->NED_Init[2] = ele->Re2t[2][0]*ele->ECFF_Init[0]+ele->Re2t[2][1]*ele->ECFF_Init[1]+ele->Re2t[2][2]*ele->ECFF_Init[2];
	}
}
