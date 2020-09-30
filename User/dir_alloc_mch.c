/* Include Files */
#include "dir_alloc_mch.h"
//#include "float.h"
//#include "limits.h"
//---------------------------------by mch
sDir dir;
//----------------------------------------
/* Function Declarations */


/* Function Definitions */


/*
 * (c) mengchaoheng
 *  Last edited 2019-11
 *    min z=c*x   subj. to  A*x (=、 >=、 <=) b
 *    x
 *  原问题
 *  Performs direct control allocation by solving the LP
 *    max z=a   subj. to  Bu = av
 *    a,u               umin <= u <= umax
 *  If a > 1, set u = u/a.
 *  Note: This function has not been optimized for speed.
 *   Inputs:
 *   -------
 *  B     control effectiveness matrix (k x m)
 *  v     commanded virtual control (k x 1)
 *  umin  lower position limits (m x 1)
 *  umax  upper position limits (m x 1)
 *   Outputs:
 *   -------
 *  u     optimal control (m x 1)
 *  a     scaling factor
 *  整理成
 *    min z=[0 -1]x   subj. to  [B -v]x = 0
 *    x                       [I 0;-I 0]x <= [umax; -umin]
 *    其中 x=[u; a]
 *  对应《凸优化》p139,记为
 *    min z=c*x   subj. to  Aeq*x = beq
 *    x                     G*x <= h
 *  合并
 *    min z=c*x   subj. to  [Aeq; G]*x (=、<=) [beq;h]
 *    x
 *  保证x>=0，变形
 *    min z=[c -c]*X   subj. to  [Aeq -Aeq;G -G]*X (=、<=) [beq;h]
 *     X
 *  其中 X=[x^+; x^-]
 *
 *  B=[-0.5   0       0.5   0;
 *       0  -0.5    0       0.5;
 *      0.25   0.25   0.25   0.25];
 * Arguments    : const double v[3]
 *                const double umin[4]
 *                const double umax[4]
 *                double u[4]
 * Return Type  : void
 */
double P_inv[13][13]={{-1,     1,     2,     0,     0,     0,     0,    0,     0,     0,     0,     0,     0},
	{0,    -2,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0},
	{1,     1,     2,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0},
	{1,    -1,    -2,     1,     0,     0,     0,     0,     0,     0,     0,     0,     0},
	{0,     2,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0,     0},
	{-1,    -1,    -2,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0},
	{0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0,     0},
	{0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0},
	{-1,     1,     2,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0},
	{0,    -2,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0},
	{1,     1,     2,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0},
	{0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     0},
    {0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1}}; 
	
bool check_c(const double c[20], int *e)
{
	for(int i=0;i<20;++i)
	{
		if(c[i]<0)
		{
			*e=i;
			return 1;
		}
	}
	return 0;
}

//double Ad5[13]={0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1};
//double Ad10[13]={0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1};
//double b[13]={0, 0, 0, 0, 0, 0, 0, 50, 0, 0, 0, 0, 0};
//double A[13][20]={{1.0f,     0.0f,     0.0f,     1.0f,     0.0f,    -1.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     1.0f,     0.0f,    -1.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     1.0f,     1.0f,     0.0f,     0.0f,     0.0f,    -1.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f}};	

void dir_alloc_mch(double v[3], double umin[4], double umax[4],
                   double u[4])
{
	memset(dir.Ad5, 0, 13U * sizeof(double));
	memset(dir.Ad10, 0, 13U * sizeof(double));
	dir.Ad5[7]=1; 
	dir.Ad5[12]=-1;
	dir.Ad10[7]=-1; 
	dir.Ad10[12]=1;
	memset(dir.A, 0, 260U * sizeof(double));
	for(int i=0;i<3;++i)
	{
		dir.A[i][i]=1;
		dir.A[i][i+5]=-1;
	}
	for(int i=0;i<10;++i)
	{
		dir.A[i+3][i+10]=1;
	}
	dir.A[0][3]=1;
	dir.A[1][3]=-1;
	dir.A[2][3]=1;
	dir.A[3][3]=-1;
	dir.A[4][3]=1;
	dir.A[5][3]=-1;
	dir.A[6][3]=1;
	dir.A[7][3]=0;
	dir.A[8][3]=1;
	dir.A[9][3]=-1;
	dir.A[10][3]=1;
	dir.A[11][3]=-1;
	dir.A[12][3]=0;
	
	dir.A[0][8]=-1;
	dir.A[1][8]=1;
	dir.A[2][8]=-1;
	dir.A[3][8]=1;
	dir.A[4][8]=-1;
	dir.A[5][8]=1;
	dir.A[6][8]=-1;
	dir.A[7][8]=0;
	dir.A[8][8]=-1;
	dir.A[9][8]=1;
	dir.A[10][8]=-1;
	dir.A[11][8]=1;
	dir.A[12][8]=0;
	memset(dir.b, 0, 13U * sizeof(double));
	for(int i=0;i<4;++i)
	{
		dir.b[i+3]=umax[i];
		dir.b[i+8]=-umin[i];
	}
	dir.b[7]=50;
	for(int i=0;i<3;++i)
	{
		dir.Ad5[i]  = -v[i];
		dir.Ad10[i] =  v[i];
	}
	for(int i=0;i<13;++i)
	{
		double temp1=0;
		double temp2=0;
		for(int k=0;k<13;++k)
		{
			temp1 += P_inv[i][k]*dir.Ad5[k];
			temp2 += P_inv[i][k]*dir.Ad10[k];
		}
		dir.A[i][4]=temp1;
		dir.A[i][9]=temp2;
	}
	memset(dir.basis, 0, 13U * sizeof(int));
	for(int i=0;i<13;++i)
	{
		if(i<3)
			dir.basis[i]  = i;
		else
			dir.basis[i]  = i+7;
	}
	memset(dir.c, 0, 20U * sizeof(double));
	dir.c[4]=-1.0f;
	dir.c[9]= 1.0f;
	dir.L=0;
	dir.z=0;
	dir.iters=0;
	dir.e=0;
	//循环计算单纯形表
	while(check_c(dir.c, &(dir.e)))
	{
		
		dir.theta_min=DBL_MAX;
		for(int i=0;i<13;++i)
		{
			if(dir.A[i][dir.e]>0)
			{
				dir.temp=dir.b[i]/dir.A[i][dir.e];
				if(dir.temp<dir.theta_min)
				{
					dir.theta_min=dir.temp;
					dir.L=i;
				}
			}				
		}
		if(dir.theta_min==DBL_MAX)
			break;
		else
		{
			double xishu=dir.A[dir.L][dir.e];
			dir.b[dir.L]=dir.b[dir.L]/xishu;
			for(int i=0;i<20;++i)
			{
				dir.A[dir.L][i]  = dir.A[dir.L][i] / xishu;
			}
			
			for(int i=0;i<13;++i)
			{
				if(i!=dir.L)
				{
					xishu=dir.A[i][dir.e];
					dir.b[i]=dir.b[i]-xishu*dir.b[dir.L];
					for(int j=0;j<20;++j)
					{
						dir.A[i][j]=dir.A[i][j]-xishu*dir.A[dir.L][j];
					}
				}
			}
			dir.z=dir.z-dir.c[dir.e]*dir.b[dir.L];
			xishu=dir.c[dir.e];
			for(int i=0;i<20;++i)
			{
				dir.c[i]  = dir.c[i] - xishu* dir.A[dir.L][i];
			}
			dir.basis[dir.L]=dir.e;
		}
		dir.iters +=  1;
	}
	memset(dir.x, 0, 20U * sizeof(double));
	for(int i=0;i<13;++i)
	{
		dir.x[dir.basis[i]]  = dir.b[i];
	}
	for(int i=0;i<4;++i)
	{
		u[i]  = dir.x[i]-dir.x[i+5];
		if(dir.z>1)
			u[i]  /=dir.z;
	}
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void dir_alloc_mch_initialize(void)
{
	dir.p_limits= mot->p_limits;//单位：度
	for(int i=0;i<4;i++)
	{
		dir.u[i]=0;
		dir.umin[i]=-dir.p_limits * D2R;//单位：弧度
		dir.umax[i]= dir.p_limits * D2R;
	}
}


//----------------------------------------------------------by mch
bool check_limits(double u[4], double p_limits)
{
	double rad_limits_min = -p_limits * 3.1415926535897931f / 180.0f;
	double rad_limits_max =  p_limits * 3.1415926535897931f / 180.0f;
	if(u[0]<rad_limits_max && u[0]>rad_limits_min
	   && u[1]<rad_limits_max && u[1]>rad_limits_min
	   && u[2]<rad_limits_max && u[2]>rad_limits_min
	   && u[3]<rad_limits_max && u[3]>rad_limits_min)
	{
		return 1;
	}
	else
		return 0;
}
void two_dir_alloc_mch(double v_T[3], double v_D[3], double p_limits, double u[4])
{
	// v_T	扰动补偿
	// v_D  动态力矩
	// p_limits  舵机最大偏转角度（度）
	// u  舵量（弧度）
	double v[3];
	double umin[4];
	double umax[4];
	double umin_res[4];
	double umax_res[4];
	double u_v[4];
	double u_v_T[4];
	double u_v_D[4];
	int i;
	for (i = 0; i < 4; i++) 
	{
		umin[i] = -p_limits * 3.1415926535897931f / 180.0f;  //转弧度制
		umax[i] = p_limits * 3.1415926535897931f / 180.0f;
		umin_res[i] = -p_limits * 3.1415926535897931f / 180.0f;
		umax_res[i] = p_limits * 3.1415926535897931f / 180.0f;
		u_v[i]=0.0f;
		u_v_T[i]=0.0f;
		u_v_D[i]=0.0f;
  }
	for(int i=0;i<3;i++)
	{
		v[i]=v_T[i]+v_D[i];
	}
	dir_alloc_mch(v, umin, umax, u_v);  // 先计算合力矩所需舵量
	if(check_limits(u_v, p_limits))  //若舵量可以满足则直接使用
	{
		for (i = 0; i < 4; i++) 
		{
			u[i]=u_v[i];
		}
	}
	else	//否则再计算扰动所需舵量
	{
		dir_alloc_mch(v_T, umin, umax, u_v_T);
		if(check_limits(u_v_T, p_limits))	//若扰动可满足，合力矩不能满足，则进行两次分配
		{
			for (i = 0; i < 4; i++) 
			{
				umin_res[i]=umin[i]-u_v_T[i];
				umax_res[i]=umax[i]-u_v_T[i];
			}
			dir_alloc_mch(v_D, umin_res, umax_res, u_v_D);
			for (i = 0; i < 4; i++) 
			{
				u[i]=u_v_T[i]+u_v_D[i];
			}
		}
		else	//扰动也不能满足，可以按照合力矩进行分配，也可以按照扰动补偿进行
		{
			for (i = 0; i < 4; i++) 
			{
				u[i]=u_v[i];
			}
		}
	}
}
//-------------------------------------------------------------------------------
/*
 * File trailer for dir_alloc_mch.c
 *
 * [EOF]
 */
