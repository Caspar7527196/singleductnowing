#include "eso.h"
sESO eso[ESO_NUM];
const float inertia[3]={0.025483,0.025504,0.00562};	

void Eso_Init_Para(sESO *ele,sESO_CFG *elecfg)
{
	ele->name = elecfg->name;
	ele->Update = false;
	ele->yes = false;
	ele->Dt = elecfg->Dt;
	ele->e = elecfg->e;
	ele->ai = elecfg->ai;

	ele->A11 = elecfg->A11;
	ele->A12 = elecfg->A12;
	ele->A21 = elecfg->A21;
	ele->A22 = elecfg->A22;
	ele->B11 = elecfg->B11;
	ele->B21 = elecfg->B21;
	ele->L_1 = elecfg->L_1;
	ele->L_2 = elecfg->L_2;
	ele->kcs = elecfg->kcs;
	
	for(u8 i=0;i<3;i++)
	{
		ele->beta[i]=elecfg->beta[i];
	}
}

bool Eso_Init(sESO *ele)
{

	Dprintf("\r\n%s Init...\r\n",ele->name);
	//HAL_Delay(50);    //µÈµØÐÅºÅÀ´ÁÙ
	
	for(u8 i=0;i<9;i++)
	{
		ele->x_k[i]=0;
		ele->x_n[i]=0;
		ele->y[i]=0;
	}

	ele->kD[0][0]=2.0f*ele->L_1*ele->kcs;
	ele->kD[1][1]=2.0f*ele->L_1*ele->kcs;
	ele->kD[2][2]=4.0f*ele->L_2*ele->kcs;
	
	if(Eso_Calc(ele)==false)
	{
		Dprintf("%s Init.[NO]\r\n",ele->name);
		return false;
	}
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}
double nonlinear_eso(double ai,int j,double r)
{
	double x;
	x = r;
	if (r < 0.0) 
	{
		x = -1.0;
	} 
	else if (r > 0.0) 
	{
		x = 1.0;
	} 
	else 
	{
		if (r == 0.0) 
		{
		x = 0.0;
		}
	}
	return x * pow(fabs(r), j * ai - (j - 1.0));
}
void Actuator(double xa_k[6],double xa_n[6],double U[3],double real_U[3])
{
//------------------执行器模型更新----------------------------------
	xa_n[0]=eso->A11*xa_k[0]+eso->A12*xa_k[1]+eso->B11*U[0];
	xa_n[1]=eso->A21*xa_k[0]+eso->A22*xa_k[1]+eso->B21*U[0];
	
	xa_n[2]=eso->A11*xa_k[2]+eso->A12*xa_k[3]+eso->B11*U[1];
	xa_n[3]=eso->A21*xa_k[2]+eso->A22*xa_k[3]+eso->B21*U[1];
	
	xa_n[4]=eso->A11*xa_k[4]+eso->A12*xa_k[5]+eso->B11*U[2];
	xa_n[5]=eso->A21*xa_k[4]+eso->A22*xa_k[5]+eso->B21*U[2];

	real_U[0]=xa_k[0];
	real_U[1]=xa_k[2];
	real_U[2]=xa_k[4];
	
	for(int i=0;i<6;i++)
	{
		xa_k[i]=xa_n[i];
	}
}
void Eso(double x_k[9],double x_n[9],float Ang[3],double y[9],double U[3])
{

//------------------eso更新--------------------------------
//	double kD_inv[3][3];

	for(int i=0;i<3;i++)
	{
		eso->u[i] = eso->kD[i][i]*U[i]/inertia[i];
	}
	for(u8 i=0;i<3;i++)
	{
		eso->r[i]=(Ang[i]-x_k[i])/(pow(eso->e,2.0f));
	}
	
	x_n[0]=x_k[0]+eso->Dt*(x_k[3]+eso->beta[0]*eso->e*nonlinear_eso(eso->ai,1.0f,eso->r[0]));
	x_n[1]=x_k[1]+eso->Dt*(x_k[4]+eso->beta[0]*eso->e*nonlinear_eso(eso->ai,1.0f,eso->r[1]));
	x_n[2]=x_k[2]+eso->Dt*(x_k[5]+eso->beta[0]*eso->e*nonlinear_eso(eso->ai,1.0f,eso->r[2]));
	
	x_n[3]=x_k[3]+eso->Dt*(x_k[6]+eso->beta[1]*nonlinear_eso(eso->ai,2.0f,eso->r[0]) + eso->u[0]);
	x_n[4]=x_k[4]+eso->Dt*(x_k[7]+eso->beta[1]*nonlinear_eso(eso->ai,2.0f,eso->r[1]) + eso->u[1]);
	x_n[5]=x_k[5]+eso->Dt*(x_k[8]+eso->beta[1]*nonlinear_eso(eso->ai,2.0f,eso->r[2]) + eso->u[2]);
	
	x_n[6]=x_k[6]+eso->Dt*(eso->beta[2]*(1.0f/eso->e)*nonlinear_eso(eso->ai,3.0f,eso->r[0]));
	x_n[7]=x_k[7]+eso->Dt*(eso->beta[2]*(1.0f/eso->e)*nonlinear_eso(eso->ai,3.0f,eso->r[1]));
	x_n[8]=x_k[8]+eso->Dt*(eso->beta[2]*(1.0f/eso->e)*nonlinear_eso(eso->ai,3.0f,eso->r[2]));
	
	//matrix_inverse_LU(3,eso->kD,kD_inv);
	for(int i=0;i<3;i++)
	{
		y[i] = -(x_k[i+6]*inertia[i]/eso->kD[i][i]);//扰动
	}
	for(int i=0;i<9;i++)
	{
		x_k[i]=x_n[i];
	}
}
bool Eso_Calc(sESO *ele)
{
	Tim_Calc(&ele->Tim);   
	
	Actuator(ele->xa_k,ele->xa_n,CtrlIO->output,ele->real_output);//  //CtrlIO->output_sat
	Eso(ele->x_k,ele->x_n,ahrs[1].Ang,ele->y,ele->real_output);
	
	ele->Update = true;
	Tim_Calc(&ele->Tim);   
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}


	
