#ifndef __ESO_H
#define __ESO_H



#include "control.h"

#include "userlib.h"

//----------------2019.6 MCH------------------------
typedef struct
{
	char *name;
	double e;
	double ai;
	double beta[3];
	float Dt;
	double kcs;
	double A11;
	double A12;
	double A21;
	double A22;
	double B11;
	double B21;
	double L_1;
	double L_2;
}sESO_CFG;

typedef struct
{
	bool Update;
	float Dt;
	char *name;
	sTIM Tim;  
	u32  Time;
	double e;
	double ai;
	double beta[3];
	double x_k[9];
	double x_n[9];
	double y[3];
	double kD[3][3];
	double kcs;
	double r[3];
	double u[3];
	bool yes;
	//------------------Ö´ÐÐÆ÷²¿·Ö------------------------
	double xa_k[6];
	double xa_n[6];
	double real_output[3];
	double A11;
	double A12;
	double A21;
	double A22;
	double B11;
	double B21;
	double L_1;
	double L_2;
	//------------------------------------------
	
}sESO;

#define ESO_NUM 1

extern sESO eso[ESO_NUM];

void Eso_Init_Para(sESO *ele,sESO_CFG *elecfg);
bool Eso_Init(sESO *ele);
bool Eso_Calc(sESO *ele);

	
#endif
