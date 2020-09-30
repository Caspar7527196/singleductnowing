/*
 * File: dir_alloc_mch.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 24-Nov-2019 20:09:11
 */

#ifndef DIR_ALLOC_MCH_H
#define DIR_ALLOC_MCH_H

/* Include Files */
//#include <stddef.h>
//#include <stdlib.h>
//#include "rtwtypes.h"
//����������������������������������������������������������������by mch
#include "userlib.h"
#include "motor.h"
#define DBL_MAX 1.79769313486231571e+16
typedef struct
{
	double u[4];//��������λ������
	double umin[4];//��С��������λ������
	double umax[4];//����������λ������
	double p_limits;//�����ֵ����λ����
	double theta_min;
	double A[13][20];
double Ad5[13];
double Ad10[13];
double b[13];
double c[20];
int basis[13];
double x[20];
	int L;
	double z;
	double iters;
	int e;
	double temp;
}sDir;
//������������������������������������������������������������������������
/* Function Declarations */
extern void dir_alloc_mch(double v[3], double umin[4], double
  umax[4], double u[4]);
extern void dir_alloc_mch_initialize(void);
//������������������������������������������������������������������������������by mch
extern sDir dir;
extern bool check_limits(double u[4], double p_limits);
extern void two_dir_alloc_mch(double v_T[3], double v_D[3], double p_limits, double u[4]);
//������������������������������������������������������������������������
#endif

/*
 * File trailer for dir_alloc_mch.h
 *
 * [EOF]
 */
