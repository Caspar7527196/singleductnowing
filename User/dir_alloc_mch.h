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
//¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ªby mch
#include "userlib.h"
#include "motor.h"
#define DBL_MAX 1.79769313486231571e+16
typedef struct
{
	double u[4];//¶æÁ¿£¬µ¥Î»£º»¡¶È
	double umin[4];//×îÐ¡¶æÁ¿£¬µ¥Î»£º»¡¶È
	double umax[4];//×î´ó¶æÁ¿£¬µ¥Î»£º»¡¶È
	double p_limits;//¶æ»ú·ùÖµ£¬µ¥Î»£º¶È
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
//¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
/* Function Declarations */
extern void dir_alloc_mch(double v[3], double umin[4], double
  umax[4], double u[4]);
extern void dir_alloc_mch_initialize(void);
//¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ªby mch
extern sDir dir;
extern bool check_limits(double u[4], double p_limits);
extern void two_dir_alloc_mch(double v_T[3], double v_D[3], double p_limits, double u[4]);
//¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
#endif

/*
 * File trailer for dir_alloc_mch.h
 *
 * [EOF]
 */
