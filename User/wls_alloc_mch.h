/*
 * File: wls_alloc_mch.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 23-Nov-2019 12:19:19
 */

#ifndef WLS_ALLOC_MCH_H
#define WLS_ALLOC_MCH_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
//����������������������������������������������������������������by mch
#include "userlib.h"
#include "motor.h"
typedef struct
{
	double u[4];//��������λ������
	double p_limits;//�����ֵ����λ����
	
}sWls;
/* Function Declarations */
extern void wls_alloc_mch(const double v[3], const double p_limits,
  boolean_T v_limits, double u[4]);
extern void wls_alloc_mch_initialize(void);
extern void wls_alloc_mch_terminate(void);
//������������������������������������������������������������������������������by mch
extern sWls wls;
//������������������������������������������������������������������������
#endif

/*
 * File trailer for wls_alloc_mch.h
 *
 * [EOF]
 */
