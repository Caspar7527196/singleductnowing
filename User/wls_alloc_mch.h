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
//¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ªby mch
#include "userlib.h"
#include "motor.h"
typedef struct
{
	double u[4];//¶æÁ¿£¬µ¥Î»£º»¡¶È
	double p_limits;//¶æ»ú·ùÖµ£¬µ¥Î»£º¶È
	
}sWls;
/* Function Declarations */
extern void wls_alloc_mch(const double v[3], const double p_limits,
  boolean_T v_limits, double u[4]);
extern void wls_alloc_mch_initialize(void);
extern void wls_alloc_mch_terminate(void);
//¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ªby mch
extern sWls wls;
//¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
#endif

/*
 * File trailer for wls_alloc_mch.h
 *
 * [EOF]
 */
