/*
 * File: imuFilter.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Jan-2023 19:11:51
 */

#ifndef IMUFILTER_H
#define IMUFILTER_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void imuFilter(const double B[9], const double P[9], const double C[9],
                      double A[9]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for imuFilter.h
 *
 * [EOF]
 */
