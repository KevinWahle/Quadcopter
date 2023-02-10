/*
 * File: IMUFilterBase.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Jan-2023 16:28:27
 */

#ifndef IMUFILTERBASE_H
#define IMUFILTERBASE_H

/* Include Files */
#include "imuFilter_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void IMUFilterBase_stepImpl(imufilter *obj, const double accelIn[3],
                            const double gyroIn[3], double orientOut[9]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for IMUFilterBase.h
 *
 * [EOF]
 */
