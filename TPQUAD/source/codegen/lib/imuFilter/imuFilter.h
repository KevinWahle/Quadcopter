/*
 * File: imuFilter.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Jan-2023 16:28:27
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
extern void imuFilter(double freqInput, const double AccMatrix[3],
                      const double GyroMatrix[3], double angles[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for imuFilter.h
 *
 * [EOF]
 */
