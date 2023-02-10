/*
 * File: _coder_imuFilter_api.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Jan-2023 19:11:51
 */

#ifndef _CODER_IMUFILTER_API_H
#define _CODER_IMUFILTER_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void imuFilter(real_T B[9], real_T P[9], real_T C[9], real_T A[9]);

void imuFilter_api(const mxArray *const prhs[3], const mxArray **plhs);

void imuFilter_atexit(void);

void imuFilter_initialize(void);

void imuFilter_terminate(void);

void imuFilter_xil_shutdown(void);

void imuFilter_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_imuFilter_api.h
 *
 * [EOF]
 */
