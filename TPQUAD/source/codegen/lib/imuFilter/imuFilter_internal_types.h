/*
 * File: imuFilter_internal_types.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Jan-2023 16:28:27
 */

#ifndef IMUFILTER_INTERNAL_TYPES_H
#define IMUFILTER_INTERNAL_TYPES_H

/* Include Files */
#include "imuFilter_types.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_quaternion
#define typedef_quaternion
typedef struct {
  double a;
  double b;
  double c;
  double d;
} quaternion;
#endif /* typedef_quaternion */

#ifndef typedef_cell_wrap_3
#define typedef_cell_wrap_3
typedef struct {
  unsigned int f1[8];
} cell_wrap_3;
#endif /* typedef_cell_wrap_3 */

#ifndef typedef_imufilter
#define typedef_imufilter
typedef struct {
  int isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_3 inputVarSize[2];
  double SampleRate;
  double AccelerometerNoise;
  double GyroscopeNoise;
  double GyroscopeDriftNoise;
  double LinearAccelerationNoise;
  double LinearAccelerationDecayFactor;
  double pQw[81];
  double pQv[9];
  quaternion pOrientPost;
  quaternion pOrientPrior;
  boolean_T pFirstTime;
  double pSensorPeriod;
  double pKalmanPeriod;
  double pGyroOffset[3];
  double pLinAccelPrior[3];
  double pLinAccelPost[3];
  double pInputPrototype[3];
} imufilter;
#endif /* typedef_imufilter */

#endif
/*
 * File trailer for imuFilter_internal_types.h
 *
 * [EOF]
 */
