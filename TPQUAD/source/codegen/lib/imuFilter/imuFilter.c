/*
 * File: imuFilter.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Jan-2023 16:28:27
 */

/* Include Files */
#include "imuFilter.h"
#include "IMUFilterBase.h"
#include "imuFilter_internal_types.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
}

/*
 * Arguments    : double freqInput
 *                const double AccMatrix[3]
 *                const double GyroMatrix[3]
 *                double angles[3]
 * Return Type  : void
 */
static imufilter FUSE;
static double rotm[9];
static double accelMeasNoiseVar;
static double r_data;
static double y_data;
void imuFilter(double freqInput, const double AccMatrix[3],
               const double GyroMatrix[3], double angles[3])
{
	if(FUSE.isInitialized != 1){
		static const double dv[81] = {
		  6.0923483957341713E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  6.0923483957341713E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  6.0923483957341713E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  7.6154354946677142E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  7.6154354946677142E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  7.6154354946677142E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		  0.0096236100000000012};

		int i;
		FUSE.AccelerometerNoise = 0.0001924722;
		FUSE.GyroscopeNoise = 9.1385E-5;
		FUSE.GyroscopeDriftNoise = 3.0462E-13;
		FUSE.LinearAccelerationNoise = 0.0096236100000000012;
		FUSE.LinearAccelerationDecayFactor = 0.5;
		FUSE.SampleRate = freqInput;
		FUSE.isInitialized = 1;
		FUSE.pInputPrototype[0] = AccMatrix[0];
		FUSE.pInputPrototype[1] = AccMatrix[1];
		FUSE.pInputPrototype[2] = AccMatrix[2];
		FUSE.pSensorPeriod = 1.0 / freqInput;
		FUSE.pKalmanPeriod = FUSE.pSensorPeriod;
		FUSE.TunablePropsChanged = false;
		FUSE.pOrientPost.a = 1.0;
		FUSE.pOrientPost.b = 0.0;
		FUSE.pOrientPost.c = 0.0;
		FUSE.pOrientPost.d = 0.0;
		FUSE.pGyroOffset[0] = 0.0;
		FUSE.pGyroOffset[1] = 0.0;
		FUSE.pGyroOffset[2] = 0.0;
		accelMeasNoiseVar = 0.0098160822000000012 + FUSE.pKalmanPeriod *
													  FUSE.pKalmanPeriod *
													  9.138500030462E-5;
		memset(&rotm[0], 0, 9U * sizeof(double));
		rotm[0] = 1.0;
		rotm[4] = 1.0;
		rotm[8] = 1.0;
		for (i = 0; i < 9; i++) {
			FUSE.pQv[i] = accelMeasNoiseVar * rotm[i];
		}
		memcpy(&FUSE.pQw[0], &dv[0], 81U * sizeof(double));
		FUSE.pLinAccelPost[0] = 0.0;
		FUSE.pLinAccelPost[1] = 0.0;
		FUSE.pLinAccelPost[2] = 0.0;
		FUSE.pFirstTime = true;
	}
	IMUFilterBase_stepImpl(&FUSE, AccMatrix, GyroMatrix, rotm);
	accelMeasNoiseVar = sqrt(rotm[0] * rotm[0] + rotm[1] * rotm[1]);
	angles[0] = rt_atan2d_snf(rotm[5], rotm[8]);
	angles[1] = rt_atan2d_snf(-rotm[2], accelMeasNoiseVar);
	angles[2] = rt_atan2d_snf(rotm[1], rotm[0]);
	if (accelMeasNoiseVar < 2.2204460492503131E-15) {
		y_data = -rotm[7];
		r_data = rt_atan2d_snf(y_data, rotm[4]);
		y_data = -rotm[2];
		angles[0] = r_data;
		r_data = rt_atan2d_snf(y_data, accelMeasNoiseVar);
		angles[1] = r_data;
		angles[2] = 0.0;
	}
	accelMeasNoiseVar = angles[0];
	angles[0] = angles[2] * 180.0 / 3.1415926535897931;
	angles[1] = angles[1] * 180.0 / 3.1415926535897931;
	angles[2] = accelMeasNoiseVar * 180.0 / 3.1415926535897931;
}

/*
 * File trailer for imuFilter.c
 *
 * [EOF]
 */
