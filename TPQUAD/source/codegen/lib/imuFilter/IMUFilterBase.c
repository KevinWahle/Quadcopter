/*
 * File: IMUFilterBase.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Jan-2023 16:28:27
 */

/* Include Files */
#include "IMUFilterBase.h"
#include "imuFilter_internal_types.h"
#include "quaternion.h"
#include "rotmat.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : imufilter *obj
 *                const double accelIn[3]
 *                const double gyroIn[3]
 *                double orientOut[9]
 * Return Type  : void
 */
void IMUFilterBase_stepImpl(imufilter *obj, const double accelIn[3],
                            const double gyroIn[3], double orientOut[9])
{
  static const signed char iv[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  quaternion qerr;
  double Ppost[81];
  double H[27];
  double b_y[27];
  double tmp_tmp[27];
  double y_tmp[27];
  double Rpost[9];
  double b_h1[9];
  double h1[9];
  double Reast[3];
  double a21;
  double deltaq_c;
  double deltaq_d;
  double maxval;
  double xa;
  double xb;
  double xc;
  int H_tmp;
  int b_H_tmp;
  int k;
  int r1;
  int r2;
  int r3;
  int xpageoffset;
  boolean_T b[9];
  boolean_T y[3];
  boolean_T exitg1;
  boolean_T nanPageIdx;
  if (obj->pFirstTime) {
    Reast[0] = accelIn[1] * 0.0 - accelIn[2] * 0.0;
    Reast[1] = accelIn[2] - accelIn[0] * 0.0;
    Reast[2] = accelIn[0] * 0.0 - accelIn[1];
    Rpost[6] = accelIn[0];
    Rpost[3] = Reast[0];
    Rpost[7] = accelIn[1];
    Rpost[4] = Reast[1];
    Rpost[8] = accelIn[2];
    Rpost[5] = Reast[2];
    Rpost[0] = Reast[1] * accelIn[2] - accelIn[1] * Reast[2];
    Rpost[1] = accelIn[0] * Reast[2] - Reast[0] * accelIn[2];
    Rpost[2] = Reast[0] * accelIn[1] - accelIn[0] * Reast[1];
    for (r2 = 0; r2 < 9; r2++) {
      maxval = Rpost[r2];
      h1[r2] = maxval * maxval;
    }
    for (r1 = 0; r1 < 3; r1++) {
      xpageoffset = r1 * 3;
      Reast[r1] =
          sqrt((h1[xpageoffset] + h1[xpageoffset + 1]) + h1[xpageoffset + 2]);
    }
    memcpy(&h1[0], &Rpost[0], 9U * sizeof(double));
    for (k = 0; k < 3; k++) {
      maxval = Reast[k];
      Rpost[3 * k] = h1[3 * k] / maxval;
      xpageoffset = 3 * k + 1;
      Rpost[xpageoffset] = h1[xpageoffset] / maxval;
      xpageoffset = 3 * k + 2;
      Rpost[xpageoffset] = h1[xpageoffset] / maxval;
    }
    for (r2 = 0; r2 < 9; r2++) {
      b[r2] = rtIsNaN(Rpost[r2]);
    }
    y[0] = false;
    y[1] = false;
    y[2] = false;
    xpageoffset = 1;
    exitg1 = false;
    while ((!exitg1) && (xpageoffset <= 3)) {
      if (!b[xpageoffset - 1]) {
        xpageoffset++;
      } else {
        y[0] = true;
        exitg1 = true;
      }
    }
    xpageoffset = 4;
    exitg1 = false;
    while ((!exitg1) && (xpageoffset <= 6)) {
      if (!b[xpageoffset - 1]) {
        xpageoffset++;
      } else {
        y[1] = true;
        exitg1 = true;
      }
    }
    xpageoffset = 7;
    exitg1 = false;
    while ((!exitg1) && (xpageoffset <= 9)) {
      if (!b[xpageoffset - 1]) {
        xpageoffset++;
      } else {
        y[2] = true;
        exitg1 = true;
      }
    }
    nanPageIdx = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 3)) {
      if (!y[k]) {
        k++;
      } else {
        nanPageIdx = true;
        exitg1 = true;
      }
    }
    if (nanPageIdx) {
      memset(&Rpost[0], 0, 9U * sizeof(double));
      Rpost[0] = 1.0;
      Rpost[4] = 1.0;
      Rpost[8] = 1.0;
    }
    obj->pFirstTime = false;
    quaternion_quaternion(Rpost, &obj->pOrientPost.a, &obj->pOrientPost.b,
                          &obj->pOrientPost.c, &obj->pOrientPost.d);
  }
  qerr = obj->pOrientPost;
  Reast[0] = (gyroIn[0] - obj->pGyroOffset[0]) * obj->pSensorPeriod;
  Reast[1] = (gyroIn[1] - obj->pGyroOffset[1]) * obj->pSensorPeriod;
  Reast[2] = (gyroIn[2] - obj->pGyroOffset[2]) * obj->pSensorPeriod;
  b_quaternion_quaternion(Reast, &maxval, &a21, &deltaq_c, &deltaq_d);
  xa = qerr.a;
  xb = qerr.b;
  xc = qerr.c;
  qerr.a = ((qerr.a * maxval - qerr.b * a21) - qerr.c * deltaq_c) -
           qerr.d * deltaq_d;
  qerr.b =
      ((xa * a21 + qerr.b * maxval) + qerr.c * deltaq_d) - qerr.d * deltaq_c;
  qerr.c = ((xa * deltaq_c - xb * deltaq_d) + qerr.c * maxval) + qerr.d * a21;
  qerr.d = ((xa * deltaq_d + xb * deltaq_c) - xc * a21) + qerr.d * maxval;
  if (qerr.a < 0.0) {
    qerr.a = -qerr.a;
    qerr.b = -qerr.b;
    qerr.c = -qerr.c;
    qerr.d = -qerr.d;
  }
  obj->pOrientPrior = qerr;
  quaternionBase_rotmat(obj->pOrientPrior.a, obj->pOrientPrior.b,
                        obj->pOrientPrior.c, obj->pOrientPrior.d, Rpost);
  maxval = obj->LinearAccelerationDecayFactor;
  obj->pLinAccelPrior[0] = maxval * obj->pLinAccelPost[0];
  obj->pLinAccelPrior[1] = maxval * obj->pLinAccelPost[1];
  obj->pLinAccelPrior[2] = maxval * obj->pLinAccelPost[2];
  memset(&h1[0], 0, 9U * sizeof(double));
  h1[3] = Rpost[8];
  h1[6] = -Rpost[7];
  h1[7] = Rpost[6];
  for (r2 = 0; r2 < 3; r2++) {
    b_h1[3 * r2] = h1[3 * r2];
    xpageoffset = 3 * r2 + 1;
    b_h1[xpageoffset] = h1[xpageoffset] - h1[r2 + 3];
    xpageoffset = 3 * r2 + 2;
    b_h1[xpageoffset] = h1[xpageoffset] - h1[r2 + 6];
  }
  memcpy(&h1[0], &b_h1[0], 9U * sizeof(double));
  for (r2 = 0; r2 < 3; r2++) {
    maxval = h1[3 * r2];
    H[3 * r2] = maxval;
    H_tmp = 3 * (r2 + 3);
    H[H_tmp] = -maxval * obj->pKalmanPeriod;
    b_H_tmp = 3 * (r2 + 6);
    H[b_H_tmp] = iv[3 * r2];
    r1 = 3 * r2 + 1;
    maxval = h1[r1];
    H[r1] = maxval;
    H[H_tmp + 1] = -maxval * obj->pKalmanPeriod;
    H[b_H_tmp + 1] = iv[r1];
    r1 = 3 * r2 + 2;
    maxval = h1[r1];
    H[r1] = maxval;
    H[H_tmp + 2] = -maxval * obj->pKalmanPeriod;
    H[b_H_tmp + 2] = iv[r1];
  }
  for (r2 = 0; r2 < 3; r2++) {
    for (r1 = 0; r1 < 9; r1++) {
      maxval = 0.0;
      for (xpageoffset = 0; xpageoffset < 9; xpageoffset++) {
        maxval += H[r2 + 3 * xpageoffset] * obj->pQw[xpageoffset + 9 * r1];
      }
      xpageoffset = r2 + 3 * r1;
      tmp_tmp[xpageoffset] = maxval;
      y_tmp[r1 + 9 * r2] = H[xpageoffset];
    }
  }
  for (r2 = 0; r2 < 3; r2++) {
    for (r1 = 0; r1 < 3; r1++) {
      maxval = 0.0;
      for (xpageoffset = 0; xpageoffset < 9; xpageoffset++) {
        maxval += tmp_tmp[r1 + 3 * xpageoffset] * y_tmp[xpageoffset + 9 * r2];
      }
      h1[r2 + 3 * r1] = maxval + obj->pQv[r1 + 3 * r2];
    }
  }
  for (r2 = 0; r2 < 9; r2++) {
    for (r1 = 0; r1 < 3; r1++) {
      maxval = 0.0;
      for (xpageoffset = 0; xpageoffset < 9; xpageoffset++) {
        maxval += obj->pQw[r2 + 9 * xpageoffset] * y_tmp[xpageoffset + 9 * r1];
      }
      b_y[r2 + 9 * r1] = maxval;
    }
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(h1[0]);
  a21 = fabs(h1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabs(h1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  h1[r2] /= h1[r1];
  h1[r3] /= h1[r1];
  h1[r2 + 3] -= h1[r2] * h1[r1 + 3];
  h1[r3 + 3] -= h1[r3] * h1[r1 + 3];
  h1[r2 + 6] -= h1[r2] * h1[r1 + 6];
  h1[r3 + 6] -= h1[r3] * h1[r1 + 6];
  if (fabs(h1[r3 + 3]) > fabs(h1[r2 + 3])) {
    xpageoffset = r2;
    r2 = r3;
    r3 = xpageoffset;
  }
  h1[r3 + 3] /= h1[r2 + 3];
  h1[r3 + 6] -= h1[r3 + 3] * h1[r2 + 6];
  for (k = 0; k < 9; k++) {
    H_tmp = k + 9 * r1;
    H[H_tmp] = b_y[k] / h1[r1];
    b_H_tmp = k + 9 * r2;
    H[b_H_tmp] = b_y[k + 9] - H[H_tmp] * h1[r1 + 3];
    xpageoffset = k + 9 * r3;
    H[xpageoffset] = b_y[k + 18] - H[H_tmp] * h1[r1 + 6];
    H[b_H_tmp] /= h1[r2 + 3];
    H[xpageoffset] -= H[b_H_tmp] * h1[r2 + 6];
    H[xpageoffset] /= h1[r3 + 6];
    H[b_H_tmp] -= H[xpageoffset] * h1[r3 + 3];
    H[H_tmp] -= H[xpageoffset] * h1[r3];
    H[H_tmp] -= H[b_H_tmp] * h1[r2];
  }
  maxval = (accelIn[0] + obj->pLinAccelPrior[0]) - Rpost[6];
  a21 = (accelIn[1] + obj->pLinAccelPrior[1]) - Rpost[7];
  deltaq_c = (accelIn[2] + obj->pLinAccelPrior[2]) - Rpost[8];
  for (r2 = 0; r2 < 9; r2++) {
    h1[r2] = (H[r2] * maxval + H[r2 + 9] * a21) + H[r2 + 18] * deltaq_c;
  }
  b_quaternion_quaternion(*(double(*)[3]) & h1[0], &qerr.a, &qerr.b, &qerr.c,
                          &qerr.d);
  qerr.b = -qerr.b;
  qerr.c = -qerr.c;
  qerr.d = -qerr.d;
  maxval = obj->pOrientPrior.a;
  a21 = obj->pOrientPrior.b;
  deltaq_c = obj->pOrientPrior.c;
  deltaq_d = obj->pOrientPrior.d;
  obj->pOrientPost.a = ((maxval * qerr.a - a21 * qerr.b) - deltaq_c * qerr.c) -
                       deltaq_d * qerr.d;
  obj->pOrientPost.b = ((maxval * qerr.b + a21 * qerr.a) + deltaq_c * qerr.d) -
                       deltaq_d * qerr.c;
  obj->pOrientPost.c = ((maxval * qerr.c - a21 * qerr.d) + deltaq_c * qerr.a) +
                       deltaq_d * qerr.b;
  obj->pOrientPost.d = ((maxval * qerr.d + a21 * qerr.c) - deltaq_c * qerr.b) +
                       deltaq_d * qerr.a;
  if (obj->pOrientPost.a < 0.0) {
    qerr = obj->pOrientPost;
    qerr.a = -qerr.a;
    qerr.b = -qerr.b;
    qerr.c = -qerr.c;
    qerr.d = -qerr.d;
    obj->pOrientPost = qerr;
  }
  qerr = obj->pOrientPost;
  maxval = sqrt(((qerr.a * qerr.a + qerr.b * qerr.b) + qerr.c * qerr.c) +
                qerr.d * qerr.d);
  qerr.a /= maxval;
  qerr.b /= maxval;
  qerr.c /= maxval;
  qerr.d /= maxval;
  obj->pOrientPost = qerr;
  obj->pGyroOffset[0] -= h1[3];
  obj->pLinAccelPost[0] = obj->pLinAccelPrior[0] - h1[6];
  obj->pGyroOffset[1] -= h1[4];
  obj->pLinAccelPost[1] = obj->pLinAccelPrior[1] - h1[7];
  obj->pGyroOffset[2] -= h1[5];
  obj->pLinAccelPost[2] = obj->pLinAccelPrior[2] - h1[8];
  for (r2 = 0; r2 < 9; r2++) {
    maxval = H[r2];
    a21 = H[r2 + 9];
    deltaq_c = H[r2 + 18];
    for (r1 = 0; r1 < 9; r1++) {
      xpageoffset = r2 + 9 * r1;
      Ppost[xpageoffset] =
          obj->pQw[xpageoffset] -
          ((maxval * tmp_tmp[3 * r1] + a21 * tmp_tmp[3 * r1 + 1]) +
           deltaq_c * tmp_tmp[3 * r1 + 2]);
    }
  }
  memset(&obj->pQw[0], 0, 81U * sizeof(double));
  maxval = obj->pKalmanPeriod;
  maxval *= maxval;
  a21 = obj->GyroscopeDriftNoise + obj->GyroscopeNoise;
  obj->pQw[0] = Ppost[0] + maxval * (Ppost[30] + a21);
  obj->pQw[30] = Ppost[30] + obj->GyroscopeDriftNoise;
  obj->pQw[10] = Ppost[10] + maxval * (Ppost[40] + a21);
  obj->pQw[40] = Ppost[40] + obj->GyroscopeDriftNoise;
  obj->pQw[20] = Ppost[20] + maxval * (Ppost[50] + a21);
  obj->pQw[50] = Ppost[50] + obj->GyroscopeDriftNoise;
  maxval = -obj->pKalmanPeriod;
  Reast[0] = maxval * obj->pQw[30];
  Reast[1] = maxval * obj->pQw[40];
  Reast[2] = maxval * obj->pQw[50];
  maxval = obj->LinearAccelerationDecayFactor;
  maxval *= maxval;
  obj->pQw[3] = Reast[0];
  obj->pQw[27] = Reast[0];
  obj->pQw[60] = maxval * Ppost[60] + obj->LinearAccelerationNoise;
  obj->pQw[13] = Reast[1];
  obj->pQw[37] = Reast[1];
  obj->pQw[70] = maxval * Ppost[70] + obj->LinearAccelerationNoise;
  obj->pQw[23] = Reast[2];
  obj->pQw[47] = Reast[2];
  obj->pQw[80] = maxval * Ppost[80] + obj->LinearAccelerationNoise;
  quaternionBase_rotmat(obj->pOrientPost.a, obj->pOrientPost.b,
                        obj->pOrientPost.c, obj->pOrientPost.d, orientOut);
}

/*
 * File trailer for IMUFilterBase.c
 *
 * [EOF]
 */
