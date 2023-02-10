/*
 * File: imuFilter.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 29-Jan-2023 19:11:51
 */

/* Include Files */
#include "imuFilter.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double B[9]
 *                const double P[9]
 *                const double C[9]
 *                double A[9]
 * Return Type  : void
 */
void imuFilter(const double B[9], const double P[9], const double C[9],
               double A[9])
{
  double a[9];
  double b_B[9];
  double x[9];
  double absx11;
  double absx21;
  double absx31;
  int itmp;
  int p1;
  int p2;
  int p3;
  memcpy(&x[0], &P[0], 9U * sizeof(double));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = fabs(P[0]);
  absx21 = fabs(P[1]);
  absx31 = fabs(P[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    x[0] = P[1];
    x[1] = P[0];
    x[3] = P[4];
    x[4] = P[3];
    x[6] = P[7];
    x[7] = P[6];
  } else if (absx31 > absx11) {
    p1 = 6;
    p3 = 0;
    x[0] = P[2];
    x[2] = P[0];
    x[3] = P[5];
    x[5] = P[3];
    x[6] = P[8];
    x[8] = P[6];
  }
  x[1] /= x[0];
  x[2] /= x[0];
  x[4] -= x[1] * x[3];
  x[5] -= x[2] * x[3];
  x[7] -= x[1] * x[6];
  x[8] -= x[2] * x[6];
  if (fabs(x[5]) > fabs(x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = x[1];
    x[1] = x[2];
    x[2] = absx11;
    absx11 = x[4];
    x[4] = x[5];
    x[5] = absx11;
    absx11 = x[7];
    x[7] = x[8];
    x[8] = absx11;
  }
  x[5] /= x[4];
  x[8] -= x[5] * x[7];
  absx11 = (x[1] * x[5] - x[2]) / x[8];
  absx21 = -(x[1] + x[7] * absx11) / x[4];
  a[p1] = ((1.0 - x[3] * absx21) - x[6] * absx11) / x[0];
  a[p1 + 1] = absx21;
  a[p1 + 2] = absx11;
  absx11 = -x[5] / x[8];
  absx21 = (1.0 - x[7] * absx11) / x[4];
  a[p2] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  a[p2 + 1] = absx21;
  a[p2 + 2] = absx11;
  absx11 = 1.0 / x[8];
  absx21 = -x[7] * absx11 / x[4];
  a[p3] = -(x[3] * absx21 + x[6] * absx11) / x[0];
  a[p3 + 1] = absx21;
  a[p3 + 2] = absx11;
  for (p1 = 0; p1 < 3; p1++) {
    absx11 = B[p1];
    absx21 = B[p1 + 3];
    absx31 = B[p1 + 6];
    for (p2 = 0; p2 < 3; p2++) {
      x[p1 + 3 * p2] = (absx11 * P[3 * p2] + absx21 * P[3 * p2 + 1]) +
                       absx31 * P[3 * p2 + 2];
    }
    absx11 = x[p1];
    absx21 = x[p1 + 3];
    absx31 = x[p1 + 6];
    for (p2 = 0; p2 < 3; p2++) {
      b_B[p1 + 3 * p2] = (absx11 * C[3 * p2] + absx21 * C[3 * p2 + 1]) +
                         absx31 * C[3 * p2 + 2];
    }
  }
  for (p1 = 0; p1 < 3; p1++) {
    absx11 = a[p1];
    absx21 = a[p1 + 3];
    absx31 = a[p1 + 6];
    for (p2 = 0; p2 < 3; p2++) {
      A[p1 + 3 * p2] = (absx11 * b_B[3 * p2] + absx21 * b_B[3 * p2 + 1]) +
                       absx31 * b_B[3 * p2 + 2];
    }
  }
}

/*
 * File trailer for imuFilter.c
 *
 * [EOF]
 */
