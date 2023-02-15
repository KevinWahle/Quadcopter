/***************************************************************************//**

 ******************************************************************************/

#ifndef _MATRIX_OPS_H_
#define _MATRIX_OPS_H_


void matrix_mult(uint8_t m, uint8_t n, uint8_t p, double A[m][n], double B[n][p], double C[m][p]);

void matrix_add_sub(uint8_t m, uint8_t n, double A[m][n], char sign, double B[m][n], double C[m][n]);

void scalar_mult(uint8_t m, uint8_t n, double scalar, double InOut[m][n]);

#endif // _MATRIX_OPS_H_
