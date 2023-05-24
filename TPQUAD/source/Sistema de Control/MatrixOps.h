/***************************************************************************//**

 ******************************************************************************/

#ifndef _MATRIX_OPS_H_
#define _MATRIX_OPS_H_
#include <stdint.h>

void matrix_mult(uint8_t m, uint8_t n, uint8_t p, const float A[m][n], const float B[n][p], float C[m][p]);

void matrix_add_sub(uint8_t m, uint8_t n, const float A[m][n], char sign, const float B[m][n], float C[m][n]);

void scalar_mult(uint8_t m, uint8_t n, float scalar, float InOut[m][n]);

#endif // _MATRIX_OPS_H_
