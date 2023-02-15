#include "MatrixOps.h"
#include <stdint.h>

static double lastIntegrateError[ROWS_INTEGRATOR_ERROR_VECTOR];


void integrateError(double newStates[ROWS_INTEGRATOR_ERROR_VECTOR], double reference[ROWS_INTEGRATOR_ERROR_VECTOR],
					double Ts, double output[ROWS_INTEGRATOR_ERROR_VECTOR]){

	double sub[ROWS_INTEGRATOR_ERROR_VECTOR];
	matrix_add_sub(ROWS_INTEGRATOR_ERROR_VECTOR, 1, newStates, '-', reference, sub);
	scalar_mult(ROWS_INTEGRATOR_ERROR_VECTOR, 1, Ts, sub);
	matrix_add_sub(ROWS_INTEGRATOR_ERROR_VECTOR, 1, lastIntegrateError, '+', sub, output);

}

void proportionalError(double newStates[ROWS_PROPORTIONAL_ERROR_VECTOR], double reference[ROWS_PROPORTIONAL_ERROR_VECTOR],
					   double output[ROWS_PROPORTIONAL_ERROR_VECTOR])
{
	matrix_add_sub(ROWS_PROPORTIONAL_ERROR_VECTOR, 1, newStates, '-', reference, output);
}

void denormalized_Kx_U_Values(double input[ROWS_PROPORTIONAL_ERROR_VECTOR], double Kx[KX_ROWS][KX_COLUMNS], double output[4]){
	matrix_mult(KX_ROWS, KX_COLUMNS, 1, Kx, input, output);
}
void denormalized_Ki_U_Values(double input[ROWS_INTEGRATOR_ERROR_VECTOR], double Ki[KI_ROWS][KI_COLUMNS], double output[4]){
	matrix_mult(KI_ROWS, KI_COLUMNS, 1, Ki, input, output);
}

void denormalized_U_total(double outKx[KX_ROWS], double outKi[KI_ROWS], double output[KX_ROWS]){
	double subKx[KX_ROWS];
	scalar_mult(KX_ROWS, 1, -1, subKx);
	matrix_add_sub(KX_ROWS, 1, subKx, '-', outKi, output);
}

void U2PWM(double U[KX_ROWS], double NormalizedU[KX_ROWS]);
