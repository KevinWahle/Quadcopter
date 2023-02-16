#include "MatrixOps.h"
#include <stdint.h>

#define COEFF_POLY {-3.1292, 3.3299, 0.1131}

static const double coeffPoly[3] = COEFF_POLY;

static double lastIntegrateError[ROWS_INTEGRATOR_ERROR_VECTOR];
static double poly(double Force);



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



void U2PWM(double U[KX_ROWS], double MotorsPWM[4]){
/*
 	 U[0] = F1 + F2 + F3 + F4        (U1)
 	 U[1] = F4 - F2				     (U2)
 	 U[2] = F3 - F1				     (U3)
 	 U[3] = c*(F4 + F2 - F1 - F3)    (U4)
*/

	double F1 = -(U[3] - U[0]*c + 2*U[2]*c)/(4*c);
	double F2 = (U[3] + U[0]*c - 2*U[1]*c)/(4*c);
	double F3 = (U[0]*c - U[3] + 2*U[2]*c)/(4*c);
	double F4 = (U[3] + U[0]*c + 2*U[1]*c)/(4*c);

	// Mapping a motores nuestros
	/*
	 	 M1* -> M2
	 	 M4* -> M3
	 	 M2* -> M1
	 	 M3* -> M4
	*/
	MotorsPWM[0] = poly(F2);
	MotorsPWM[1] = poly(F1);
	MotorsPWM[2] = poly(F4);
	MotorsPWM[3] = poly(F3);
}

static double poly(double Force){
	return coeffPoly[0]*Force*Force + coeffPoly[1]*Force + coeffPoly[2];
}
