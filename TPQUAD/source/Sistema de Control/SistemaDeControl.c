#include "MatrixOps.h"
#include "SistemaDeControl.h"
#include <stdint.h>
#include <string.h>

#define COEFF_POLY {-3.1292, 3.3299, 0.1131}
#define MAX_INTEGRAL_ERROR 1    // TODO
static const double coeffPoly[3] = COEFF_POLY;

static double lastIntegrateError[ROWS_INTEGRATOR_ERROR_VECTOR][1];
static double poly(double Force);

/*
 *  newStates un vector de [phi_k, theta_k]
 */
void integrateError(double newStates[ROWS_INTEGRATOR_ERROR_VECTOR][1], double reference[ROWS_INTEGRATOR_ERROR_VECTOR][1],
					double Ts, double output[ROWS_INTEGRATOR_ERROR_VECTOR][1]){

	double sub[ROWS_INTEGRATOR_ERROR_VECTOR][1];
	matrix_add_sub(ROWS_INTEGRATOR_ERROR_VECTOR, 1, newStates, '-', reference, sub);
	scalar_mult(ROWS_INTEGRATOR_ERROR_VECTOR, 1, Ts, sub);
	matrix_add_sub(ROWS_INTEGRATOR_ERROR_VECTOR, 1, lastIntegrateError, '+', sub, output);
	memcpy(lastIntegrateError, output, sizeof(double)*ROWS_INTEGRATOR_ERROR_VECTOR);
}

void proportionalError(double newStates[ROWS_PROPORTIONAL_ERROR_VECTOR][1], double reference[ROWS_PROPORTIONAL_ERROR_VECTOR][1],
					   double output[ROWS_PROPORTIONAL_ERROR_VECTOR][1])
{
	matrix_add_sub(ROWS_PROPORTIONAL_ERROR_VECTOR, 1, newStates, '-', reference, output);
}

void denormalized_Kx_U_Values(double Kx[KX_ROWS][KX_COLUMNS], double input[ROWS_PROPORTIONAL_ERROR_VECTOR][1], double output[4][1]){
	matrix_mult(KX_ROWS, KX_COLUMNS, 1, Kx, input, output);
}
void denormalized_Ki_U_Values(double Ki[KI_ROWS][KI_COLUMNS], double input[ROWS_INTEGRATOR_ERROR_VECTOR][1], double output[4][1]){
	matrix_mult(KI_ROWS, KI_COLUMNS, 1, Ki, input, output);
}

void denormalized_U_total(double outKx[KX_ROWS][1], double outKi[KI_ROWS][1], double output[KX_ROWS][1]){
	scalar_mult(KX_ROWS, 1, -1, outKx);
	matrix_add_sub(KX_ROWS, 1, outKx, '-', outKi, output);
}



void U2PWM(double U[KX_ROWS][1], double MotorsPWM[4]){
/*
 	 U[0] = F1 + F2 + F3 + F4        (U1)
 	 U[1] = F4 - F2				     (U2)
 	 U[2] = F3 - F1				     (U3)
 	 U[3] = c*(F4 + F2 - F1 - F3)    (U4)
*/
	double c = 10; // ????????????????????????????????????
	double F1 = -(U[3][0] - U[0][0]*c + 2*U[2][0]*c)/(4*c);
	double F2 = (U[3][0] + U[0][0]*c - 2*U[1][0]*c)/(4*c);
	double F3 = (U[0][0]*c - U[3][0] + 2*U[2][0]*c)/(4*c);
	double F4 = (U[3][0] + U[0][0]*c + 2*U[1][0]*c)/(4*c);

	// Mapping a motores nuestros
	/*
	 	 M1* -> M2
	 	 M4* -> M3
	 	 M2* -> M1
	 	 M3* -> M4
	*/
	MotorsPWM[0] = poly(F2) < 0.0 || poly(F2) > 1.0 ? MotorsPWM[0] : poly(F2);
	MotorsPWM[1] = poly(F1) < 0.0 || poly(F1) > 1.0 ? MotorsPWM[1] : poly(F1);
	MotorsPWM[2] = poly(F4) < 0.0 || poly(F4) > 1.0 ? MotorsPWM[2] : poly(F4);
	MotorsPWM[3] = poly(F3) < 0.0 || poly(F3) > 1.0 ? MotorsPWM[3] : poly(F3);
}

static double poly(double Force){
	return coeffPoly[0]*Force*Force + coeffPoly[1]*Force + coeffPoly[2];
}
