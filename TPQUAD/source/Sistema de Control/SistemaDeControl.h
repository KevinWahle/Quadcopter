/***************************************************************************//**

 ******************************************************************************/
#ifndef _SISTEMA_DE_CONTROL_H_
#define _SISTEMA_DE_CONTROL_H_

#define ROWS_INTEGRATOR_ERROR_VECTOR 2
#define ROWS_PROPORTIONAL_ERROR_VECTOR 6

#define KX_ROWS 4
#define KX_COLUMNS 6

#define KI_ROWS 4
#define KI_COLUMNS 2

void runControlStep();

void integrateError(double newStates[ROWS_INTEGRATOR_ERROR_VECTOR], double reference[ROWS_INTEGRATOR_ERROR_VECTOR],
					double Ts, double output[ROWS_INTEGRATOR_ERROR_VECTOR]);

void proportionalError(double newStates[ROWS_PROPORTIONAL_ERROR_VECTOR], double reference[ROWS_PROPORTIONAL_ERROR_VECTOR],
					   double output[ROWS_PROPORTIONAL_ERROR_VECTOR]);

void denormalized_Kx_U_Values(double input[ROWS_PROPORTIONAL_ERROR_VECTOR], double Kx[KX_ROWS][KX_COLUMNS], double output[4]);
void denormalized_Ki_U_Values(double input[ROWS_INTEGRATOR_ERROR_VECTOR], double Ki[KI_ROWS][KI_COLUMNS], double output[4]);

void denormalized_U_total(double outKx[KX_ROWS], double outKi[KI_ROWS], double output[KX_ROWS]);

void U2PWM(double U[KX_ROWS], double NormalizedU[KX_ROWS]);

#endif // _SISTEMA_DE_CONTROL_H_
