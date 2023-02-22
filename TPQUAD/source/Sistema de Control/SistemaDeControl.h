/***************************************************************************//**

 ******************************************************************************/
#ifndef _SISTEMA_DE_CONTROL_H_
#define _SISTEMA_DE_CONTROL_H_

#include <stdbool.h>

#define ROWS_INTEGRATOR_ERROR_VECTOR 2
#define ROWS_PROPORTIONAL_ERROR_VECTOR 6

#define KX_ROWS 4
#define KX_COLUMNS 6

#define KI_ROWS 4
#define KI_COLUMNS 2

bool integrateError(double newStates[ROWS_INTEGRATOR_ERROR_VECTOR][1], double reference[ROWS_INTEGRATOR_ERROR_VECTOR][1],
					double Ts, double output[ROWS_INTEGRATOR_ERROR_VECTOR][1], double KiVal);

void proportionalError(double newStates[ROWS_PROPORTIONAL_ERROR_VECTOR][1], double reference[ROWS_PROPORTIONAL_ERROR_VECTOR][1],
					   double output[ROWS_PROPORTIONAL_ERROR_VECTOR][1]);

void denormalized_Kx_U_Values(double Kx[KX_ROWS][KX_COLUMNS], double input[ROWS_PROPORTIONAL_ERROR_VECTOR][1], double output[4][1]);
void denormalized_Ki_U_Values(double Ki[KI_ROWS][KI_COLUMNS], double input[ROWS_INTEGRATOR_ERROR_VECTOR][1], double output[4][1]);

void denormalized_U_total(double outKx[KX_ROWS][1], double outKi[KI_ROWS][1], double output[KX_ROWS][1]);

void U2PWM(double U[KX_ROWS][1], double MotorsPWM[4]);

#endif // _SISTEMA_DE_CONTROL_H_
