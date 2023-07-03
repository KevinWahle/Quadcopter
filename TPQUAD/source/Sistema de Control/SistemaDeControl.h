/***************************************************************************//**

 ******************************************************************************/
#ifndef _SISTEMA_DE_CONTROL_H_
#define _SISTEMA_DE_CONTROL_H_

#include <stdbool.h>

#define ROWS_INTEGRATOR_ERROR_VECTOR 3
#define ROWS_PROPORTIONAL_ERROR_VECTOR 6

#define KX_ROWS 4
#define KX_COLUMNS 6

#define KI_ROWS 4
#define KI_COLUMNS 3

bool integrateError(float newStates[ROWS_INTEGRATOR_ERROR_VECTOR][1], float reference[ROWS_INTEGRATOR_ERROR_VECTOR][1],
					float Ts, float output[ROWS_INTEGRATOR_ERROR_VECTOR][1], float KiVal[ROWS_INTEGRATOR_ERROR_VECTOR]);

void proportionalError(float newStates[ROWS_PROPORTIONAL_ERROR_VECTOR][1], float reference[ROWS_PROPORTIONAL_ERROR_VECTOR][1],
					   float output[ROWS_PROPORTIONAL_ERROR_VECTOR][1]);

void denormalized_Kx_U_Values(float Kx[KX_ROWS][KX_COLUMNS], float input[ROWS_PROPORTIONAL_ERROR_VECTOR][1], float output[4][1]);
void denormalized_Ki_U_Values(float Ki[KI_ROWS][KI_COLUMNS], float input[ROWS_INTEGRATOR_ERROR_VECTOR][1], float output[4][1]);

void denormalized_U_total(float outKx[KX_ROWS][1], float outKi[KI_ROWS][1], float output[KX_ROWS][1]);

void U2PWM(float U[KX_ROWS][1], float MotorsPWM[4]);

#endif // _SISTEMA_DE_CONTROL_H_
