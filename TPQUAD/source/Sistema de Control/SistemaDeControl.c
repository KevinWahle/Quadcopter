#include "MatrixOps.h"
#include "SistemaDeControl.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define COEFF_POLY {-0.0313, 0.3330, 0.1131}
#define TOTAL_N_FORCE_TRIGGER 	1  // Recien a partir de [TOTAL_N_FORCE_TRIGGER]N empieza a prender los motores 
// Defino que el ajuste por integral no puede ser mayor a 1N de diferencia

#define MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL	0.5    // 0.2 Newton
#define MAX_INTEGRAL_ERROR_NEWTORN_YAW 			1  // 1 Newton Heuristico

static const float coeffPoly[3] = COEFF_POLY;

static float lastIntegrateError[ROWS_INTEGRATOR_ERROR_VECTOR][1];
static float poly(float Force);

/*
 *  newStates un vector de [phi_k, theta_k]
 */
bool integrateError(float newStates[ROWS_INTEGRATOR_ERROR_VECTOR][1], float reference[ROWS_INTEGRATOR_ERROR_VECTOR][1],
					float Ts, float output[ROWS_INTEGRATOR_ERROR_VECTOR][1], float KiVal[ROWS_INTEGRATOR_ERROR_VECTOR]){
	bool saturation = false;
	float sub[ROWS_INTEGRATOR_ERROR_VECTOR][1];
	matrix_add_sub(ROWS_INTEGRATOR_ERROR_VECTOR, 1, newStates, '-', reference, sub);
	scalar_mult(ROWS_INTEGRATOR_ERROR_VECTOR, 1, Ts, sub);
	matrix_add_sub(ROWS_INTEGRATOR_ERROR_VECTOR, 1, lastIntegrateError, '+', sub, output);
	if(output[0][0] > MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL/KiVal[0] || output[0][0] < -MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL/KiVal[0]){
		output[0][0] = output[0][0] > 0 ? MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL/KiVal[0] : -MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL/KiVal[0];
		//saturation = true;
	}
	if(output[1][0] > MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL/KiVal[1] || output[1][0] < -MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL/KiVal[1]){
		output[1][0] = output[1][0] > 0 ? MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL/KiVal[1] : -MAX_INTEGRAL_ERROR_NEWTON_THETA_ROLL/KiVal[1];
		//saturation = true;
	}
	if(output[2][0] > MAX_INTEGRAL_ERROR_NEWTORN_YAW/KiVal[2] || output[2][0] < -MAX_INTEGRAL_ERROR_NEWTORN_YAW/KiVal[2]){
		output[2][0] = output[2][0] > 0 ? MAX_INTEGRAL_ERROR_NEWTORN_YAW/KiVal[2] : -MAX_INTEGRAL_ERROR_NEWTORN_YAW/KiVal[2];
		saturation = true;
	}
	memcpy(lastIntegrateError, output, sizeof(float)*ROWS_INTEGRATOR_ERROR_VECTOR);
	return saturation;
}

void proportionalError(float newStates[ROWS_PROPORTIONAL_ERROR_VECTOR][1], float reference[ROWS_PROPORTIONAL_ERROR_VECTOR][1],
					   float output[ROWS_PROPORTIONAL_ERROR_VECTOR][1])
{
	matrix_add_sub(ROWS_PROPORTIONAL_ERROR_VECTOR, 1, newStates, '-', reference, output);
}

void denormalized_Kx_U_Values(float Kx[KX_ROWS][KX_COLUMNS], float input[ROWS_PROPORTIONAL_ERROR_VECTOR][1], float output[4][1]){
	matrix_mult(KX_ROWS, KX_COLUMNS, 1, Kx, input, output);
}
void denormalized_Ki_U_Values(float Ki[KI_ROWS][KI_COLUMNS], float input[ROWS_INTEGRATOR_ERROR_VECTOR][1], float output[4][1]){
	matrix_mult(KI_ROWS, ROWS_INTEGRATOR_ERROR_VECTOR, 1, Ki, input, output);
}

void denormalized_U_total(float outKx[KX_ROWS][1], float outKi[KI_ROWS][1], float output[KX_ROWS][1]){
	scalar_mult(KX_ROWS, 1, -1, outKx);
	matrix_add_sub(KX_ROWS, 1, outKx, '-', outKi, output);
}

void U2PWM(float U[KX_ROWS][1], float MotorsPWM[4]){
/*
 	 U[0] = F1 + F2 + F3 + F4        (U1)
 	 U[1] = F4 - F2				     (U2)
 	 U[2] = F3 - F1				     (U3)
 	 U[3] = c*(F4 + F2 - F1 - F3)    (U4)
*/
/*
	eqn1 = F1 + F2 + F3 + F4 == U1;
	eqn2 = F3 - F1 == U2;
	eqn3 = F4 - F2 == U3; 
	eqn4 = +c*F1 + c*F3 - c*F2 - c*F4 == U4;
*/

	float c = 10; // ????????????????????????????????????
	float F1 = (U[3][0] + U[0][0]*c - 2*U[1][0]*c)/(4*c);
	float F2 = -(U[3][0] - U[0][0]*c + 2*U[2][0]*c)/(4*c);
	float F3 = (U[3][0] + U[0][0]*c + 2*U[1][0]*c)/(4*c);
	float F4 = (U[0][0]*c - U[3][0] + 2*U[2][0]*c)/(4*c);

	// Mapping a motores nuestros
	/*
	 	 M1* -> M2
	 	 M4* -> M3
	 	 M2* -> M1
	 	 M3* -> M4
	*/
	MotorsPWM[0] = poly(F1) < 0.0 || poly(F1) > 1.0 ? ( poly(F1) < 0.0 ? 0.0 : 1.0 ) : poly(F1);
	MotorsPWM[1] = poly(F2) < 0.0 || poly(F2) > 1.0 ? ( poly(F2) < 0.0 ? 0.0 : 1.0 ) : poly(F2);
	MotorsPWM[2] = poly(F3) < 0.0 || poly(F3) > 1.0 ? ( poly(F3) < 0.0 ? 0.0 : 1.0 ) : poly(F3);
	MotorsPWM[3] = poly(F4) < 0.0 || poly(F4) > 1.0 ? ( poly(F4) < 0.0 ? 0.0 : 1.0 ) : poly(F4);

	if(U[0][0] < TOTAL_N_FORCE_TRIGGER)     // apago los motores si el value del joystick throttle esta debajo de cierto nivel 
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			MotorsPWM[i] = 0.0;
		}
	}
}

static float poly(float Force){
	return coeffPoly[0]*Force*Force + coeffPoly[1]*Force + coeffPoly[2];
}
