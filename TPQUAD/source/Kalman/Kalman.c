/*
 * Kalman.c
 *
 *  Created on: Jan 27, 2023
 *      Author: BS
 */
#include "Kalman/Kalman.h"

void KalmanRollPitch_init(KalmanRollPitch *kal, double Pinit, double *Q, double *R){

	kal->phi_rad = 0.0f;
	kal->theta_rad = 0.0f;

	kal->P[0] = Pinit; kal->P[1] = 0.0f;
	kal->P[2] = 0.0f;  kal->P[3] = Pinit;

	kal->Q[0] = Q[0];  kal->Q[1] = Q[1];
	kal->R[0] = R[0];  kal->R[1] = R[1]; kal->R[2] = R[2];
}
void KalmanRollPitch_predict(KalmanRollPitch *kal, double *gyr_rps, double T){

	/*Extract measurements*/

	double p = gyr_rps[0];
	double q = gyr_rps[1];
	double r = gyr_rps[2];

	/* Predict */

	double sp = sin(kal->phi_rad);	double cp = cos(kal->phi_rad);
	double tt = tan(kal->theta_rad);

	/* x+ = x- + T*f(x,u) */

	kal->phi_rad = kal->phi_rad + T * (p + tt * (q * sp + r * cp));
	kal->theta_rad = kal->theta_rad + T * ( q * cp - r * sp);

	/* Recompute common trig terms using new state estimates */
		  sp = sin(kal->phi_rad);		  cp = cos(kal->phi_rad);
	double st = sin(kal->theta_rad); double ct = cos(kal->theta_rad); tt = st / ct;

	/* Jacobian of f(x,u) */

	double A[4] = { tt * (q * cp - r * cp), (r * cp + q * sp) * (tt * tt + 1.0f),
				 -(r * cp + q * sp),   0.0f};

	/* Update Covariance matrix  P+ = P- + T * (A*P- + P-*A' + Q) */

	double Ptmp[4] = {T*(kal->Q[0]		+ 2.0f*A[0]*kal->P[0] + A[1]*kal->P[1] + A[1]*kal->P[2]), T*(A[0]*kal->P[1] + A[2]*kal->P[0] + A[1]*kal->P[3] + A[3]*kal->P[1]),
				    T*(A[0]*kal->P[2] + A[2]*kal->P[0] + A[1]*kal->P[3] + A[3]*kal->P[2]), T*(kal->Q[1] + A[2]*kal->P[1] + A[2]*kal->P[2] + 2.0f*A[3]*kal->P[3]) };

	kal->P[0] = kal->P[0] + Ptmp[0]; kal->P[1] = kal->P[1] + Ptmp[1];
	kal->P[2] = kal->P[2] + Ptmp[2]; kal->P[3] = kal->P[3] + Ptmp[3];


}
void KalmanRollPitch_update(KalmanRollPitch *kal, double *acc_mps2){

	/* Extract measurements */
	double ax = acc_mps2[0];
	double ay = acc_mps2[1];
	double az = acc_mps2[2];

	/* Compute common trig terms */
	double sp = sin(kal->phi_rad); double cp = cos(kal->phi_rad);
	double st = sin(kal->theta_rad); double ct = cos(kal->theta_rad);

	/* Output function h(x,u) */
	double h[3] = { g * st ,
				  -g * ct * sp,
				  -g * ct * cp};

	/* Jacobian of h(x, u) */

	double C[6] = { 0.0f, g * ct,
				  -g * cp * ct, g * sp * st,
				   g * sp * ct, g * cp * st};

	/* Kalman gain K =  P * C' / (C * P * C' + R) */
	double G[9] = { kal->P[3]*C[1]*C[1] + kal->R[0], 	C[1]*C[2]*kal->P[2] + C[1]*C[3]*kal->P[3],				                                        C[1]*C[4]*kal->P[2] + C[1]*C[5]*kal->P[3],
				   C[1]*(C[2]*kal->P[1] + C[3]*kal->P[3]), kal->R[1] + C[2]*(C[2]*kal->P[0] + C[3]*kal->P[2]) + C[3]*(C[2]*kal->P[1] + C[3]*kal->P[3]), C[4]*(C[2]*kal->P[0] + C[3]*kal->P[2]) + C[5]*(C[2]*kal->P[1]+ C[3]*kal->P[3]),
				   C[1]*(C[4]*kal->P[1] + C[5]*kal->P[3]), C[2]*(C[4]*kal->P[0] + C[5]*kal->P[2]) + C[3]*(C[4]*kal->P[1] + C[5]*kal->P[3]),				kal->R[2] + C[4]*(C[4]*kal->P[0] + C[5]*kal->P[2]) + C[5]*(C[4]*kal->P[1] + C[5]*kal->P[3])};

	double Gdetinv = 1.0f / (G[0]*G[4]*G[8] - G[0]*G[5]*G[7] - G[1]*G[3]*G[8] + G[1]*G[5]*G[6] + G[2]*G[3]*G[7] - G[2]*G[4]*G[6]);

	double Ginv[9] = { Gdetinv * (G[4]*G[8] - G[5]*G[7]), -Gdetinv * (G[1]*G[8] - G[2]*G[7]), Gdetinv * (G[1]*G[5] - G[2]*G[4]),
					 -Gdetinv * (G[3]*G[8] - G[5]*G[6]),  Gdetinv * (G[0]*G[8] - G[2]*G[6]),-Gdetinv * (G[0]*G[5] - G[2]*G[3]),
					  Gdetinv * (G[3]*G[7] - G[4]*G[6]), -Gdetinv * (G[0]*G[7] - G[1]*G[6]), Gdetinv * (G[0]*G[4] - G[1]*G[3]) };

	double K[6] = { Ginv[3]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[6]*(C[4]*kal->P[0] + C[5]*kal->P[1]) + C[1]*Ginv[0]*kal->P[1], Ginv[4]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[7]* (C[4]*kal->P[0] + C[5]*kal->P[1]) + C[1]*Ginv[1]*kal->P[1], Ginv[5]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[8]*(C[4]*kal->P[0] + C[5]*kal->P[1]) + C[1]*Ginv[2]*kal->P[1],
				   Ginv[3]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[6]*(C[4]*kal->P[2] + C[5]*kal->P[3]) + C[1]*Ginv[0]*kal->P[3], Ginv[4]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[7] * (C[4]*kal->P[2] + C[5]*kal->P[3]) + C[1]*Ginv[1]*kal->P[3], Ginv[5]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[8]*(C[4]*kal->P[2] + C[5]*kal->P[3]) + C[1]*Ginv[2]*kal->P[3] };


	/* Update covariance matrix P++= (I - K*C)*P+ */

	double Ptmp[4];

	Ptmp[0] = -kal->P[2]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - kal->P[0]*(C[2]*K[1] + C[4]*K[2] - 1.0f); Ptmp[1] = -kal->P[3]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - kal->P[1]*(C[2]*K[1] + C[4]*K[2] - 1.0f);
	Ptmp[2] = -kal->P[2]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f) - kal->P[0]*(C[2]*K[4] + C[4]*K[5]); Ptmp[3] = -kal->P[3]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f) - kal->P[1]*(C[2]*K[4] + C[4]*K[5]);

	kal->P[0] = kal->P[0] + Ptmp[0]; kal->P[1] = kal->P[1] + Ptmp[1];
	kal->P[2] = kal->P[2] + Ptmp[2]; kal->P[3] = kal->P[3] + Ptmp[3];

	/* Update state estimate x++ = x+ + K*(y-h) */

	kal->phi_rad = kal->phi_rad + K[0] * (ax - h[0]) + K[1] * (ay - h[1]) + K[2] * (az - h[2]);
	kal->theta_rad = kal->theta_rad + K[3] * (ax - h[0]) + K[4] * (ay - h[1]) + K[5] * (az - h[2]);

}











