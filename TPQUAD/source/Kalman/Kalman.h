#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <math.h>

#define g ((double) 9.81f)

#define KALMAN_P_INIT 0.0001f
#define KALMAN_Q 0.001f//0.001f
#define KALMAN_R 0.01f//0.011f
#define AMOUNT_OF_PREDICTIONS 100

typedef struct{
	double phi_rad;
	double theta_rad;

	double P[4];
	double Q[2];
	double R[3];

}KalmanRollPitch;

void KalmanRollPitch_init(KalmanRollPitch *kal, double Pinit, double *Q, double *R);
void KalmanRollPitch_predict(KalmanRollPitch *kal, double *gyr_rps, double T);
void KalmanRollPitch_update(KalmanRollPitch *kal, double *acc_mps);

#endif // _KALMAN_H_
