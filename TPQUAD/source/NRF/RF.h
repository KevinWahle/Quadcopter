/*
 * RF.h
 *
 *  Created on: Mar 1, 2023
 *      Author: sch_b
 */

#ifndef NRF_RF_H_
#define NRF_RF_H_
#include <stdint.h>
typedef struct{
	double throttle;
	double pitch;
	double roll;
	double yaw;
}OrientationRF;


void RFinit();
void RFcalibrate();
void RFbegin();

void RFgetDeNormalizedData(OrientationRF * dataPtr);
void RF2Newton(OrientationRF * dataPtr);


#endif /* NRF_RF_H_ */
