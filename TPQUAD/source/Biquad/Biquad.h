/*
 * Biquad.h
 *
 *  Created on: Feb 22, 2023
 *      Author: sch_b
 */

#ifndef BIQUAD_BIQUAD_H_
#define BIQUAD_BIQUAD_H_

typedef struct {
	float b0, b1, b2, a1, a2; // filter coefficients
	float x1, x2, y1, y2;     // filter state variables
} BiQuad;

void BiQuad_init(BiQuad* bq, float b[3], float a[3]);

float BiQuad_filter(BiQuad* bq, float input);


#endif /* BIQUAD_BIQUAD_H_ */
