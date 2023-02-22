/*
 * Biquad.h
 *
 *  Created on: Feb 22, 2023
 *      Author: sch_b
 */

#ifndef BIQUAD_BIQUAD_H_
#define BIQUAD_BIQUAD_H_

typedef struct {
    double b0, b1, b2, a1, a2; // filter coefficients
    double x1, x2, y1, y2;     // filter state variables
} BiQuad;

void BiQuad_init(BiQuad* bq, double b[3], double a[3]);

double BiQuad_filter(BiQuad* bq, double input);


#endif /* BIQUAD_BIQUAD_H_ */
