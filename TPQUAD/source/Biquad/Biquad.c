/*
 * Biquad.c
 *
 *  Created on: Feb 22, 2023
 *      Author: sch_b
 */
#include "Biquad.h"



void BiQuad_init(BiQuad* bq, double b[3], double a[3]) {

    double b0 = b[0];
    double b1 = b[1];
    double b2 = b[2];
    double a0 = a[0];
    double a1 = a[1];
    double a2 = a[2];

    bq->b0 = b0/a0;
    bq->b1 = b1/a0;
    bq->b2 = b2/a0;
    bq->a1 = a1/a0;
    bq->a2 = a2/a0;

    bq->x1 = bq->x2 = 0;
    bq->y1 = bq->y2 = 0;
}

double BiQuad_filter(BiQuad* bq, double input) {
    double output = bq->b0 * input + bq->b1 * bq->x1 + bq->b2 * bq->x2
                 - bq->a1 * bq->y1 - bq->a2 * bq->y2;

    bq->x2 = bq->x1;
    bq->x1 = input;
    bq->y2 = bq->y1;
    bq->y1 = output;

    return output;
}


