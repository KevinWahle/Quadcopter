/*
 * Biquad.c
 *
 *  Created on: Feb 22, 2023
 *      Author: sch_b
 */
#include "Biquad.h"



void BiQuad_init(BiQuad* bq) {

    double b0 = 0.8549;
    double b1 = -1.0693;
    double b2 = 0.8226;
    double a0 = 1.0000;
    double a1 = -1.0693;
    double a2 = 0.6775;

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


