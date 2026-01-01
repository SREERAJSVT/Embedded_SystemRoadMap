/*
 * dewpoint.c
 *
 *  Created on: 26-Dec-2025
 *      Author: sreer
 */
#include <math.h>

float DewPoint_Compute(float t, float rh) {
    float gamma = logf(rh / 100.0f) + (17.625f * t) / (243.04f + t);
    return (243.04f * gamma) / (17.625f - gamma);
}

