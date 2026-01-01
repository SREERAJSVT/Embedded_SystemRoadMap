/*
 * sgp40.h
 *
 *  Created on: 27-Dec-2025
 *      Author: sreer
 */

#ifndef SGP40_H_
#define SGP40_H_

#include <stdint.h>

void SGP40_Init(void);
uint16_t SGP40_GetRaw(float t, float rh);

#endif /* SGP40_H_ */
