/*
 * sensirion_voc_algorithm.h
 *
 *  Created on: 27-Dec-2025
 *      Author: sreer
 */

#ifndef SENSIRION_VOC_ALGORITHM_H_
#define SENSIRION_VOC_ALGORITHM_H_

#include <stdint.h>
typedef struct { int32_t dummy; } VocAlgorithmParams;
void VocAlgorithm_init(VocAlgorithmParams* params);
int32_t VocAlgorithm_process(VocAlgorithmParams* params, uint16_t sraw);

#endif /* SENSIRION_VOC_ALGORITHM_H_ */
