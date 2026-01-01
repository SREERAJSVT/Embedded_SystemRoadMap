/*
 * voc_stub.c
 *
 *  Created on: 27-Dec-2025
 *      Author: sreer
 */


#include "sensirion_voc_algorithm.h"

void VocAlgorithm_init(VocAlgorithmParams* params) {
    // Empty stub to satisfy the linker
}

int32_t VocAlgorithm_process(VocAlgorithmParams* params, uint16_t sraw) {
    return (int32_t)sraw; // Just return raw value for now
}
