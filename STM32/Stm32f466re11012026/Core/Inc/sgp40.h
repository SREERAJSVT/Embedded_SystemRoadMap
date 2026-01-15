/*
 * sgp40.h
 *
 *  Created on: Jan 15, 2026
 *      Author: sreer
 */

#ifndef SGP40_H
#define SGP40_H

#include "stm32f4xx_hal.h"

#define SGP40_I2C_ADDR    0x59
#define SGP40_CMD_MEASURE_RAW 0x260F
#define SGP40_CMD_HEATER_OFF 0x3615
#define SGP40_CMD_GET_SERIAL 0x3682
#define SGP40_CMD_SELF_TEST 0x280E
#define SGP40_CMD_SOFT_RESET 0x0006

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t serial[6];
    uint16_t raw_signal;
    uint32_t voc_index;
} SGP40_HandleTypeDef;

HAL_StatusTypeDef SGP40_Init(SGP40_HandleTypeDef *hsgp, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SGP40_MeasureRaw(SGP40_HandleTypeDef *hsgp, uint16_t *raw_signal);
HAL_StatusTypeDef SGP40_GetSerial(SGP40_HandleTypeDef *hsgp);
HAL_StatusTypeDef SGP40_SelfTest(SGP40_HandleTypeDef *hsgp);
void SGP40_CalculateVOC(SGP40_HandleTypeDef *hsgp, float temperature, float humidity);

#endif
