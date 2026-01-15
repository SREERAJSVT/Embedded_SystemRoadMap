/*
 * sgp40.c
 *
 *  Created on: Jan 15, 2026
 *      Author: sreer
 */

#include "sgp40.h"

static uint8_t SGP40_CRC8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for(uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(uint8_t b = 0; b < 8; b++) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

HAL_StatusTypeDef SGP40_Init(SGP40_HandleTypeDef *hsgp, I2C_HandleTypeDef *hi2c) {
    hsgp->hi2c = hi2c;

    // Soft reset
    uint8_t reset_cmd[2] = {SGP40_CMD_SOFT_RESET >> 8, SGP40_CMD_SOFT_RESET & 0xFF};
    HAL_I2C_Master_Transmit(hsgp->hi2c, SGP40_I2C_ADDR << 1, reset_cmd, 2, HAL_MAX_DELAY);
    HAL_Delay(100);

    // Get serial number (optional)
    return SGP40_GetSerial(hsgp);
}

HAL_StatusTypeDef SGP40_MeasureRaw(SGP40_HandleTypeDef *hsgp, uint16_t *raw_signal) {
    uint8_t cmd[2] = {SGP40_CMD_MEASURE_RAW >> 8, SGP40_CMD_MEASURE_RAW & 0xFF};
    uint8_t rx_data[6];

    // Send command
    if(HAL_I2C_Master_Transmit(hsgp->hi2c, SGP40_I2C_ADDR << 1, cmd, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Wait for measurement (30ms typical)
    HAL_Delay(50);

    // Read data
    if(HAL_I2C_Master_Receive(hsgp->hi2c, SGP40_I2C_ADDR << 1, rx_data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Verify CRC for first two bytes
    uint8_t crc = SGP40_CRC8(&rx_data[0], 2);
    if(crc != rx_data[2]) {
        return HAL_ERROR;
    }

    // Combine bytes
    *raw_signal = (rx_data[0] << 8) | rx_data[1];

    return HAL_OK;
}

HAL_StatusTypeDef SGP40_GetSerial(SGP40_HandleTypeDef *hsgp) {
    uint8_t cmd[2] = {SGP40_CMD_GET_SERIAL >> 8, SGP40_CMD_GET_SERIAL & 0xFF};
    uint8_t rx_data[9];

    HAL_I2C_Master_Transmit(hsgp->hi2c, SGP40_I2C_ADDR << 1, cmd, 2, HAL_MAX_DELAY);
    HAL_Delay(10);
    HAL_I2C_Master_Receive(hsgp->hi2c, SGP40_I2C_ADDR << 1, rx_data, 9, HAL_MAX_DELAY);

    // Verify CRC for each 2-byte word
    for(uint8_t i = 0; i < 3; i++) {
        uint8_t crc = SGP40_CRC8(&rx_data[i * 3], 2);
        if(crc != rx_data[i * 3 + 2]) {
            return HAL_ERROR;
        }
    }

    // Store serial
    for(uint8_t i = 0; i < 6; i++) {
        hsgp->serial[i] = rx_data[i];
    }

    return HAL_OK;
}
