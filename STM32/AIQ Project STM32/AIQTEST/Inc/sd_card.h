/*
 * sd_card.h
 *
 *  Created on: 27-Dec-2025
 *      Author: sreer
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include <stdint.h>

// Initialize SD Card (SPI Mode)
uint8_t SD_Init(void);

// Write a single block (512 bytes) to a specific sector
uint8_t SD_WriteBlock(uint32_t sector, const uint8_t *data);

// Read a single block (512 bytes) from a specific sector
uint8_t SD_ReadBlock(uint32_t sector, uint8_t *buffer);

// Simple logging function (appends to a fixed sector for demo)
void SD_Log(const char *str);

#endif /* SD_CARD_H_ */
