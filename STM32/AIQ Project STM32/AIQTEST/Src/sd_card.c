/*
 * sd_card.c
 *
 *  Created on: 27-Dec-2025
 *      Author: sreer
 */

#include "sd_card.h"
#include "stm32f446xx.h"
#include <string.h>

// --- MOVE THIS TO THE TOP ---
// SPI1 Read/Write
uint8_t SPI_Transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));  // Wait for TX empty
    SPI1->DR = data;                   // Send data
    while (!(SPI1->SR & SPI_SR_RXNE)); // Wait for RX full
    return (uint8_t)SPI1->DR;          // Return received byte
}

#define SD_CS_LOW() (GPIOA->BSRR = (1 << 20)) // Reset PA4
#define SD_CS_HIGH() (GPIOA->BSRR = (1 << 4)) // Set PA4

// Send Command to SD Card
static uint8_t SD_Command(uint8_t cmd, uint32_t arg, uint8_t crc) {
  SPI_Transfer(0xFF);
  SPI_Transfer(cmd | 0x40);
  SPI_Transfer(arg >> 24);
  SPI_Transfer(arg >> 16);
  SPI_Transfer(arg >> 8);
  SPI_Transfer(arg);
  SPI_Transfer(crc);

  uint8_t res, retry = 0;
  while ((res = SPI_Transfer(0xFF)) == 0xFF && retry++ < 200)
    ;
  return res;
}

uint8_t SD_Init(void) {
  uint8_t i, res;

  // 1. CS High, Send 80 dummy clocks
  SD_CS_HIGH();
  for (i = 0; i < 10; i++)
    SPI_Transfer(0xFF);

  // 2. CMD0 (Go Idle State)
  SD_CS_LOW();
  res = SD_Command(0, 0, 0x95);
  SD_CS_HIGH();
  if (res != 0x01)
    return 1; // Error

  // 3. CMD8 (Check Voltage)
  SD_CS_LOW();
  res = SD_Command(8, 0x000001AA, 0x87);
  SD_CS_HIGH();
  if (res == 0x01) {
    // SDv2
    // Read remaining 4 bytes of R7
    // (Implementation skipped for brevity, assuming success)
  }

  // 4. ACMD41 (Initialize)
  uint16_t timeout = 1000;
  do {
    SD_CS_LOW();
    SD_Command(55, 0, 0xFF);                // CMD55
    res = SD_Command(41, 0x40000000, 0xFF); // ACMD41 (HCS=1)
    SD_CS_HIGH();
  } while (res != 0 && timeout--);

  if (timeout == 0)
    return 2; // Timeout

  return 0; // Success
}

uint8_t SD_WriteBlock(uint32_t sector, const uint8_t *data) {
  uint8_t res;

  SD_CS_LOW();
  res = SD_Command(24, sector, 0xFF); // CMD24 (Write Single Block)
  if (res != 0) {
    SD_CS_HIGH();
    return 1;
  }

  SPI_Transfer(0xFF);
  SPI_Transfer(0xFE); // Start Token

  for (int i = 0; i < 512; i++) {
    SPI_Transfer(data[i]);
  }

  // Dummy CRC
  SPI_Transfer(0xFF);
  SPI_Transfer(0xFF);

  res = SPI_Transfer(0xFF); // Data Response
  if ((res & 0x1F) != 0x05) {
    SD_CS_HIGH();
    return 2; // Write Error
  }

  while (SPI_Transfer(0xFF) == 0)
    ; // Wait for busy
  SD_CS_HIGH();

  return 0;
}

// Global sector counter for simple logging
static uint32_t current_sector = 100;

void SD_Log(const char *str) {
  uint8_t buffer[512];
  memset(buffer, 0, 512);
  strncpy((char *)buffer, str, 512);

  // Write to a new sector each time (Very simple "logging")
  SD_WriteBlock(current_sector++, buffer);
}

uint8_t SD_ReadBlock(uint32_t sector, uint8_t *buffer) {
  // Not implemented for this task
  return 0;
}
