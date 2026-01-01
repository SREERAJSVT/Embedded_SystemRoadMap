/*
 * sgp40.c
 *
 *  Created on: 27-Dec-2025
 *      Author: sreer
 */

#include "sgp40.h"
#include "stm32f446xx.h"

#define SGP40_ADDR (0x59 << 1)

static uint8_t SGP40_CRC8(uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void SGP40_Init(void) {
  // Soft Reset or Self-Test could go here
  // For now, just wait a bit for sensor to be ready
  for (volatile int i = 0; i < 100000; i++)
    ;
}

uint16_t SGP40_GetRaw(float t, float rh) {
  uint16_t t_ticks = (uint16_t)(((t + 45.0f) * 65535.0f) / 175.0f);
  uint16_t rh_ticks = (uint16_t)(((rh + 6.0f) * 65535.0f) / 125.0f);

  uint8_t cmd[8];
  cmd[0] = 0x26; // Measure Raw Command MSB
  cmd[1] = 0x0F; // Measure Raw Command LSB

  cmd[2] = (rh_ticks >> 8) & 0xFF;
  cmd[3] = rh_ticks & 0xFF;
  uint8_t rh_crc_buf[2] = {cmd[2], cmd[3]};
  cmd[4] = SGP40_CRC8(rh_crc_buf, 2);

  cmd[5] = (t_ticks >> 8) & 0xFF;
  cmd[6] = t_ticks & 0xFF;
  uint8_t t_crc_buf[2] = {cmd[5], cmd[6]};
  cmd[7] = SGP40_CRC8(t_crc_buf, 2);

  // Send Command + Args
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB))
    ;
  I2C1->DR = SGP40_ADDR;
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
    ;
  (void)I2C1->SR2;

  for (int i = 0; i < 8; i++) {
    I2C1->DR = cmd[i];
    while (!(I2C1->SR1 & I2C_SR1_BTF))
      ;
  }
  I2C1->CR1 |= I2C_CR1_STOP;

  // Wait 30ms
  for (volatile int i = 0; i < 300000; i++)
    ;

  // Read 3 bytes (Word + CRC)
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB))
    ;
  I2C1->DR = SGP40_ADDR | 1;
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
    ;
  (void)I2C1->SR2;

  I2C1->CR1 |= I2C_CR1_ACK;
  uint8_t buf[3];
  for (int i = 0; i < 3; i++) {
    if (i == 2)
      I2C1->CR1 &= ~I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_RXNE))
      ;
    buf[i] = I2C1->DR;
  }
  I2C1->CR1 |= I2C_CR1_STOP;

  // Verify CRC (Optional but good practice)
  // uint8_t read_crc = SGP40_CRC8(buf, 2);
  // if (read_crc != buf[2]) return 0;

  return (buf[0] << 8) | buf[1];
}
