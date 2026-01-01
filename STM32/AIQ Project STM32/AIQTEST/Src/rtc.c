/*
 * rtc.c
 *
 *  Created on: 27-Dec-2025
 *      Author: sreer
 */

#include "rtc.h"
#include "stm32f446xx.h"

#define RTC_ADDR (0x68 << 1)

// Helper to convert Decimal to BCD
static uint8_t decToBcd(int val) {
  return (uint8_t)((val / 10 * 16) + (val % 10));
}

// Helper to convert BCD to Decimal
static uint8_t bcdToDec(uint8_t val) {
  return (int)((val / 16 * 10) + (val % 16));
}

void RTC_Init(void) {
  // I2C1 is already initialized in System_Init
  // We can add specific RTC config here if needed (e.g. 24h mode)
}

void RTC_GetTime(RTC_Time *time) {
  // Write register address 0x00
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB))
    ;
  I2C1->DR = RTC_ADDR;
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
    ;
  (void)I2C1->SR2;

  I2C1->DR = 0x00; // Start at Seconds register
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ;
  while (!(I2C1->SR1 & I2C_SR1_BTF))
    ;

  // Repeated Start to Read
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB))
    ;
  I2C1->DR = RTC_ADDR | 1;
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
    ;
  (void)I2C1->SR2;

  // Read 7 bytes (Sec, Min, Hour, Day, Date, Month, Year)
  I2C1->CR1 |= I2C_CR1_ACK;
  uint8_t buf[7];
  for (int i = 0; i < 7; i++) {
    if (i == 6)
      I2C1->CR1 &= ~I2C_CR1_ACK; // NACK last byte
    while (!(I2C1->SR1 & I2C_SR1_RXNE))
      ;
    buf[i] = I2C1->DR;
  }
  I2C1->CR1 |= I2C_CR1_STOP;

  time->seconds = bcdToDec(buf[0] & 0x7F);
  time->minutes = bcdToDec(buf[1]);
  time->hours = bcdToDec(buf[2] & 0x3F); // 24h format assumption
  time->day = bcdToDec(buf[3]);
  time->date = bcdToDec(buf[4]);
  time->month = bcdToDec(buf[5]);
  time->year = bcdToDec(buf[6]);
}

void RTC_SetTime(RTC_Time *time) {
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB))
    ;
  I2C1->DR = RTC_ADDR;
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
    ;
  (void)I2C1->SR2;

  I2C1->DR = 0x00; // Start at Seconds register
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ;

  I2C1->DR = decToBcd(time->seconds);
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ;
  I2C1->DR = decToBcd(time->minutes);
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ;
  I2C1->DR = decToBcd(time->hours);
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ;
  I2C1->DR = decToBcd(time->day);
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ;
  I2C1->DR = decToBcd(time->date);
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ;
  I2C1->DR = decToBcd(time->month);
  while (!(I2C1->SR1 & I2C_SR1_TXE))
    ;
  I2C1->DR = decToBcd(time->year);
  while (!(I2C1->SR1 & I2C_SR1_BTF))
    ;

  I2C1->CR1 |= I2C_CR1_STOP;
}
