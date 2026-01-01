/*
 * system_init.c
 *
 *  Created on: 26-Dec-2025
 *      Author: sreer
 */
#include "stm32f446xx.h" // Register definitions

void System_Init(void) {
  // 1. Enable Clocks for GPIOA, GPIOB, I2C1, and SPI1
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN);
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  // 2. I2C1 GPIO: PB8 (SCL), PB9 (SDA) -> AF4 [cite: 6]
  GPIOB->MODER |= (2 << GPIO_MODER_MODER8_Pos) | (2 << GPIO_MODER_MODER9_Pos);
  GPIOB->OTYPER |=
      (1 << GPIO_OTYPER_OT8_Pos) | (1 << GPIO_OTYPER_OT9_Pos); // Open-Drain
  GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL8_Pos) | (4 << GPIO_AFRH_AFSEL9_Pos);

  // 3. SPI1 GPIO: PA5 (SCK), PA6 (MISO), PA7 (MOSI) -> AF5 [cite: 7]
  GPIOA->MODER |= (2 << GPIO_MODER_MODER5_Pos) | (2 << GPIO_MODER_MODER6_Pos) |
                  (2 << GPIO_MODER_MODER7_Pos);
  GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) |
                   (5 << GPIO_AFRL_AFSEL7_Pos);

  // PA4 as CS (Output) [cite: 7]
  GPIOA->MODER |= (1 << GPIO_MODER_MODER4_Pos);
  GPIOA->BSRR = GPIO_BSRR_BS4; // CS High

  // 4. Configure I2C1 (Standard Mode 100kHz @ 16MHz HSI)
  I2C1->CR2 |= 16;         // FREQ = 16MHz
  I2C1->CCR = 80;          // 100kHz
  I2C1->TRISE = 17;        // Max rise time
  I2C1->CR1 |= I2C_CR1_PE; // Peripheral Enable

  // 5. Configure USART2 (PA2=TX, PA3=RX)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  // Configure PA2 (TX) and PA3 (RX) as AF7
  GPIOA->MODER &=
      ~((3 << GPIO_MODER_MODER2_Pos) | (3 << GPIO_MODER_MODER3_Pos));
  GPIOA->MODER |= ((2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos));
  GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);

  // Configure USART2: 115200 Baud @ 16MHz
  // BRR = 16MHz / 115200 = 138.88 -> 139 (0x8B)
  USART2->BRR = 0x8B;
  USART2->CR1 |=
      USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; // Enable USART, TX, RX
}
