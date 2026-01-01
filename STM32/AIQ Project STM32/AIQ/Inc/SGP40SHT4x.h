#ifndef SGP40SHT4X_H
#define SGP40SHT4X_H

#include <stdint.h>

// --- Memory Map Base Addresses ---
#define RCC_BASE      0x40023800
#define GPIOA_BASE    0x40020000
#define GPIOB_BASE    0x40020400
#define I2C1_BASE     0x40005400
#define USART2_BASE   0x40004400

// --- RCC Registers ---
#define RCC_AHB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x40))

// --- GPIO Registers ---
#define GPIOA_MODER   (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_AFRL    (*(volatile uint32_t *)(GPIOA_BASE + 0x20))
#define GPIOB_MODER   (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER  (*(volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_PUPDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_AFRH    (*(volatile uint32_t *)(GPIOB_BASE + 0x24))

// --- I2C Registers ---
#define I2C1_CR1      (*(volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2      (*(volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C1_DR       (*(volatile uint32_t *)(I2C1_BASE + 0x10))
#define I2C1_SR1      (*(volatile uint32_t *)(I2C1_BASE + 0x14))
#define I2C1_SR2      (*(volatile uint32_t *)(I2C1_BASE + 0x18))
#define I2C1_CCR      (*(volatile uint32_t *)(I2C1_BASE + 0x1C))
#define I2C1_TRISE    (*(volatile uint32_t *)(I2C1_BASE + 0x20))

// --- USART2 Registers ---
#define USART2_SR     (*(volatile uint32_t *)(USART2_BASE + 0x00))
#define USART2_DR     (*(volatile uint32_t *)(USART2_BASE + 0x04))
#define USART2_BRR    (*(volatile uint32_t *)(USART2_BASE + 0x08))
#define USART2_CR1    (*(volatile uint32_t *)(USART2_BASE + 0x0C))

// --- Device Addresses ---
#define SHT4X_ADDR    0x44
#define SGP40_ADDR    0x59
#define OLED_ADDR     0x3C

// --- Prototypes ---
void System_Init(void);
void Delay(uint32_t count);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Write(uint8_t data);
void I2C1_Address(uint8_t addr);
uint8_t I2C1_ReadAck(void);
uint8_t I2C1_ReadNack(void);

void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t page, uint8_t col);
void OLED_PutString(char *s);

void SHT4x_ReadRaw(uint16_t *temp);
void SGP40_ReadRaw(uint16_t *voc);

#endif
