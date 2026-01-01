#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void I2C1_Init(void);
void I2C1_Write(uint8_t addr, uint8_t control, uint8_t data);

#endif
