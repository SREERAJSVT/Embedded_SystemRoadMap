/*
 * i2c.h
 *
 * Created: 18-05-2026 23:47:03
 *  Author: sreer
 */ 


#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read(uint8_t ack);



#endif /* I2C_H_ */