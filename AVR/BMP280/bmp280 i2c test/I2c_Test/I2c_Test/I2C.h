#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>

void i2c_init(void);
bool i2c_start_addr(uint8_t addr_rw); // sends start + SLA+R/W, returns ack
void i2c_stop(void);
bool i2c_write(uint8_t data);         // returns ack true
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);

bool i2c_read_registers(uint8_t devaddr, uint8_t reg, uint8_t *buf, uint8_t len);
bool i2c_write_register(uint8_t devaddr, uint8_t reg, uint8_t val);

#endif // I2C_H_
