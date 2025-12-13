#include "i2c.h"
#include <avr/io.h>
#include <util/delay.h>

#define SCL_CLOCK 100000UL

void i2c_init(void)
{
	// TWSR prescaler = 1
	TWSR = 0x00;
	// TWBR = (F_CPU/SCL - 16) / 2
	TWBR = (uint8_t)(((F_CPU / SCL_CLOCK) - 16) / 2);
}

bool i2c_start_addr(uint8_t addr_rw)
{
	// start
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	uint8_t status = TWSR & 0xF8;
	if ((status != 0x08) && (status != 0x10)) return false;

	// send address+rw
	TWDR = addr_rw;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	status = TWSR & 0xF8;
	if ((addr_rw & 1) && (status != 0x40)) return false; // SLA+R ack
	if (!(addr_rw & 1) && (status != 0x18)) return false; // SLA+W ack
	return true;
}

void i2c_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	_delay_us(10);
}

bool i2c_write(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	uint8_t status = TWSR & 0xF8;
	return (status == 0x28);
}

uint8_t i2c_read_ack(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

bool i2c_read_registers(uint8_t devaddr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	// start + write address
	if (!i2c_start_addr((devaddr << 1) | 0)) { i2c_stop(); return false; }
	if (!i2c_write(reg)) { i2c_stop(); return false; }
	// repeated start + read address
	if (!i2c_start_addr((devaddr << 1) | 1)) { i2c_stop(); return false; }
	for (uint8_t i = 0; i < len; ++i) {
		if (i < (len - 1)) buf[i] = i2c_read_ack();
		else buf[i] = i2c_read_nack();
	}
	i2c_stop();
	return true;
}

bool i2c_write_register(uint8_t devaddr, uint8_t reg, uint8_t val)
{
	if (!i2c_start_addr((devaddr << 1) | 0)) { i2c_stop(); return false; }
	if (!i2c_write(reg)) { i2c_stop(); return false; }
	if (!i2c_write(val)) { i2c_stop(); return false; }
	i2c_stop();
	return true;
}
