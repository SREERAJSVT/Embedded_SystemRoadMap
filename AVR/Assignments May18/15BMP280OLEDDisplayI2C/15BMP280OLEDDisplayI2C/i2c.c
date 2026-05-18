/*
 * i2c.c
 *
 * Created: 18-05-2026 23:46:46
 *  Author: sree
 * Software I2C using PC4 (SDA), PC5 (SCL)
 */

/*
 * i2c.c
 * Software I2C using PC4 (SDA), PC5 (SCL)
 */

#define F_CPU 16000000UL          // <-- ADD THIS
#include "i2c.h"
#include <util/delay.h>

#define SDA PC4
#define SCL PC5
#define I2C_DDR DDRC
#define I2C_PORT PORTC
#define I2C_PIN PINC

static void sda_high() { I2C_DDR &= ~(1<<SDA); }
static void sda_low()  { I2C_DDR |= (1<<SDA); I2C_PORT &= ~(1<<SDA); }
static void scl_high() { I2C_DDR &= ~(1<<SCL); }
static void scl_low()  { I2C_DDR |= (1<<SCL); I2C_PORT &= ~(1<<SCL); }

void i2c_init(void)
{
    I2C_PORT |= (1<<SDA) | (1<<SCL);
    sda_high();
    scl_high();
}

void i2c_start(void)
{
    sda_high();
    scl_high();
    _delay_us(5);
    sda_low();
    _delay_us(5);
    scl_low();
}

void i2c_stop(void)
{
    sda_low();
    scl_high();
    _delay_us(5);
    sda_high();
    _delay_us(5);
}

uint8_t i2c_write(uint8_t data)
{
    for (uint8_t i=0; i<8; i++)
    {
        if (data & 0x80) sda_high(); else sda_low();
        _delay_us(2);
        scl_high();
        _delay_us(5);
        scl_low();
        data <<= 1;
    }
    sda_high();
    scl_high();
    _delay_us(2);
    uint8_t ack = !(I2C_PIN & (1<<SDA));
    scl_low();
    return ack;
}

uint8_t i2c_read(uint8_t ack)
{
    uint8_t data = 0;
    sda_high();
    for (uint8_t i=0; i<8; i++)
    {
        scl_high();
        _delay_us(2);
        data = (data << 1) | ((I2C_PIN & (1<<SDA)) ? 1 : 0);
        scl_low();
        _delay_us(2);
    }
    if (ack) sda_low(); else sda_high();
    scl_high();
    _delay_us(2);
    scl_low();
    sda_high();
    return data;
}