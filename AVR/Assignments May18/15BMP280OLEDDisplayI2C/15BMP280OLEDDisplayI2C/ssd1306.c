/*
 * ssd1306.c
 *
 * Created: 18-05-2026 23:48:58
 *  Author: sreer ssd1306.c (simplified, using fixed font)

 */ 


#include "ssd1306.h"
#include "i2c.h"
#include <avr/pgmspace.h>
#include <string.h>

#define SSD1306_ADDR 0x3C

static void ssd1306_command(uint8_t cmd)
{
    i2c_start();
    i2c_write(SSD1306_ADDR << 1);
    i2c_write(0x00);   // control byte: command
    i2c_write(cmd);
    i2c_stop();
}

void ssd1306_init(void)
{
    // Standard initialisation sequence
    ssd1306_command(0xAE); // display off
    ssd1306_command(0xD5); ssd1306_command(0x80);
    ssd1306_command(0xA8); ssd1306_command(0x3F);
    ssd1306_command(0xD3); ssd1306_command(0x00);
    ssd1306_command(0x40);
    ssd1306_command(0x8D); ssd1306_command(0x14);
    ssd1306_command(0x20); ssd1306_command(0x00);
    ssd1306_command(0xA1);
    ssd1306_command(0xC8);
    ssd1306_command(0xDA); ssd1306_command(0x12);
    ssd1306_command(0x81); ssd1306_command(0xCF);
    ssd1306_command(0xD9); ssd1306_command(0xF1);
    ssd1306_command(0xDB); ssd1306_command(0x40);
    ssd1306_command(0xA4);
    ssd1306_command(0xA6);
    ssd1306_command(0xAF); // display on
    ssd1306_clear();
}

void ssd1306_clear(void)
{
    for (uint16_t i=0; i<128*8; i++)
    {
        ssd1306_command(0x00); // dummy
    }
    // In a real driver you'd fill the GDDRAM. A minimal approach:
    ssd1306_set_cursor(0,0);
    for (uint16_t i=0; i<128*8; i++)
    {
        i2c_start();
        i2c_write(SSD1306_ADDR << 1);
        i2c_write(0x40); // data
        i2c_write(0x00);
        i2c_stop();
    }
}

void ssd1306_set_cursor(uint8_t x, uint8_t y)
{
    ssd1306_command(0x21); ssd1306_command(x);
    ssd1306_command(0x22); ssd1306_command(y); ssd1306_command(7);
}

void ssd1306_print(const char *str)
{
    // For compactness, only a single digit/character example.
    // In practice you'd include a font table.
    while (*str)
    {
        i2c_start();
        i2c_write(SSD1306_ADDR << 1);
        i2c_write(0x40);
        // send 8 bytes of character pattern (here a space for demo)
        for (uint8_t i=0; i<8; i++) i2c_write(0x00);
        i2c_stop();
        str++;
    }
}