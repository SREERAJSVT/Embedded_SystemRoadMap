/*
 *lcd.c  (4?bit mode, PB0?PB5)
 *
 * Created: 18-05-2026 23:20:14
 *  Author: sreer
 */ 

#include "lcd.h"
#include <util/delay.h>

#define RS PB0
#define EN PB1
#define D4 PB2
#define D5 PB3
#define D6 PB4
#define D7 PB5

static void lcd_nibble(uint8_t data)
{
    PORTB = (PORTB & 0xC0) | (data & 0x0F);
    PORTB |= (1 << EN);
    _delay_us(1);
    PORTB &= ~(1 << EN);
    _delay_us(100);
}

static void lcd_write(uint8_t data, uint8_t rs)
{
    if (rs) PORTB |= (1 << RS);
    else    PORTB &= ~(1 << RS);
    lcd_nibble(data >> 4);
    lcd_nibble(data);
}

void lcd_init(void)
{
    DDRB |= (1<<RS) | (1<<EN) | (1<<D4) | (1<<D5) | (1<<D6) | (1<<D7);
    _delay_ms(50);
    PORTB &= ~(1<<RS);
    lcd_nibble(0x03); _delay_ms(5);
    lcd_nibble(0x03); _delay_us(150);
    lcd_nibble(0x03);
    lcd_nibble(0x02);               // 4?bit mode
    lcd_write(0x28, 0);             // 2 lines, 5x8
    lcd_write(0x0C, 0);             // display on, no cursor
    lcd_write(0x06, 0);             // increment mode
    lcd_clear();
}

void lcd_clear(void)
{
    lcd_write(0x01, 0);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    lcd_write(0x80 | (addr + col), 0);
}

void lcd_print(const char *str)
{
    while (*str)
        lcd_write(*str++, 1);
}

void lcd_print_number(uint32_t num, uint8_t digits)
{
    char buf[10];
    for (int8_t i = digits-1; i >= 0; i--)
    {
        buf[i] = '0' + (num % 10);
        num /= 10;
    }
    buf[digits] = '\0';
    lcd_print(buf);
}
