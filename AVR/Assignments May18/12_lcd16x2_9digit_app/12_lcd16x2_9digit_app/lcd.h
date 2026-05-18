/*
 * lcd.h
 *
 * Created: 18-05-2026 23:19:43
 *  Author: sreer
 */ 
#ifndef LCD_H_
#define LCD_H_

#include <avr/io.h>

void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char *str);
void lcd_print_number(uint32_t num, uint8_t digits);

#endif