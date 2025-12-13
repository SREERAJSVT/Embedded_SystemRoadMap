#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>
#include <stdbool.h>

void lcd_init(void);
void lcd_clear(void);
void lcd_puts_at(uint8_t row, uint8_t col, const char *s);
#endif // LCD_H_
