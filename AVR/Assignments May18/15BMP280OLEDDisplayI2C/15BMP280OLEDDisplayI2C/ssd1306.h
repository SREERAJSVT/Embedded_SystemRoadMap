/*
 * ssd1306.hbasic 128x64 I2C OLED ;
 *
 * Created: 18-05-2026 23:48:07
 *  Author: sreer
 */ 



#ifndef SSD1306_H_
#define SSD1306_H_
#include <stdint.h>    
void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_set_cursor(uint8_t x, uint8_t y);
void ssd1306_print(const char *str);

#endif