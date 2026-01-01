#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

#define SSD1306_ADDR 0x3C

void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_Update(void);

void SSD1306_WriteCmd(uint8_t cmd);
void SSD1306_WriteData(uint8_t data);

#endif
