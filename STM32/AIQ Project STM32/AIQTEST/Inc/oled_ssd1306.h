/*
 * oled_ssd1306.h
 *
 *  Created on: 26-Dec-2025
 *      Author: sreer
 */

#ifndef OLED_SSD1306_H_
#define OLED_SSD1306_H_

#include "rtc.h"
#include <stdint.h>


void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t page, uint8_t col);
void OLED_Char(char c);
void OLED_String(char *str);
void OLED_Display_Metrics(RTC_Time *time, float t, float rh, int32_t voc);

#endif /* OLED_SSD1306_H_ */
