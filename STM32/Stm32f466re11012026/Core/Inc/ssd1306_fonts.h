/*
 * ssd1306_fonts.h
 *
 *  Created on: Jan 11, 2026
 *      Author: sreer
 */

#ifndef __SSD1306_FONTS_H__
#define __SSD1306_FONTS_H__

#include <stdint.h>

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;

extern FontDef Font_7x10;
extern FontDef Font_11x18;

#endif
