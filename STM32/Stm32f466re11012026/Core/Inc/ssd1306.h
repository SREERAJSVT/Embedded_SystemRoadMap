#ifndef SH1106_H
#define SH1106_H

#include "stm32f4xx_hal.h"

#define SH1106_I2C_ADDR   0x3C
#define SH1106_WIDTH      128
#define SH1106_HEIGHT     32

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t buffer[SH1106_WIDTH * SH1106_HEIGHT / 8];
    uint8_t x;
    uint8_t y;
} SH1106_HandleTypeDef;

// Commands
#define SH1106_SETCONTRAST 0x81
#define SH1106_DISPLAYALLON_RESUME 0xA4
#define SH1106_DISPLAYALLON 0xA5
#define SH1106_NORMALDISPLAY 0xA6
#define SH1106_INVERTDISPLAY 0xA7
#define SH1106_DISPLAYOFF 0xAE
#define SH1106_DISPLAYON 0xAF
#define SH1106_SETDISPLAYOFFSET 0xD3
#define SH1106_SETCOMPINS 0xDA
#define SH1106_SETVCOMDETECT 0xDB
#define SH1106_SETDISPLAYCLOCKDIV 0xD5
#define SH1106_SETPRECHARGE 0xD9
#define SH1106_SETMULTIPLEX 0xA8
#define SH1106_SETLOWCOLUMN 0x00
#define SH1106_SETHIGHCOLUMN 0x10
#define SH1106_SETSTARTLINE 0x40
#define SH1106_MEMORYMODE 0x20
#define SH1106_COLUMNADDR 0x21
#define SH1106_PAGEADDR   0x22
#define SH1106_COMSCANINC 0xC0
#define SH1106_COMSCANDEC 0xC8
#define SH1106_SEGREMAP 0xA0
#define SH1106_CHARGEPUMP 0x8D

// Function prototypes
void SH1106_Init(SH1106_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c);
void SH1106_DrawPixel(SH1106_HandleTypeDef *hdev, uint8_t x, uint8_t y, uint8_t color);
void SH1106_UpdateScreen(SH1106_HandleTypeDef *hdev);
void SH1106_Clear(SH1106_HandleTypeDef *hdev);
void SH1106_DrawChar(SH1106_HandleTypeDef *hdev, char ch, uint8_t x, uint8_t y, uint8_t size);
void SH1106_DrawString(SH1106_HandleTypeDef *hdev, char *str, uint8_t x, uint8_t y, uint8_t size);
void SH1106_SetCursor(SH1106_HandleTypeDef *hdev, uint8_t x, uint8_t y);

#endif
