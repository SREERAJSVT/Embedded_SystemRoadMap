#include "ssd1306.h"
#include "i2c.h"

uint8_t SSD1306_Buffer[128 * 4];

void SSD1306_WriteCmd(uint8_t cmd)
{
    I2C1_Write(SSD1306_ADDR << 1, 0x00, cmd);
}

void SSD1306_WriteData(uint8_t data)
{
    I2C1_Write(SSD1306_ADDR << 1, 0x40, data);
}

void SSD1306_Init(void)
{
    SSD1306_WriteCmd(0xAE);
    SSD1306_WriteCmd(0xA8);
    SSD1306_WriteCmd(0x1F);
    SSD1306_WriteCmd(0xAF);
}

void SSD1306_Clear(void)
{
    for (int i = 0; i < sizeof(SSD1306_Buffer); i++)
        SSD1306_Buffer[i] = 0;
}

void SSD1306_Update(void)
{
    for (uint8_t page = 0; page < 4; page++)
    {
        SSD1306_WriteCmd(0xB0 + page);
        SSD1306_WriteCmd(0x00);
        SSD1306_WriteCmd(0x10);

        for (uint8_t col = 0; col < 128; col++)
            SSD1306_WriteData(SSD1306_Buffer[page * 128 + col]);
    }
}
