#include "i2c.h"
#include "ssd1306.h"
#include "gfx.h"

int main(void)
{
    I2C1_Init();
    SSD1306_Init();
    SSD1306_Clear();

    GFX_DrawString(0, 0, "Hello STM32");
    GFX_DrawRect(0, 10, 50, 20);
    GFX_DrawLine(0, 31, 127, 31);

    SSD1306_Update();

    while (1);
}
