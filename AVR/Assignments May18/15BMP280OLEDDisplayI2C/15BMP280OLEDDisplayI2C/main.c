/*
 * 15BMP280OLEDDisplayI2C.c
 *
 * Created: 18-05-2026 23:45:30
 * Author : sreer
  * BMP280 + OLED, display temperature every 2s */ 

#define F_CPU 16000000UL          // <-- ADD THIS
#include "bmp280.h"
#include "ssd1306.h"
#include "i2c.h"
#include <util/delay.h>
#include <stdio.h>

int main(void)
{
    i2c_init();
    ssd1306_init();
    ssd1306_clear();

    if (!bmp280_init())
    {
        ssd1306_set_cursor(0,0);
        ssd1306_print("BMP ERR");
        while(1);
    }

    char buf[20];
    while (1)
    {
        float temp = bmp280_read_temperature();
        ssd1306_clear();
        ssd1306_set_cursor(0,2);
        sprintf(buf, "Temp: %.1f C", temp);
        ssd1306_print(buf);
        _delay_ms(2000);
    }
}
