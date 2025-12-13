#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "i2c.h"
#include "uart.h"
#include "lcd.h"
#include "bmp280.h"

// helper signature for dtostrf (avr-libc)
extern char *dtostrf(double __val, signed char __width, unsigned char __prec, char *__s);
int main(void)
{
	// init peripherals
	uart_init();
	uart_puts("\r\nBMP280 AVR starting...\r\n");

	lcd_init();
	lcd_puts_at(0, 0, "BMP280 - AVR");
	lcd_puts_at(1, 0, "Init...");

	bmp280_set_debug(true);

	if (!bmp280_begin(0x76)) {
		uart_puts("BMP280 init failed\r\n");
		lcd_puts_at(0, 0, "BMP280 not found");
		lcd_puts_at(1, 0, "Check wiring");
		while (1) { _delay_ms(500); }
	}

	char line1[17];
	char line2[17]="C ---------- hPa";
	char tbuf[16], pbuf[16];
	char uart_buf[80];

	while (1) {
		float T = 0.0f, P = 0.0f;
		if (!bmp280_read(&T, &P)) {
			uart_puts("Read error\r\n");
			lcd_puts_at(0, 0, "Read failed");
			_delay_ms(500);
			continue;
		}


		// format using dtostrf
		dtostrf(T, 4, 1, tbuf);   // e.g. "25.3"
		// pressure in hPa for display
		float P_hpa = P / 100.0f;
		dtostrf(P_hpa, 6, 1, pbuf); // e.g. "1013.2"
		// build lines (limit 16 chars)
		snprintf(line1, sizeof(line1), "T:%sC P:%shPa", tbuf, pbuf);
		//snprintf(line2, sizeof(line2), "T:%sC P:%shPa", tbuf, pbuf);
        
		lcd_puts_at(0, 0, line1);
		lcd_puts_at(1, 0, line2);

		// uart debug
		snprintf(uart_buf, sizeof(uart_buf), "T=%s C, P=%s hPa\r\n", tbuf, pbuf);
		uart_puts(uart_buf);
	}

	return 0;
}
/*avrdude -v -patmega328p -carduino -PCOM19 -b115200 -D -Uflash:w:"C:\Users\sreer\OneDrive\Documents\GitHub\Embedded_SystemRoadMap\AVR\BMP280\bmp280 i2c test\I2c_Test\I2c_Test\Debug\I2c_Test.hex":i*/