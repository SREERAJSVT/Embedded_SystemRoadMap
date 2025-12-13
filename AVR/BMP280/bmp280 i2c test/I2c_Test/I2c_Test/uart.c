#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include "uart.h"
#include <avr/io.h>

#ifndef BAUD
#define BAUD 115200
#endif

// Using double speed (U2X0) gives better accuracy for some baud rates
static inline void uart_set_ubrr(uint32_t ubrr)
{
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)(ubrr & 0xFF);
}

void uart_init(void)
{
	// Try double speed for better accuracy
	UCSR0A = (1 << U2X0);
	uint32_t ubrr = (F_CPU / (8UL * (uint32_t)BAUD)) - 1;
	uart_set_ubrr(ubrr);

	UCSR0B = (1 << TXEN0); // enable TX only
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit, no parity, 1 stop
}

void uart_putc(char c)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = (uint8_t)c;
}

void uart_puts(const char *s)
{
	while (*s) uart_putc(*s++);
}
