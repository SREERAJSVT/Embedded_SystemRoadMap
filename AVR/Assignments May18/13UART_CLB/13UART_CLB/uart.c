/*
 * uart.c
 *
 * Created: 18-05-2026 23:32:13
 *  Author: sreer
*/
#define F_CPU 16000000UL 
#include "uart.h"

void uart_init(uint32_t baud)
{
    uint16_t ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8?bit, 1 stop
}

void uart_transmit(uint8_t data)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

uint8_t uart_receive(void)
{
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

void uart_print(const char *str)
{
    while (*str) uart_transmit(*str++);
}

void uart_println(const char *str)
{
    uart_print(str);
    uart_transmit('\r');
    uart_transmit('\n');
}