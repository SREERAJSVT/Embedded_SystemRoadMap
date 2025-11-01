#include "UART_H.h"

void UART_Init(void)
{
	UBRR0H = (unsigned char)(UBRR_VALUE >> 8);
	UBRR0L = (unsigned char)UBRR_VALUE;
	UCSR0B = (1 << TXEN0);                      // Enable transmitter
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // 8-bit data
}

void UART_TxChar(char data)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void UART_TxString(const char *str)
{
	while (*str)
	{
		UART_TxChar(*str++);
	}
}

void UART_TxHex(uint8_t value)
{
	const char hexChars[] = "0123456789ABCDEF";
	char hex[3];
	hex[0] = hexChars[(value >> 4) & 0x0F];
	hex[1] = hexChars[value & 0x0F];
	hex[2] = '\0';
	UART_TxString("0x");
	UART_TxString(hex);
	UART_TxString("\r\n");
}

void UART_TxNumber(uint32_t num)
{
	char buffer[12];
	ltoa(num, buffer, 10);
	UART_TxString(buffer);
}
