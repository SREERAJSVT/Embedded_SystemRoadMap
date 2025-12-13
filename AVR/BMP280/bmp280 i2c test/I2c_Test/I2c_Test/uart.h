#ifndef UART_H_
#define UART_H_

#include <stdint.h>

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);

#endif // UART_H_
