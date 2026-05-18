/*
 * uart.h
 *
 * Created: 18-05-2026 23:31:35
 *  Author: sreer
 */ 
#ifndef UART_H_
#define UART_H_
#include <avr/io.h>
void uart_init(uint32_t baud);
void uart_transmit(uint8_t data);
uint8_t uart_receive(void);
void uart_print(const char *str);
void uart_println(const char *str);
#endif /* UART_H_ */