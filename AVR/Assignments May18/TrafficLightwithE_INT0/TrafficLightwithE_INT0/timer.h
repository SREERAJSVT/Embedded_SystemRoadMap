/*
 * timer.h
 *
 * Created: 18-05-2026 22:45:56
 *  Author: sreer
 */ 


#ifndef TIMER_H_
#define TIMER_H_


#include <avr/io.h>

void timer1_init(void);
void delay_ms_timer(uint16_t ms);


#endif /* TIMER_H_ */