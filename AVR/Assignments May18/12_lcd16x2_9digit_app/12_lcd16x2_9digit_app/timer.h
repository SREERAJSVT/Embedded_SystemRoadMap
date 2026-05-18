/*
 * timer.h
 *
 * Created: 18-05-2026 23:23:18
 *  Author: sreer
 * Timer1?based millisecond delay – header
 */

#ifndef TIMER_H_
#define TIMER_H_
#include <avr/io.h>
extern volatile uint16_t timer1_ticks;
void timer1_init(void);
void delay_ms_timer(uint16_t ms);
#endif /* TIMER_H_ */