/*
 * timer.c
 *
 * Created: 18-05-2026 23:23:07
 *  Author: sreer
 * Timer1 driver – NO main() function here!
 */
#include "timer.h"
#include <avr/interrupt.h>

volatile uint16_t timer1_ticks;   // <-- removed "static"

void timer1_init(void)
{
	TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A = 249;
	TIMSK1 |= (1 << OCIE1A);
	timer1_ticks = 0;
	sei();
}

void delay_ms_timer(uint16_t ms)
{
    timer1_ticks = 0;
    while (timer1_ticks < ms);       // wait until enough ticks passed
}

ISR(TIMER1_COMPA_vect)
{
    timer1_ticks++;                  // increment every 1 ms
}